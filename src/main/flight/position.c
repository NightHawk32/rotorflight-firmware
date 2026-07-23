/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"
#include "common/time.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/position.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#ifdef USE_RANGEFINDER
#include "sensors/rangefinder.h"
#endif

#ifdef USE_OPTICAL_FLOW
#include "sensors/optical_flow.h"
#endif

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

// Rangefinder reliability decay constant (time constant ~400ms at 50Hz)
#define AGL_RELIABILITY_INCREMENT   0.05f
#define AGL_RELIABILITY_DECREMENT   0.10f
#define AGL_RELIABILITY_THRESHOLD   0.33f
// Optical flow quality threshold (0-255)
#define FLOW_QUALITY_THRESHOLD      50
// Maximum dead-reckoning radius before position resets (cm)
#define POSXY_MAX_DEADRECKONING_CM  1000.0f


typedef struct {

    uint8_t     source;

    float       altitude;
    float       variometer;

    bool        haveBaroAlt;
    bool        haveGpsAlt;

    float       baroAlt;
    float       baroAltOffset;

    float       gpsAlt;
    float       gpsAltOffset;

    difFilter_t varioFilter;

    filter_t    gpsFilter;
    filter_t    baroFilter;

    filter_t    gpsOffsetFilter;
    filter_t    baroOffsetFilter;

} altState_t;

static FAST_DATA altState_t alt;

#ifdef USE_RANGEFINDER
typedef struct {
    float       aglAlt;         // AGL altitude in meters
    float       aglVario;       // AGL vertical velocity in m/s
    float       reliability;    // 0.0 = invalid, 1.0 = perfect
    difFilter_t varioFilter;
} aglState_t;

static FAST_DATA aglState_t agl;
#endif

#ifdef USE_OPTICAL_FLOW
typedef struct {
    float       posX;           // Dead-reckoning position East  (cm, relative to arm point)
    float       posY;           // Dead-reckoning position North (cm, relative to arm point)
    float       velX;           // Earth-frame velocity East  (cm/s)
    float       velY;           // Earth-frame velocity North (cm/s)
    bool        valid;
    timeMs_t    lastUpdateMs;
} posXYState_t;

static FAST_DATA posXYState_t posxy;
#endif


float getAltitude(void)
{
    return alt.altitude;
}

float getVario(void)
{
    return alt.variometer;
}

int getEstimatedAltitudeCm(void)
{
    return lrintf(alt.altitude * 100);
}

int getEstimatedVarioCms(void)
{
    return lrintf(alt.variometer * 100);
}

#ifdef USE_RANGEFINDER
float getAGLAltitude(void)
{
    return agl.aglAlt;
}

float getAGLVario(void)
{
    return agl.aglVario;
}

bool isAGLAltitudeValid(void)
{
    return (agl.reliability >= AGL_RELIABILITY_THRESHOLD) &&
           rangefinderIsHealthy();
}

float getAGLReliability(void)
{
    return agl.reliability;
}
#endif

#ifdef USE_OPTICAL_FLOW
float getPositionXCm(void)
{
    return posxy.posX;
}

float getPositionYCm(void)
{
    return posxy.posY;
}

float getVelocityXCms(void)
{
    return posxy.velX;
}

float getVelocityYCms(void)
{
    return posxy.velY;
}

bool isPositionXYValid(void)
{
    return posxy.valid;
}
#endif


static float calculateVario(float altitude)
{
    return difFilterApply(&alt.varioFilter, altitude);
}

void positionUpdate(void)
{
#ifdef USE_BARO
    if (alt.source == ALT_SOURCE_DEFAULT || alt.source == ALT_SOURCE_BARO_ONLY) {
        if (sensors(SENSOR_BARO) && baroIsReady()) {
            if (baro.baroAltitude < 15000 && baro.baroAltitude > -2500) {
                alt.baroAlt = filterApply(&alt.baroFilter, baro.baroAltitude / 100.0f);
                alt.haveBaroAlt = true;
            }
        }
        else {
            alt.haveBaroAlt = false;
        }
    }
    else {
        // Source doesn't use baro (e.g. GPS_ONLY / LIDAR_ONLY): don't carry a stale reading
        alt.haveBaroAlt = false;
    }
#endif

#ifdef USE_GPS
    if (alt.source & ALT_SOURCE_DEFAULT || alt.source == ALT_SOURCE_GPS_ONLY) {
        if (sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= positionConfig()->gps_min_sats) {
            alt.gpsAlt = filterApply(&alt.gpsFilter, gpsSol.llh.altCm / 100.0f);
            alt.haveGpsAlt = true;
        }
        else {
            alt.haveGpsAlt = false;
        }
    }
    else {
        // Source doesn't use GPS (e.g. BARO_ONLY / LIDAR_ONLY): don't carry a stale reading
        alt.haveGpsAlt = false;
    }
#endif

    if (!ARMING_FLAG(ARMED)) {
        if (alt.haveBaroAlt) {
            alt.baroAltOffset = filterApply(&alt.baroOffsetFilter, alt.baroAlt);
        }
        if (alt.haveGpsAlt) {
            alt.gpsAltOffset = filterApply(&alt.gpsOffsetFilter, alt.gpsAlt);
        }
    }
    else {
        if (alt.haveBaroAlt && alt.haveGpsAlt && alt.gpsAltOffset) {
            alt.baroAltOffset = filterApply(&alt.baroOffsetFilter,
                alt.baroAlt - (alt.gpsAlt - alt.gpsAltOffset));
        }
    }

    if (alt.haveBaroAlt && alt.baroAltOffset) {
        alt.altitude = alt.baroAlt - alt.baroAltOffset;
        alt.variometer = calculateVario(alt.baroAlt);
    }
    else if (alt.haveGpsAlt && alt.gpsAltOffset) {
        alt.altitude = alt.gpsAlt - alt.gpsAltOffset;
        alt.variometer = calculateVario(alt.gpsAlt);
    }
    else {
        alt.altitude = 0;
        alt.variometer = 0;
    }

    DEBUG(ALTITUDE, 0, alt.altitude * 100);
    DEBUG(ALTITUDE, 1, alt.variometer * 100);
    DEBUG(ALTITUDE, 2, alt.baroAlt * 100);
    DEBUG(ALTITUDE, 3, alt.baroAltOffset * 100);
    DEBUG(ALTITUDE, 4, alt.gpsAlt * 100);
    DEBUG(ALTITUDE, 5, alt.gpsAltOffset * 100);
    DEBUG(ALTITUDE, 6, gpsSol.llh.altCm);
    DEBUG(ALTITUDE, 7, gpsSol.numSat);

#ifdef USE_RANGEFINDER
    // --- AGL altitude estimation from rangefinder ---
    if (sensors(SENSOR_RANGEFINDER)) {
        const int32_t rawAlt = rangefinderGetLatestAltitude(); // tilt-compensated cm

        if (rawAlt > 0) {
            // Valid reading: update AGL estimate and increase reliability
            const float newAgl = rawAlt / 100.0f; // convert cm to m
            agl.aglVario = difFilterApply(&agl.varioFilter, newAgl);
            agl.aglAlt = newAgl;
            agl.reliability = MIN(1.0f, agl.reliability + AGL_RELIABILITY_INCREMENT);
        } else {
            // No valid reading: decay reliability
            agl.reliability = MAX(0.0f, agl.reliability - AGL_RELIABILITY_DECREMENT);
        }
    } else {
        agl.reliability = 0.0f;
    }

    DEBUG(ALTHOLD, 0, agl.aglAlt * 100);
    DEBUG(ALTHOLD, 1, agl.aglVario * 100);
    DEBUG(ALTHOLD, 2, (int32_t)(agl.reliability * 1000));
    DEBUG(ALTHOLD, 3, rangefinderGetLatestAltitude());

    // When configured for LIDAR_ONLY, the general altitude/vario estimate
    // (used by OSD, blackbox, telemetry, etc.) is sourced from the rangefinder
    // AGL estimate instead of the baro/GPS blend above.
    if (alt.source == ALT_SOURCE_LIDAR_ONLY) {
        if (isAGLAltitudeValid()) {
            alt.altitude = agl.aglAlt;
            alt.variometer = agl.aglVario;
        }
        else {
            alt.altitude = 0;
            alt.variometer = 0;
        }
    }
#endif

#ifdef USE_OPTICAL_FLOW
    // --- Optical flow XY position estimation ---
    if (sensors(SENSOR_OPTICAL_FLOW) && opticalFlowIsHealthy()) {
        const uint8_t quality = opticalFlowGetLatestQuality();
        const timeMs_t now = millis();

        if (quality >= FLOW_QUALITY_THRESHOLD && isAGLAltitudeValid()) {
            const float dt = (posxy.lastUpdateMs > 0) ?
                             (now - posxy.lastUpdateMs) / 1000.0f : 0.0f;

            if (dt > 0.0f && dt < 0.1f) {
                // Optical flow gives velocity in cm/s already scaled by distance
                // (driver already applied: flowX = flow_vel_x * distance_mm / 1000)
                const float flowBodyX = opticalFlowGetLatestX();
                const float flowBodyY = opticalFlowGetLatestY();

                // Transform body-frame velocity to earth frame using rotation matrix
                // rMat[0][0..1] = north components of body X/Y
                // rMat[1][0..1] = east  components of body X/Y
                const float velEast  =  rMat[1][0] * flowBodyX + rMat[1][1] * flowBodyY;
                const float velNorth =  rMat[0][0] * flowBodyX + rMat[0][1] * flowBodyY;

                posxy.velX = velEast;
                posxy.velY = velNorth;

                // Integrate to dead-reckoning position
                posxy.posX += velEast  * dt;
                posxy.posY += velNorth * dt;

                // Clamp to prevent unbounded drift
                const float dist = sqrtf(posxy.posX * posxy.posX + posxy.posY * posxy.posY);
                if (dist > POSXY_MAX_DEADRECKONING_CM) {
                    const float scale = POSXY_MAX_DEADRECKONING_CM / dist;
                    posxy.posX *= scale;
                    posxy.posY *= scale;
                }

                posxy.valid = true;
            }

            posxy.lastUpdateMs = now;
        } else {
            posxy.velX = 0;
            posxy.velY = 0;
            if ((now - posxy.lastUpdateMs) > 500) {
                posxy.valid = false;
            }
        }
    } else {
        posxy.velX = 0;
        posxy.velY = 0;
        posxy.valid = false;
    }

    DEBUG(POSHOLD, 0, (int32_t)posxy.posX);
    DEBUG(POSHOLD, 1, (int32_t)posxy.posY);
    DEBUG(POSHOLD, 2, (int32_t)posxy.velX);
    DEBUG(POSHOLD, 3, (int32_t)posxy.velY);
    DEBUG(POSHOLD, 4, posxy.valid ? 1 : 0);
    DEBUG(POSHOLD, 5, opticalFlowGetLatestQuality());
#endif
}

void INIT_CODE positionInit(void)
{
    alt.source = positionConfig()->alt_source;

    difFilterInit(&alt.varioFilter, positionConfig()->vario_lpf / 100.0f, pidGetPidFrequency());

    lowpassFilterInit(&alt.gpsFilter, LPF_PT2, positionConfig()->gps_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroFilter, LPF_PT2, positionConfig()->baro_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);

    lowpassFilterInit(&alt.gpsOffsetFilter, LPF_PT2, positionConfig()->gps_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroOffsetFilter, LPF_PT2, positionConfig()->baro_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);

#ifdef USE_RANGEFINDER
    difFilterInit(&agl.varioFilter, 1.0f, pidGetPidFrequency());
    agl.reliability = 0.0f;
    agl.aglAlt = 0.0f;
    agl.aglVario = 0.0f;
#endif

#ifdef USE_OPTICAL_FLOW
    posxy.posX = 0;
    posxy.posY = 0;
    posxy.velX = 0;
    posxy.velY = 0;
    posxy.valid = false;
    posxy.lastUpdateMs = 0;
#endif
}
