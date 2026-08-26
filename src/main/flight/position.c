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

/*
 * Position/altitude state estimation.
 *
 * Altitude (Z) and horizontal position (XY) are each estimated by a 2-state
 * [position, velocity] Kalman filter per axis (kalman.c, ported from
 * Betaflight's position_filter.c).  IMU acceleration drives the prediction
 * step; sensors correct it with measurement noise (R) scaled by their
 * quality metrics:
 *
 *   Z  <- baro (offset vs arm point), GPS altitude (R scaled by HDOP^2),
 *         rangefinder AGL (gated by the reliability ramp)
 *   XY <- GPS position/velocity vs the arm-point origin (R scaled by HDOP^2),
 *         optical flow velocity (R scaled by flow quality)
 *
 * Rotorflight's original safety layers are kept on top of the filters:
 * the rangefinder reliability ramp, the hard flow-quality floor, the
 * dead-reckoning radius clamp (applies only while GPS is not anchoring),
 * and the XY staleness timeout.
 *
 * The estimator itself runs decimated at 100Hz (matching the fusion rate the
 * ported Q/R constants were tuned for), even though positionUpdate() is
 * called at PID-loop rate.
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/position.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"

#ifdef USE_RANGEFINDER
#include "sensors/rangefinder.h"
#endif

#ifdef USE_OPTICAL_FLOW
#include "sensors/optical_flow.h"
#endif

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/kalman.h"
#include "flight/position.h"

// Rangefinder reliability decay constant (time constant ~400ms at 50Hz)
#define AGL_RELIABILITY_INCREMENT   0.05f
#define AGL_RELIABILITY_DECREMENT   0.10f
#define AGL_RELIABILITY_THRESHOLD   0.33f
// Optical flow quality hard floor (0-255): below this a sample is never fused
#define FLOW_QUALITY_MIN            50
// Maximum dead-reckoning radius (cm) while no absolute (GPS) anchor is active
#define POSXY_MAX_DEADRECKONING_CM  1000.0f

// ---- Estimator constants (ported from Betaflight position_estimator.c) ----
#define ESTIMATOR_PERIOD_US         10000       // 100Hz fusion rate
#define INITIAL_POS_VAR             10000.0f    // cm^2 (1m uncertainty)
#define INITIAL_VEL_VAR             10000.0f    // (cm/s)^2
#define R_GPS_ALT_BASE              60000.0f    // cm^2 at HDOP 1.0
#define GRAVITY_CMSS                980.665f
#define GPS_DOP_MIN_VALID           100         // DOP is stored *100; below 1.0 is unset
#define GPS_DOP_UNKNOWN_R_SCALE     100.0f      // unknown DOP: 10x stddev, 100x variance
#define CROSS_CAL_ALPHA             0.0001f     // baro-offset drift correction per tick
#define Z_MEASUREMENT_TIMEOUT_MS    2000
#define XY_MEASUREMENT_TIMEOUT_MS   500
// cm per 1e-7 degree of latitude (111.3195 km per degree)
#define EARTH_CM_PER_DEG7           1.113195f


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

    filter_t    gpsFilter;
    filter_t    baroFilter;

    filter_t    gpsOffsetFilter;
    filter_t    baroOffsetFilter;

    // Z-axis Kalman filter (cm, relative to the arm-point / power-up baseline)
    positionKalman_t kfUp;
    timeMs_t    lastZMeasMs;

#ifdef USE_RANGEFINDER
    float       rfAltOffset;    // aligns rangefinder AGL to the KF frame (cm)
    bool        rfOffsetSet;
#endif

} altState_t;

static FAST_DATA altState_t alt;

static FAST_DATA timeUs_t estimatorLastUs;
static FAST_DATA bool estimatorWasArmed;

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
    // East/North Kalman filters (cm, relative to arm point)
    positionKalman_t kfEast;
    positionKalman_t kfNorth;

    float       posX;           // East position (cm, relative to arm point)
    float       posY;           // North position (cm, relative to arm point)
    float       velX;           // Earth-frame velocity East  (cm/s)
    float       velY;           // Earth-frame velocity North (cm/s)
    bool        valid;

    timeMs_t    lastFlowFuseMs;
    timeMs_t    lastFlowSampleMs;

#ifdef USE_GPS
    timeMs_t    lastGpsFuseMs;
    uint32_t    lastGpsStampMs;
    int32_t     originLat;      // 1e-7 deg
    int32_t     originLon;      // 1e-7 deg
    float       lonScale;       // cos(origin latitude)
    bool        originSet;
#endif
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


/*
 * Earth-frame linear acceleration from the IMU, gravity removed, in cm/s^2.
 *
 * rMat is the body->earth rotation in NWU (North-West-Up) convention:
 * row 0 = North, row 1 = West, row 2 = Up (verified identical to Betaflight's
 * imuComputeRotationMatrix). East is therefore the negated West row.
 */
static void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp)
{
    const float accScale = acc.dev.acc_1G_rec;
    const float bx = acc.accADC[X] * accScale;
    const float by = acc.accADC[Y] * accScale;
    const float bz = acc.accADC[Z] * accScale;

    const float aNorth = rMat[0][0] * bx + rMat[0][1] * by + rMat[0][2] * bz;
    const float aWest  = rMat[1][0] * bx + rMat[1][1] * by + rMat[1][2] * bz;
    const float aUp    = rMat[2][0] * bx + rMat[2][1] * by + rMat[2][2] * bz;

    *accelEast  = -aWest * GRAVITY_CMSS;
    *accelNorth =  aNorth * GRAVITY_CMSS;
    *accelUp    = (aUp - 1.0f) * GRAVITY_CMSS;  // remove the steady 1g
}

#ifdef USE_GPS
// GPS measurement noise scaled by DOP. Unknown/implausibly low DOP must not be
// treated as excellent GPS, otherwise GPS would dominate optical-flow fusion.
static float gpsMeasurementR(float baseR, uint16_t dop)
{
    if (dop < GPS_DOP_MIN_VALID) {
        return baseR * GPS_DOP_UNKNOWN_R_SCALE;
    }

    const float dopScale = dop * 0.01f;
    return baseR * dopScale * dopScale;
}

static bool gpsIsUsable(void)
{
    return sensors(SENSOR_GPS) && STATE(GPS_FIX) &&
           gpsSol.numSat >= positionConfig()->gps_min_sats;
}
#endif

static void estimatorUpdateZ(timeMs_t nowMs, float dt, float accelUp, bool armed)
{
    kalmanPredict(&alt.kfUp, dt, armed ? accelUp : 0.0f);

    bool baroFused = false;
    bool zAnchorActive = false;     // a non-drifting source (GPS or rangefinder) fused

    if (alt.haveBaroAlt) {
        kalmanUpdatePosition(&alt.kfUp, (alt.baroAlt - alt.baroAltOffset) * 100.0f,
                             positionConfig()->est_r_baro_alt);
        alt.lastZMeasMs = nowMs;
        baroFused = true;
    }

#ifdef USE_GPS
    if (alt.haveGpsAlt) {
        const float gpsAltR = gpsMeasurementR(R_GPS_ALT_BASE, gpsSol.hdop);
        kalmanUpdatePosition(&alt.kfUp, (alt.gpsAlt - alt.gpsAltOffset) * 100.0f, gpsAltR);
        alt.lastZMeasMs = nowMs;
        zAnchorActive = true;
    }
#endif

#ifdef USE_RANGEFINDER
    if (alt.source != ALT_SOURCE_BARO_ONLY && alt.source != ALT_SOURCE_GPS_ONLY &&
        isAGLAltitudeValid())
    {
        const float rfAltCm = agl.aglAlt * 100.0f;

        // First valid sample per flight: align the terrain-relative rangefinder
        // reading with the arm-relative KF frame instead of assuming they agree.
        if (!alt.rfOffsetSet) {
            alt.rfAltOffset = rfAltCm - kalmanGetPosition(&alt.kfUp);
            alt.rfOffsetSet = true;
        }

        float rfR = positionConfig()->est_r_rangefinder_alt;
        if (alt.source == ALT_SOURCE_LIDAR_ONLY) {
            rfR *= 0.25f;   // stronger pull when the user prefers the lidar
        }

        kalmanUpdatePosition(&alt.kfUp, rfAltCm - alt.rfAltOffset, rfR);
        alt.lastZMeasMs = nowMs;
        zAnchorActive = true;
    }
#endif

    // Cross-calibration: baro drifts; whenever a non-drifting anchor is fused,
    // slowly re-derive the baro zero-offset from the KF estimate.  Replaces the
    // old armed-time baro-vs-GPS offset blend with a generalized version that
    // also uses the rangefinder as an anchor.
    if (armed && baroFused && zAnchorActive) {
        const float idealOffset = alt.baroAlt - kalmanGetPosition(&alt.kfUp) / 100.0f;
        alt.baroAltOffset += CROSS_CAL_ALPHA * (idealOffset - alt.baroAltOffset);
    }

    if (alt.lastZMeasMs != 0 && (nowMs - alt.lastZMeasMs) < Z_MEASUREMENT_TIMEOUT_MS) {
        alt.altitude = kalmanGetPosition(&alt.kfUp) / 100.0f;
        alt.variometer = kalmanGetVelocity(&alt.kfUp) / 100.0f;
    }
    else {
        alt.altitude = 0;
        alt.variometer = 0;
    }
}

#ifdef USE_OPTICAL_FLOW

static void estimatorResetXY(void)
{
    kalmanInit(&posxy.kfEast, 0, 0, INITIAL_POS_VAR, INITIAL_VEL_VAR,
               positionConfig()->est_q_accel_xy);
    kalmanInit(&posxy.kfNorth, 0, 0, INITIAL_POS_VAR, INITIAL_VEL_VAR,
               positionConfig()->est_q_accel_xy);
    posxy.lastFlowFuseMs = 0;
    posxy.valid = false;
#ifdef USE_GPS
    posxy.lastGpsFuseMs = 0;
    posxy.originSet = false;
#endif
}

static void estimatorUpdateXY(timeMs_t nowMs, float dt, float accelEast, float accelNorth, bool armed)
{
    const uint8_t xySource = positionConfig()->xy_source;

    if (!armed) {
        posxy.valid = false;
        return;
    }

    kalmanPredict(&posxy.kfEast, dt, accelEast);
    kalmanPredict(&posxy.kfNorth, dt, accelNorth);

#ifdef USE_GPS
    // --- GPS position/velocity: fused once per new GPS message ---
    if (xySource != XY_SOURCE_FLOW_ONLY && gpsIsUsable() &&
        gpsData.lastMessage != posxy.lastGpsStampMs)
    {
        posxy.lastGpsStampMs = gpsData.lastMessage;

        if (!posxy.originSet) {
            // Anchor the local frame at the first usable fix of this flight
            posxy.originLat = gpsSol.llh.lat;
            posxy.originLon = gpsSol.llh.lon;
            posxy.lonScale = cos_approx(DEGREES_TO_RADIANS(gpsSol.llh.lat * 1e-7f));
            posxy.originSet = true;
        }

        const float north = (gpsSol.llh.lat - posxy.originLat) * EARTH_CM_PER_DEG7;
        const float east  = (gpsSol.llh.lon - posxy.originLon) * EARTH_CM_PER_DEG7 * posxy.lonScale;

        const float rPos = gpsMeasurementR(positionConfig()->est_r_gps_pos, gpsSol.hdop);
        kalmanUpdatePosition(&posxy.kfEast, east, rPos);
        kalmanUpdatePosition(&posxy.kfNorth, north, rPos);

        // groundSpeed is 0.1 m/s -> cm/s; groundCourse is decidegrees from North
        const float speedCms = gpsSol.groundSpeed * 10.0f;
        const float courseRad = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
        const float rVel = gpsMeasurementR(positionConfig()->est_r_gps_vel, gpsSol.hdop);
        kalmanUpdateVelocity(&posxy.kfEast, speedCms * sin_approx(courseRad), rVel);
        kalmanUpdateVelocity(&posxy.kfNorth, speedCms * cos_approx(courseRad), rVel);

        posxy.lastGpsFuseMs = nowMs;
    }
#endif

    // --- Optical flow velocity: fused once per new sample ---
    if (xySource != XY_SOURCE_GPS_ONLY &&
        sensors(SENSOR_OPTICAL_FLOW) && opticalFlowIsHealthy() &&
        isAGLAltitudeValid() &&
        opticalFlowGetLastUpdateMs() != posxy.lastFlowSampleMs)
    {
        posxy.lastFlowSampleMs = opticalFlowGetLastUpdateMs();

        const uint8_t quality = opticalFlowGetLatestQuality();
        if (quality > FLOW_QUALITY_MIN) {
            // Continuous quality->noise scaling on top of the hard floor above
            const float qualityNorm = constrainf(
                (float)(quality - FLOW_QUALITY_MIN) / (255.0f - FLOW_QUALITY_MIN),
                0.01f, 1.0f);
            const float flowR = positionConfig()->est_r_flow_vel / qualityNorm;

            // The MicroLink driver outputs body-frame velocity in cm/s, already
            // scaled by the lidar distance.  Body frame is x-forward, y-left
            // (FLU, matching rMat's NWU convention); the sensor must be
            // mounted/configured accordingly - verify signs with the debug
            // values before first flight.
            const float bodyFwd  = opticalFlowGetLatestX();
            const float bodyLeft = opticalFlowGetLatestY();

            // Project onto the horizontal plane (remove the tilt-induced
            // component).  The driver scales by slant range rather than
            // vertical height; the residual (second-order in tilt) is
            // absorbed by the measurement noise.
            const float velFwd   = bodyFwd * cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.pitch));
            const float velRight = -bodyLeft * cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll));

            // Rotate heading frame -> earth frame (yaw is compass convention)
            const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
            const float cosYaw = cos_approx(yawRad);
            const float sinYaw = sin_approx(yawRad);
            const float velEast  = velFwd * sinYaw + velRight * cosYaw;
            const float velNorth = velFwd * cosYaw - velRight * sinYaw;

            kalmanUpdateVelocity(&posxy.kfEast, velEast, flowR);
            kalmanUpdateVelocity(&posxy.kfNorth, velNorth, flowR);

            posxy.lastFlowFuseMs = nowMs;
        }
    }

    const bool gpsFresh =
#ifdef USE_GPS
        posxy.lastGpsFuseMs != 0 && (nowMs - posxy.lastGpsFuseMs) < XY_MEASUREMENT_TIMEOUT_MS;
#else
        false;
#endif
    const bool flowFresh =
        posxy.lastFlowFuseMs != 0 && (nowMs - posxy.lastFlowFuseMs) < XY_MEASUREMENT_TIMEOUT_MS;

    // Hard dead-reckoning bound: without an absolute (GPS) anchor the position
    // is velocity-integrated only, so clamp its radius.  Never applied while
    // GPS is anchoring - flying further than this from the arm point is then
    // perfectly legitimate.
    if (!gpsFresh) {
        const float px = kalmanGetPosition(&posxy.kfEast);
        const float py = kalmanGetPosition(&posxy.kfNorth);
        const float dist = sqrtf(px * px + py * py);
        if (dist > POSXY_MAX_DEADRECKONING_CM) {
            const float scale = POSXY_MAX_DEADRECKONING_CM / dist;
            posxy.kfEast.x[0] *= scale;
            posxy.kfNorth.x[0] *= scale;
        }
    }

    posxy.valid = gpsFresh || flowFresh;

    posxy.posX = kalmanGetPosition(&posxy.kfEast);
    posxy.posY = kalmanGetPosition(&posxy.kfNorth);
    posxy.velX = kalmanGetVelocity(&posxy.kfEast);
    posxy.velY = kalmanGetVelocity(&posxy.kfNorth);
}

#endif // USE_OPTICAL_FLOW

/*
 * 100Hz decimated estimator step: predict from IMU accel, correct from sensors.
 * positionUpdate() itself runs at PID-loop rate; the Q/R constants assume the
 * fusion cadence below, so keep them together.
 */
static void estimatorUpdate(void)
{
    const timeUs_t nowUs = micros();
    const timeMs_t nowMs = millis();

    if (estimatorLastUs != 0 && cmpTimeUs(nowUs, estimatorLastUs) < ESTIMATOR_PERIOD_US) {
        return;
    }

    float dt = (estimatorLastUs != 0) ? cmpTimeUs(nowUs, estimatorLastUs) * 1e-6f
                                      : ESTIMATOR_PERIOD_US * 1e-6f;
    dt = constrainf(dt, 0.001f, 0.05f);
    estimatorLastUs = nowUs;

    const bool armed = ARMING_FLAG(ARMED);

    if (armed && !estimatorWasArmed) {
        // Arming edge: the arm point is the origin of the local frame
#ifdef USE_OPTICAL_FLOW
        estimatorResetXY();
#endif
#ifdef USE_RANGEFINDER
        alt.rfOffsetSet = false;
#endif
    }
    estimatorWasArmed = armed;

    float accelEast, accelNorth, accelUp;
    getLinearAccelENU(&accelEast, &accelNorth, &accelUp);

    estimatorUpdateZ(nowMs, dt, accelUp, armed);

#ifdef USE_OPTICAL_FLOW
    estimatorUpdateXY(nowMs, dt, accelEast, accelNorth, armed);
#else
    UNUSED(accelEast);
    UNUSED(accelNorth);
#endif
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
    // NOTE: was `alt.source & ALT_SOURCE_DEFAULT`, a bitwise AND with 0 that
    // never matched - GPS altitude was silently unused in DEFAULT mode.
    if (alt.source == ALT_SOURCE_DEFAULT || alt.source == ALT_SOURCE_GPS_ONLY) {
        if (gpsIsUsable()) {
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

    // While disarmed, track the sensor baselines so altitude is relative to
    // the arm point.  While armed the baro offset is instead cross-calibrated
    // against the KF estimate inside estimatorUpdateZ().
    if (!ARMING_FLAG(ARMED)) {
        if (alt.haveBaroAlt) {
            alt.baroAltOffset = filterApply(&alt.baroOffsetFilter, alt.baroAlt);
        }
        if (alt.haveGpsAlt) {
            alt.gpsAltOffset = filterApply(&alt.gpsOffsetFilter, alt.gpsAlt);
        }
    }

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
#endif

    // --- Kalman estimator (decimated to 100Hz internally) ---
    estimatorUpdate();

#ifdef USE_RANGEFINDER
    // When configured for LIDAR_ONLY, the general altitude/vario estimate
    // (used by OSD, blackbox, telemetry, etc.) is terrain-relative: it comes
    // straight from the rangefinder AGL estimate rather than the KF output.
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

    DEBUG(ALTITUDE, 0, alt.altitude * 100);
    DEBUG(ALTITUDE, 1, alt.variometer * 100);
    DEBUG(ALTITUDE, 2, alt.baroAlt * 100);
    DEBUG(ALTITUDE, 3, alt.baroAltOffset * 100);
    DEBUG(ALTITUDE, 4, alt.gpsAlt * 100);
    DEBUG(ALTITUDE, 5, alt.gpsAltOffset * 100);
    DEBUG(ALTITUDE, 6, gpsSol.llh.altCm);
    DEBUG(ALTITUDE, 7, gpsSol.numSat);

#ifdef USE_OPTICAL_FLOW
    DEBUG(POSHOLD, 0, (int32_t)posxy.posX);
    DEBUG(POSHOLD, 1, (int32_t)posxy.posY);
    DEBUG(POSHOLD, 2, (int32_t)posxy.velX);
    DEBUG(POSHOLD, 3, (int32_t)posxy.velY);
    DEBUG(POSHOLD, 4, posxy.valid ? 1 : 0);
    DEBUG(POSHOLD, 5, opticalFlowGetLatestQuality());
    DEBUG(POSHOLD, 6, (int32_t)sqrtf(kalmanGetPositionVariance(&posxy.kfEast)));
    DEBUG(POSHOLD, 7, (int32_t)sqrtf(kalmanGetPositionVariance(&alt.kfUp)));
#endif
}

void INIT_CODE positionInit(void)
{
    alt.source = positionConfig()->alt_source;

    lowpassFilterInit(&alt.gpsFilter, LPF_PT2, positionConfig()->gps_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroFilter, LPF_PT2, positionConfig()->baro_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);

    lowpassFilterInit(&alt.gpsOffsetFilter, LPF_PT2, positionConfig()->gps_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroOffsetFilter, LPF_PT2, positionConfig()->baro_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);

    kalmanInit(&alt.kfUp, 0, 0, INITIAL_POS_VAR, INITIAL_VEL_VAR,
               positionConfig()->est_q_accel_z);
    alt.lastZMeasMs = 0;

    estimatorLastUs = 0;
    estimatorWasArmed = false;

#ifdef USE_RANGEFINDER
    difFilterInit(&agl.varioFilter, 1.0f, pidGetPidFrequency());
    agl.reliability = 0.0f;
    agl.aglAlt = 0.0f;
    agl.aglVario = 0.0f;
    alt.rfAltOffset = 0.0f;
    alt.rfOffsetSet = false;
#endif

#ifdef USE_OPTICAL_FLOW
    posxy.posX = 0;
    posxy.posY = 0;
    posxy.velX = 0;
    posxy.velY = 0;
    posxy.lastFlowSampleMs = 0;
#ifdef USE_GPS
    posxy.lastGpsStampMs = 0;
#endif
    estimatorResetXY();
#endif
}
