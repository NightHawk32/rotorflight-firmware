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
 *   Z  <- baro (offset vs arm point, plus an estimated downwash bias),
 *         GPS altitude and GPS Doppler vertical velocity (R scaled by HDOP^2),
 *         rangefinder AGL (gated by the reliability ramp)
 *   XY <- GPS position/velocity vs the arm-point origin (R scaled by HDOP^2),
 *         optical flow velocity (R scaled by flow quality)
 *
 * Rotorflight's original safety layers are kept on top of the filters:
 * the rangefinder reliability ramp, the hard flow-quality floor, the
 * dead-reckoning radius clamp (applies only while GPS is not anchoring),
 * and the XY staleness timeout.
 *
 * Baro downwash handling (Z axis):
 *
 *   The baro sits inside the rotor downwash, so its reading carries a
 *   pressure error that depends on rotor thrust and changes character when
 *   the thrust direction reverses (inverted flight).  The Z filter therefore
 *   carries a third state, the baro bias, which is observable whenever a
 *   non-drifting anchor (GPS altitude/velocity or rangefinder) is fused.
 *   On top of that:
 *     - baro measurement noise is inflated during rotor transients
 *       (collective steps, high roll/pitch rates) so the IMU and GPS carry
 *       the estimate through them;
 *     - a separate bias is remembered for positive and negative collective,
 *       and swapped in when the thrust direction reverses, so a learned
 *       inverted-flight offset is not re-learned after every flip;
 *     - baro innovations beyond a 5-sigma gate are rejected (spikes).
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
#include "sensors/gyro.h"

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
// Beyond ~45 deg of tilt the flat-ground flow geometry (and the 1/cos^2 scale
// factor) stops being trustworthy, so no flow sample is fused at all
#define FLOW_MIN_COS_TILT           0.707f
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
#define Z_MEASUREMENT_TIMEOUT_MS    2000
#define XY_MEASUREMENT_TIMEOUT_MS   500
// cm per 1e-7 degree of latitude (111.3195 km per degree)
#define EARTH_CM_PER_DEG7           1.113195f

// ---- Baro downwash / bias model ----
#define INITIAL_BIAS_VAR            100.0f      // cm^2: bias is zero at the arm point
#define BIAS_SWAP_VAR               2500.0f     // cm^2 added when the thrust direction flips
#define BIAS_FORCE_ACCEPT_VAR       10000.0f    // cm^2 added after a run of gated samples
#define BARO_GATE_SIGMA             5.0f
#define BARO_GATE_MAX_REJECTS       50          // 0.5s at 100Hz, then accept unconditionally
#define BARO_COLL_SLOW_TAU          1.0f        // s, reference for collective transients
#define BARO_COLL_TRANSIENT_REF     0.25f       // collective change (of +-1) that counts as 1.0
#define BARO_RATE_REF               360.0f      // deg/s of roll/pitch rate that counts as 1.0
#define BARO_DISTURBANCE_MAX        5.0f
#define THRUST_DIR_HYSTERESIS       0.10f       // collective (of +-1) to switch thrust direction
#define Z_ANCHOR_TIMEOUT_MS         2000

enum {
    THRUST_UPRIGHT = 0,
    THRUST_INVERTED,
    THRUST_DIR_COUNT
};


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
    altitudeKalman_t kfUp;
    timeMs_t    lastZMeasMs;
    timeMs_t    lastZAnchorMs;  // last GPS/rangefinder fusion (bias observable)
    bool        altValid;
    bool        kfValid;        // KF output valid (independent of LIDAR_ONLY override)

    // Baro downwash model
    float       collSlow;       // slow reference collective for transient detection
    float       disturbance;    // 0 = calm rotor, larger = baro unreliable
    float       baroBiasMem[THRUST_DIR_COUNT];
    uint8_t     thrustDir;
    uint8_t     baroRejects;

#ifdef USE_GPS
    uint32_t    lastGpsStampMs;
#endif

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

/*
 * False when no Z measurement has reached the filter recently.  getAltitude()
 * then reads 0, which is indistinguishable from genuinely being at the arm
 * altitude - closed-loop consumers must check this before trusting it.
 */
bool isAltitudeValid(void)
{
    return alt.altValid;
}

/*
 * Raw fused (Kalman) altitude in the arm-point frame, independent of the
 * LIDAR_ONLY display override, together with its 1-sigma uncertainty.
 * Safety features (hard deck) use this so they can apply a margin that grows
 * automatically when the estimate is poor.
 */
bool getAltitudeEstimate(float *altitudeM, float *varioMs, float *stdDevM)
{
    *altitudeM = altKalmanGetAltitude(&alt.kfUp) / 100.0f;
    *varioMs = altKalmanGetVelocity(&alt.kfUp) / 100.0f;
    *stdDevM = sqrtf(fmaxf(altKalmanGetAltitudeVariance(&alt.kfUp), 0.0f)) / 100.0f;
    return alt.kfValid;
}

float getBaroBias(void)
{
    return altKalmanGetBias(&alt.kfUp) / 100.0f;
}

float getBaroDisturbance(void)
{
    return alt.disturbance;
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

/*
 * How disturbed the rotor flow around the baro currently is.
 *
 * Collective transients (compared with a slow reference) change the downwash
 * faster than the bias state can follow, and high roll/pitch rates sweep the
 * fuselage through the rotor wake.  0 means calm, 1 is a quarter-stick
 * collective step or 360 deg/s of cyclic rate.
 */
static float baroDisturbanceUpdate(float dt, float collective)
{
    alt.collSlow += (collective - alt.collSlow) * constrainf(dt / BARO_COLL_SLOW_TAU, 0.0f, 1.0f);

    const float collTransient = fabsf(collective - alt.collSlow) / BARO_COLL_TRANSIENT_REF;
    const float cyclicRate = sqrtf(sq(gyro.gyroADCf[FD_ROLL]) + sq(gyro.gyroADCf[FD_PITCH])) / BARO_RATE_REF;

    return fminf(collTransient + cyclicRate, BARO_DISTURBANCE_MAX);
}

/*
 * The downwash reverses relative to the fuselage when the collective changes
 * sign, so the baro error for upright and inverted flight differ.  Keep one
 * learned bias per thrust direction and swap it into the filter on a change.
 */
static void baroThrustDirectionUpdate(float collective)
{
    uint8_t dir = alt.thrustDir;

    if (collective > THRUST_DIR_HYSTERESIS) {
        dir = THRUST_UPRIGHT;
    }
    else if (collective < -THRUST_DIR_HYSTERESIS) {
        dir = THRUST_INVERTED;
    }

    if (dir != alt.thrustDir) {
        alt.baroBiasMem[alt.thrustDir] = alt.kfUp.x[2];
        alt.kfUp.x[2] = alt.baroBiasMem[dir];
        alt.kfUp.P[2][2] += BIAS_SWAP_VAR;
        alt.thrustDir = dir;
    }
}

static void estimatorResetZBias(void)
{
    alt.kfUp.x[2] = 0.0f;
    alt.kfUp.P[0][2] = alt.kfUp.P[2][0] = 0.0f;
    alt.kfUp.P[1][2] = alt.kfUp.P[2][1] = 0.0f;
    alt.kfUp.P[2][2] = INITIAL_BIAS_VAR;
    alt.baroBiasMem[THRUST_UPRIGHT] = 0.0f;
    alt.baroBiasMem[THRUST_INVERTED] = 0.0f;
    alt.thrustDir = THRUST_UPRIGHT;
    alt.collSlow = 0.0f;
    alt.baroRejects = 0;
}

static void estimatorUpdateZ(timeMs_t nowMs, float dt, float accelUp, bool armed)
{
    static const float H_BARO[3] = { 1.0f, 0.0f, 1.0f };
    static const float H_ALT[3]  = { 1.0f, 0.0f, 0.0f };
#ifdef USE_GPS
    static const float H_VEL[3]  = { 0.0f, 1.0f, 0.0f };
#endif

    const uint8_t downwashComp = positionConfig()->baro_downwash_comp;
    const float collective = armed ? pidGetCollective() : 0.0f;

    alt.disturbance = (armed && downwashComp) ? baroDisturbanceUpdate(dt, collective) : 0.0f;

    // The bias is only observable against a non-drifting anchor.  Without
    // one (baro-only) freeze it, so the baro keeps acting as the altitude.
    const bool anchorFresh = alt.lastZAnchorMs != 0 &&
                             (nowMs - alt.lastZAnchorMs) < Z_ANCHOR_TIMEOUT_MS;

    if (armed && downwashComp && anchorFresh) {
        baroThrustDirectionUpdate(collective);
    }
    const float biasNoiseScale = (armed && anchorFresh) ? (1.0f + 10.0f * alt.disturbance) : 0.0f;

    altKalmanPredict(&alt.kfUp, dt, armed ? accelUp : 0.0f, biasNoiseScale);

    if (alt.haveBaroAlt) {
        const float baroCm = (alt.baroAlt - alt.baroAltOffset) * 100.0f;
        const float k = downwashComp / 10.0f;
        const float baroR = positionConfig()->est_r_baro_alt *
                            (1.0f + k * alt.disturbance * alt.disturbance);

        // Gate spikes, but never lock the baro out for long: after a run of
        // rejections accept the reading, and if an anchor is available let
        // the step go into the bias rather than the altitude.
        float gate = armed ? BARO_GATE_SIGMA : 0.0f;
        if (alt.baroRejects >= BARO_GATE_MAX_REJECTS) {
            if (anchorFresh) {
                alt.kfUp.P[2][2] += BIAS_FORCE_ACCEPT_VAR;
            }
            gate = 0.0f;
        }

        if (altKalmanUpdate(&alt.kfUp, H_BARO, baroCm, baroR, gate)) {
            alt.baroRejects = 0;
        }
        else if (alt.baroRejects < 255) {
            alt.baroRejects++;
        }
        alt.lastZMeasMs = nowMs;
    }

#ifdef USE_GPS
    // Fused once per new GPS message, using the raw (unfiltered) altitude:
    // re-fusing the same sample every tick would make the filter far more
    // confident than the GPS justifies, and the hard deck margin relies on
    // an honest altitude variance.
    if (alt.haveGpsAlt && gpsData.lastMessage != alt.lastGpsStampMs) {
        alt.lastGpsStampMs = gpsData.lastMessage;

        const float gpsAltCm = gpsSol.llh.altCm - alt.gpsAltOffset * 100.0f;
        const float gpsAltR = gpsMeasurementR(R_GPS_ALT_BASE, gpsSol.hdop);
        altKalmanUpdate(&alt.kfUp, H_ALT, gpsAltCm, gpsAltR, 0.0f);

        // Doppler vertical velocity: unaffected by downwash and far less
        // noisy than differentiated GPS altitude
        if (armed && GPS_velDownValid) {
            const float velR = gpsMeasurementR(positionConfig()->est_r_gps_vvel, gpsSol.hdop);
            altKalmanUpdate(&alt.kfUp, H_VEL, -(float)GPS_velDownCms, velR, 0.0f);
        }

        alt.lastZMeasMs = nowMs;
        alt.lastZAnchorMs = nowMs;
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
            alt.rfAltOffset = rfAltCm - altKalmanGetAltitude(&alt.kfUp);
            alt.rfOffsetSet = true;
        }

        float rfR = positionConfig()->est_r_rangefinder_alt;
        if (alt.source == ALT_SOURCE_LIDAR_ONLY) {
            rfR *= 0.25f;   // stronger pull when the user prefers the lidar
        }

        altKalmanUpdate(&alt.kfUp, H_ALT, rfAltCm - alt.rfAltOffset, rfR, 0.0f);
        alt.lastZMeasMs = nowMs;
        alt.lastZAnchorMs = nowMs;
    }
#endif

    alt.altValid = (alt.lastZMeasMs != 0 && (nowMs - alt.lastZMeasMs) < Z_MEASUREMENT_TIMEOUT_MS);
    alt.kfValid = alt.altValid;

    if (alt.altValid) {
        alt.altitude = altKalmanGetAltitude(&alt.kfUp) / 100.0f;
        alt.variometer = altKalmanGetVelocity(&alt.kfUp) / 100.0f;
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
    posxy.lastFlowSampleMs = 0;
    posxy.posX = 0;
    posxy.posY = 0;
    posxy.velX = 0;
    posxy.velY = 0;
    posxy.valid = false;
#ifdef USE_GPS
    posxy.lastGpsFuseMs = 0;
    posxy.lastGpsStampMs = 0;
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

            // Flow -> ground velocity.
            //
            // The module reports raw angular flow in "cm/s at 1m": the ground
            // speed the flow would represent if the imaged ground were 1m away
            // along the sensor's optical axis (body -Z).
            //
            // For a sensor tilted by t from vertical over flat ground at
            // vertical height h, the boresight slant range is D = h/cos(t) and
            // the angular rate produced by a horizontal velocity v is
            //
            //     w = v * cos(t) / D    =>    v = w * D / cos(t) = w * h / cos(t)^2
            //
            // so the flow scales with the VERTICAL height and is *divided* by
            // cos(t)^2.  The previous code scaled by the raw slant range and
            // then multiplied by cos(t), i.e. it applied cos(t)^2 the wrong
            // way and was off by cos(t)^4 (-24% at 20 deg of tilt).
            //
            // h comes from getAGLAltitude() rather than the driver's raw lidar
            // reading: it is median-filtered, range-gated and tilt-compensated
            // in sensors/rangefinder.c, so one bad lidar sample cannot spike
            // the velocity estimate.
            const float cosTilt = getCosTiltAngle();

            if (cosTilt >= FLOW_MIN_COS_TILT) {
                const float heightM = getAGLAltitude();
                const float flowScale = heightM / (cosTilt * cosTilt);

                // Body frame is x-forward, y-left (FLU, matching rMat's NWU
                // convention); the sensor must be mounted/configured
                // accordingly - verify signs with the debug values before
                // first flight.
                const float velFwd   =  opticalFlowGetLatestX() * flowScale;
                const float velRight = -opticalFlowGetLatestY() * flowScale;

                // Rotate heading frame -> earth frame (yaw is compass convention)
                const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
                const float cosYaw = cos_approx(yawRad);
                const float sinYaw = sin_approx(yawRad);
                const float velEast  = velFwd * sinYaw + velRight * cosYaw;
                const float velNorth = velFwd * cosYaw - velRight * sinYaw;

                kalmanUpdateVelocity(&posxy.kfEast, velEast, flowR);
                kalmanUpdateVelocity(&posxy.kfNorth, velNorth, flowR);

                posxy.lastFlowFuseMs = nowMs;

                DEBUG(OPTICAL_FLOW, 3, (int32_t)velEast);
                DEBUG(OPTICAL_FLOW, 4, (int32_t)velNorth);
                DEBUG(OPTICAL_FLOW, 5, (int32_t)(heightM * 100));
                DEBUG(OPTICAL_FLOW, 6, (int32_t)(flowScale * 100));
            }
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
        estimatorResetZBias();
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
    // the arm point.  While armed the remaining baro error (downwash, drift)
    // is tracked by the bias state of the Z filter inside estimatorUpdateZ().
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
        alt.altValid = isAGLAltitudeValid();
        if (alt.altValid) {
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
    // 6 and 7 are written by posHoldUpdate() later in the same cycle.
#endif
}

void INIT_CODE positionInit(void)
{
    alt.source = positionConfig()->alt_source;

    lowpassFilterInit(&alt.gpsFilter, LPF_PT2, positionConfig()->gps_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroFilter, LPF_PT2, positionConfig()->baro_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);

    lowpassFilterInit(&alt.gpsOffsetFilter, LPF_PT2, positionConfig()->gps_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroOffsetFilter, LPF_PT2, positionConfig()->baro_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);

    altKalmanInit(&alt.kfUp, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_BIAS_VAR,
                  positionConfig()->est_q_accel_z, positionConfig()->est_q_baro_bias);
    estimatorResetZBias();
    alt.lastZMeasMs = 0;
    alt.lastZAnchorMs = 0;
    alt.altValid = false;
    alt.kfValid = false;
    alt.disturbance = 0.0f;
#ifdef USE_GPS
    alt.lastGpsStampMs = 0;
#endif

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
