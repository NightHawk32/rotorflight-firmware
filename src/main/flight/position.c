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
 * Altitude (Z) is estimated by a 4-state [altitude, velocity, baro bias,
 * terrain offset] Kalman filter, horizontal position (XY) by a 2-state
 * [position, velocity] filter per axis (kalman.c, the 2-state filter ported
 * from Betaflight's position_filter.c).  IMU acceleration drives the
 * prediction step; sensors correct it with measurement noise (R) scaled by
 * their quality metrics:
 *
 *   Z  <- baro (offset vs arm point, plus an estimated downwash bias),
 *         GPS altitude and GPS Doppler vertical velocity (R from the
 *         receiver's accuracy estimate, or DOP^2 with NMEA),
 *         rangefinder AGL (gated by the reliability ramp, measures
 *         altitude minus the terrain offset)
 *   XY <- GPS position/velocity vs the arm-point origin (R as above),
 *         optical flow velocity (R scaled by flow quality)
 *
 * Every sensor is fused exactly once per sample it produces: baro, GPS,
 * rangefinder and flow all run slower than the 100Hz estimator step, and
 * re-fusing a held sample would make the filter far more confident than
 * the sensor justifies.
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
 *   carries a bias state, which is observable whenever a non-drifting anchor
 *   (GPS altitude/velocity or rangefinder) is fused.  On top of that:
 *     - baro measurement noise is inflated during rotor transients
 *       (collective steps, high roll/pitch rates) so the IMU and GPS carry
 *       the estimate through them;
 *     - a separate bias is remembered for positive and negative collective,
 *       and swapped in when the thrust direction reverses, so a learned
 *       inverted-flight offset is not re-learned after every flip;
 *     - baro innovations beyond a 5-sigma gate are rejected (spikes);
 *     - when the anchor is lost the bias is decoupled from the other
 *       states, so the baro drives the altitude again.
 *
 * Terrain (Z axis):
 *
 *   The rangefinder measures height above ground, not altitude.  After a
 *   one-time alignment per flight the terrain state absorbs changes in the
 *   ground height under the model.  Its random walk is driven by the
 *   distance flown, so hovering over one spot keeps the rangefinder a firm
 *   altitude anchor while a hill or a table edge moves the terrain state.
 *
 * Optical flow (XY):
 *
 *   The module reports raw angular flow.  A rotating camera sees the ground
 *   move too, so the vehicle's own pitch/roll rate (averaged over the flow
 *   interval) is subtracted before the flow is scaled by the height and
 *   rotated into the earth frame.
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

// Rangefinder reliability ramp, applied once per rangefinder sample (not per
// PID loop): at 50Hz valid needs ~7 good samples (~140ms) from zero, and a
// fully trusted reading is dropped after ~7 bad ones.
#define AGL_RELIABILITY_INCREMENT   0.05f
#define AGL_RELIABILITY_DECREMENT   0.10f
#define AGL_RELIABILITY_THRESHOLD   0.33f
// AGL vario differentiator cutoff, and the longest gap between two valid
// samples that is still differentiated (longer gaps restart the vario)
#define AGL_VARIO_CUTOFF_HZ         1.0f
#define AGL_MAX_SAMPLE_GAP_US       500000
// Optical flow quality hard floor (0-255): below this a sample is never fused
#define FLOW_QUALITY_MIN            50
// Beyond ~45 deg of tilt the flat-ground flow geometry (and the 1/cos^2 scale
// factor) stops being trustworthy, so no flow sample is fused at all
#define FLOW_MIN_COS_TILT           0.707f

// Outcome of the latest optical-flow sample (DEBUG_OPTICAL_FLOW[7] and
// DEBUG_POS_EST_XY[7] bits 8+): why a sample was or was not fused
enum {
    FLOW_STATUS_FUSED = 0,
    FLOW_STATUS_DISABLED,       // position_xy_source = GPS_ONLY
    FLOW_STATUS_NO_SENSOR,      // no sensor, or no sample for 500ms
    FLOW_STATUS_NO_AGL,         // no valid rangefinder height to scale with
    FLOW_STATUS_LOW_QUALITY,    // quality at or below FLOW_QUALITY_MIN
    FLOW_STATUS_TILT,           // tilted beyond FLOW_MIN_COS_TILT
};

// DEBUG_POS_EST_Z[7] flag bits (x1000, disturbance x100 in the low digits)
#define POS_EST_Z_ANCHOR_FRESH      (1 << 0)
#define POS_EST_Z_INVERTED          (1 << 1)
#define POS_EST_Z_BARO_GATED        (1 << 2)
#define POS_EST_Z_HAVE_BARO         (1 << 3)

// DEBUG_POS_EST_XY[7] flag bits (flow status in bits 8+)
#define POS_EST_XY_VALID            (1 << 0)
#define POS_EST_XY_GPS_FRESH        (1 << 1)   // GPS fused within XY_GPS_TIMEOUT_MS
#define POS_EST_XY_FLOW_FRESH       (1 << 2)
#define POS_EST_XY_CLAMPED          (1 << 3)
#define POS_EST_XY_GPS_ORIGIN       (1 << 4)
// Maximum dead-reckoning radius (cm) while no absolute (GPS) anchor is active
#define POSXY_MAX_DEADRECKONING_CM  1000.0f

// ---- Estimator constants (ported from Betaflight position_estimator.c) ----
#define ESTIMATOR_PERIOD_US         10000       // 100Hz fusion rate
#define INITIAL_POS_VAR             10000.0f    // cm^2 (1m uncertainty)
#define INITIAL_VEL_VAR             10000.0f    // (cm/s)^2
#define R_GPS_ALT_BASE              60000.0f    // cm^2 at DOP 1.0, floor with vAcc
#define GRAVITY_CMSS                980.665f
#define GPS_DOP_MIN_VALID           100         // DOP is stored *100; below 1.0 is unset
#define Z_MEASUREMENT_TIMEOUT_MS    2000
#define XY_MEASUREMENT_TIMEOUT_MS   500     // flow
#define XY_GPS_TIMEOUT_MS           2000    // GPS: a CRSF/FBUS GPS may only report at 1 Hz
// Accuracy assumed for a GPS that reports neither an accuracy estimate nor
// a DOP (CRSF sensor GPS): 1 sigma, cm and cm/s
#define GPS_ASSUMED_HACC_CM         250
#define GPS_ASSUMED_VACC_CM         400
#define GPS_ASSUMED_SACC_CMS        40
// cm per 1e-7 degree of latitude (111.3195 km per degree)
#define EARTH_CM_PER_DEG7           1.113195f

// ---- Baro downwash / bias model ----
#define INITIAL_BIAS_VAR            100.0f      // cm^2: bias is zero at the arm point
#define BIAS_SWAP_VAR               2500.0f     // cm^2 added when the thrust direction flips
#define BIAS_FORCE_ACCEPT_VAR       10000.0f    // cm^2 added after a run of gated samples
#define BARO_GATE_SIGMA             5.0f
#define BARO_GATE_MAX_REJECT_MS     500         // gated continuously this long: accept unconditionally
#define BARO_COLL_SLOW_TAU          1.0f        // s, reference for collective transients
#define BARO_COLL_TRANSIENT_REF     0.25f       // collective change (of +-1) that counts as 1.0
#define BARO_RATE_REF               360.0f      // deg/s of roll/pitch rate that counts as 1.0
#define BARO_DISTURBANCE_MAX        5.0f
#define THRUST_DIR_HYSTERESIS       0.10f       // collective (of +-1) to switch thrust direction
#define Z_ANCHOR_TIMEOUT_MS         2000

// ---- Terrain model ----
#define INITIAL_TERRAIN_VAR         25.0f       // cm^2 right after the rangefinder alignment

// Z filter state indices
enum {
    ZS_ALT = 0,
    ZS_VEL,
    ZS_BIAS,
    ZS_TERRAIN,
};

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
    bool        anchorWasFresh; // previous step's anchorFresh (edge detection)

    // Baro downwash model
    float       collSlow;       // slow reference collective for transient detection
    float       disturbance;    // 0 = calm rotor, larger = baro unreliable
    float       baroBiasMem[THRUST_DIR_COUNT];
    uint8_t     thrustDir;
    timeMs_t    baroRejectSinceMs;  // start of the current run of gated samples (0 = none)
    uint32_t    lastBaroSample; // baro sample counter last fused

    // Latest measurements as fused (cm, KF frame), for DEBUG_POS_EST_Z
    float       dbgBaroCm;
    float       dbgGpsCm;
    float       dbgRfCm;
    float       dbgRfInnovCm;   // last rangefinder innovation
    float       dbgTerrainSpeedCms; // horizontal speed driving the terrain walk
    bool        baroGated;      // last baro sample rejected by the gate
    bool        anchorFresh;    // GPS/rangefinder fused recently (bias observable)

#ifdef USE_GPS
    uint32_t    lastGpsStampMs;
#endif

#ifdef USE_RANGEFINDER
    float       rfAltOffset;    // aligns rangefinder AGL to the KF frame (cm)
    bool        rfOffsetSet;
    uint32_t    lastRfSample;   // last AGL sample fused into the KF
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
    uint32_t    lastSampleCount;// rangefinder sample counter last processed
    uint32_t    validSamples;   // increments on every valid AGL sample
    timeUs_t    lastValidUs;    // time of the last valid sample
    bool        lastValid;      // previous sample was valid (vario continuity)
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
    uint8_t     flowStatus;     // FLOW_STATUS_* of the latest sample
    bool        gpsFresh;       // GPS fused within XY_GPS_TIMEOUT_MS
    bool        flowFresh;      // flow fused within XY_MEASUREMENT_TIMEOUT_MS
    bool        clamped;        // dead-reckoning clamp applied this step
    float       flowVelEast;    // last fused flow velocity (cm/s)
    float       flowVelNorth;

    // Body rate integrated since the previous flow sample (deg), and the
    // time it covers (s): its mean over the flow interval is subtracted
    // from the flow (a rotating camera sees the ground move too)
    float       gyroRollInt;
    float       gyroPitchInt;
    float       gyroIntTime;

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

float getTerrainOffset(void)
{
    return altKalmanGetTerrain(&alt.kfUp) / 100.0f;
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
/*
 * GPS measurement noise.
 *
 * With u-blox the receiver reports a 1-sigma accuracy estimate per fix
 * (hAcc/vAcc/sAcc); its square is the variance directly, and the configured
 * value only acts as a floor.  Without one but with a DOP (NMEA) the
 * configured value is scaled by DOP^2.  With neither (CRSF sensor GPS) a
 * typical accuracy is assumed: an unknown-quality GPS must not be treated
 * as excellent, otherwise it would dominate optical-flow fusion.
 */
static float gpsMeasurementR(float baseR, uint16_t dop, uint16_t accuracy, uint16_t assumedAccuracy)
{
    if (accuracy > 0) {
        return fmaxf(baseR, (float)accuracy * (float)accuracy);
    }

    if (dop < GPS_DOP_MIN_VALID) {
        return fmaxf(baseR, (float)assumedAccuracy * (float)assumedAccuracy);
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
 * Horizontal ground speed (cm/s) for the terrain random walk: the ground
 * under the model can only change while it moves.
 */
static float horizontalSpeedCms(void)
{
#ifdef USE_OPTICAL_FLOW
    if (posxy.valid) {
        return sqrtf(sq(posxy.velX) + sq(posxy.velY));
    }
#endif
#ifdef USE_GPS
    if (gpsIsUsable()) {
        return gpsSol.groundSpeed;
    }
#endif
    return 0.0f;
}

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
        alt.baroBiasMem[alt.thrustDir] = alt.kfUp.x[ZS_BIAS];
        alt.kfUp.x[ZS_BIAS] = alt.baroBiasMem[dir];
        alt.kfUp.P[ZS_BIAS][ZS_BIAS] += BIAS_SWAP_VAR;
        alt.thrustDir = dir;
    }
}

static void estimatorResetZBias(void)
{
    altKalmanResetState(&alt.kfUp, ZS_BIAS, 0.0f, INITIAL_BIAS_VAR);
    altKalmanResetState(&alt.kfUp, ZS_TERRAIN, 0.0f, INITIAL_TERRAIN_VAR);
    alt.baroBiasMem[THRUST_UPRIGHT] = 0.0f;
    alt.baroBiasMem[THRUST_INVERTED] = 0.0f;
    alt.thrustDir = THRUST_UPRIGHT;
    alt.collSlow = 0.0f;
    alt.baroRejectSinceMs = 0;
    alt.anchorWasFresh = false;
}

static void estimatorUpdateZ(timeMs_t nowMs, float dt, float accelUp, bool armed)
{
    static const float H_BARO[ALT_KF_STATES] = { 1.0f, 0.0f, 1.0f,  0.0f };
    static const float H_ALT[ALT_KF_STATES]  = { 1.0f, 0.0f, 0.0f,  0.0f };
#ifdef USE_GPS
    static const float H_VEL[ALT_KF_STATES]  = { 0.0f, 1.0f, 0.0f,  0.0f };
#endif
#ifdef USE_RANGEFINDER
    static const float H_RF[ALT_KF_STATES]   = { 1.0f, 0.0f, 0.0f, -1.0f };
#endif

    const uint8_t downwashComp = positionConfig()->baro_downwash_comp;
    const float collective = armed ? pidGetCollective() : 0.0f;

    alt.disturbance = (armed && downwashComp) ? baroDisturbanceUpdate(dt, collective) : 0.0f;

    // The bias is only observable against a non-drifting anchor.  Without
    // one (baro-only) freeze it, so the baro keeps acting as the altitude.
    const bool anchorFresh = alt.lastZAnchorMs != 0 &&
                             (nowMs - alt.lastZAnchorMs) < Z_ANCHOR_TIMEOUT_MS;
    alt.anchorFresh = anchorFresh;

    if (alt.anchorWasFresh && !anchorFresh) {
        // Anchor lost.  Zero process noise alone does not stop the bias:
        // its cross-covariances still hand it a share of every baro
        // innovation.  Decouple it so the baro moves the altitude instead.
        altKalmanDecoupleState(&alt.kfUp, ZS_BIAS, INITIAL_BIAS_VAR);
    }
    alt.anchorWasFresh = anchorFresh;

    if (armed && downwashComp && anchorFresh) {
        baroThrustDirectionUpdate(collective);
    }
    const float biasNoiseScale = (armed && anchorFresh) ? (1.0f + 10.0f * alt.disturbance) : 0.0f;

    // Terrain random walk: est_q_terrain cm^2 per metre flown
    float terrainNoise = 0.0f;
    if (armed) {
        alt.dbgTerrainSpeedCms = horizontalSpeedCms();
        terrainNoise = positionConfig()->est_q_terrain * (alt.dbgTerrainSpeedCms * 0.01f) * dt;
    }
    else {
        alt.dbgTerrainSpeedCms = 0.0f;
    }

    altKalmanPredict(&alt.kfUp, dt, armed ? accelUp : 0.0f, biasNoiseScale, terrainNoise);

#ifdef USE_BARO
    // Fused once per new baro sample with the raw reading: the baro task
    // produces a sample every few 100Hz ticks, and the pre-filtered value is
    // only used for the arm-point offset and the display.
    if (alt.haveBaroAlt && baroGetSampleCount() != alt.lastBaroSample) {
        alt.lastBaroSample = baroGetSampleCount();

        const float baroCm = baro.baroAltitude - alt.baroAltOffset * 100.0f;
        const float k = downwashComp / 10.0f;
        const float baroR = positionConfig()->est_r_baro_alt *
                            (1.0f + k * alt.disturbance * alt.disturbance);

        // Gate spikes, but never lock the baro out for long: after a run of
        // rejections accept the reading, and if an anchor is available let
        // the step go into the bias rather than the altitude.
        float gate = armed ? BARO_GATE_SIGMA : 0.0f;
        if (alt.baroRejectSinceMs != 0 && (nowMs - alt.baroRejectSinceMs) >= BARO_GATE_MAX_REJECT_MS) {
            if (anchorFresh) {
                alt.kfUp.P[ZS_BIAS][ZS_BIAS] += BIAS_FORCE_ACCEPT_VAR;
            }
            gate = 0.0f;
        }

        alt.dbgBaroCm = baroCm;
        alt.baroGated = !altKalmanUpdate(&alt.kfUp, H_BARO, baroCm, baroR, gate, NULL);

        if (!alt.baroGated) {
            alt.baroRejectSinceMs = 0;
        }
        else if (alt.baroRejectSinceMs == 0) {
            alt.baroRejectSinceMs = nowMs ? nowMs : 1;
        }
        alt.lastZMeasMs = nowMs;
    }
#endif

#ifdef USE_GPS
    // Fused once per new GPS message, using the raw (unfiltered) altitude:
    // re-fusing the same sample every tick would make the filter far more
    // confident than the GPS justifies, and the hard deck margin relies on
    // an honest altitude variance.
    if (alt.haveGpsAlt && gpsData.lastMessage != alt.lastGpsStampMs) {
        alt.lastGpsStampMs = gpsData.lastMessage;

        const float gpsAltCm = gpsSol.llh.altCm - alt.gpsAltOffset * 100.0f;
        alt.dbgGpsCm = gpsAltCm;
        const float gpsAltR = gpsMeasurementR(R_GPS_ALT_BASE, gpsSol.hdop, gpsSol.vAcc, GPS_ASSUMED_VACC_CM);
        altKalmanUpdate(&alt.kfUp, H_ALT, gpsAltCm, gpsAltR, 0.0f, NULL);

        // Doppler vertical velocity: unaffected by downwash and far less
        // noisy than differentiated GPS altitude
        if (armed && GPS_velDownValid) {
            const float velR = gpsMeasurementR(positionConfig()->est_r_gps_vvel, gpsSol.hdop, gpsSol.sAcc, GPS_ASSUMED_SACC_CMS);
            altKalmanUpdate(&alt.kfUp, H_VEL, -(float)GPS_velDownCms, velR, 0.0f, NULL);
        }

        alt.lastZMeasMs = nowMs;
        alt.lastZAnchorMs = nowMs;
    }
#endif

#ifdef USE_RANGEFINDER
    // Fused once per new rangefinder sample: the sensor runs slower than the
    // 100Hz estimator, and re-fusing a held sample would overstate confidence
    if (alt.source != ALT_SOURCE_BARO_ONLY && alt.source != ALT_SOURCE_GPS_ONLY &&
        isAGLAltitudeValid() && agl.validSamples != alt.lastRfSample)
    {
        alt.lastRfSample = agl.validSamples;

        const float rfAltCm = agl.aglAlt * 100.0f;

        // First valid sample per flight: align the terrain-relative rangefinder
        // reading with the arm-relative KF frame instead of assuming they
        // agree.  The terrain state then tracks changes from this point on.
        if (!alt.rfOffsetSet) {
            alt.rfAltOffset = rfAltCm - altKalmanGetAltitude(&alt.kfUp);
            alt.rfOffsetSet = true;
            altKalmanResetState(&alt.kfUp, ZS_TERRAIN, 0.0f, INITIAL_TERRAIN_VAR);
        }

        float rfR = positionConfig()->est_r_rangefinder_alt;
        if (alt.source == ALT_SOURCE_LIDAR_ONLY) {
            rfR *= 0.25f;   // stronger pull when the user prefers the lidar
        }

        alt.dbgRfCm = rfAltCm - alt.rfAltOffset;
        altKalmanUpdate(&alt.kfUp, H_RF, alt.dbgRfCm, rfR, 0.0f, &alt.dbgRfInnovCm);
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

    // Everything needed to tune the Z filter from one blackbox log: the
    // state, each measurement as fused, the bias, sigma and the downwash model
    uint32_t zFlags = 0;
    if (anchorFresh) {
        zFlags |= POS_EST_Z_ANCHOR_FRESH;
    }
    if (alt.thrustDir == THRUST_INVERTED) {
        zFlags |= POS_EST_Z_INVERTED;
    }
    if (alt.haveBaroAlt && alt.baroGated) {
        zFlags |= POS_EST_Z_BARO_GATED;
    }
    if (alt.haveBaroAlt) {
        zFlags |= POS_EST_Z_HAVE_BARO;
    }

    DEBUG(POS_EST_Z, 0, lrintf(altKalmanGetAltitude(&alt.kfUp)));
    DEBUG(POS_EST_Z, 1, lrintf(altKalmanGetVelocity(&alt.kfUp)));
    DEBUG(POS_EST_Z, 2, lrintf(alt.dbgBaroCm));
    DEBUG(POS_EST_Z, 3, lrintf(alt.dbgGpsCm));
    DEBUG(POS_EST_Z, 4, lrintf(alt.dbgRfCm));
    DEBUG(POS_EST_Z, 5, lrintf(altKalmanGetBias(&alt.kfUp)));
    DEBUG(POS_EST_Z, 6, lrintf(sqrtf(fmaxf(altKalmanGetAltitudeVariance(&alt.kfUp), 0.0f))));
    DEBUG(POS_EST_Z, 7, (int32_t)(zFlags * 1000 + lrintf(alt.disturbance * 100)));

#ifdef USE_RANGEFINDER
    // The rangefinder chain into the Z filter and the terrain state
    DEBUG(POS_EST_TERRAIN, 0, rangefinderGetLatestRawAltitude());
    DEBUG(POS_EST_TERRAIN, 1, lrintf(agl.aglAlt * 100));
    DEBUG(POS_EST_TERRAIN, 2, lrintf(alt.dbgRfCm));
    DEBUG(POS_EST_TERRAIN, 3, lrintf(alt.dbgRfInnovCm));
    DEBUG(POS_EST_TERRAIN, 4, lrintf(altKalmanGetTerrain(&alt.kfUp)));
    DEBUG(POS_EST_TERRAIN, 5, lrintf(sqrtf(fmaxf(altKalmanGetTerrainVariance(&alt.kfUp), 0.0f))));
    DEBUG(POS_EST_TERRAIN, 6, lrintf(alt.dbgTerrainSpeedCms));
    DEBUG(POS_EST_TERRAIN, 7, lrintf(alt.rfAltOffset));
#endif
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
    posxy.flowStatus = FLOW_STATUS_NO_SENSOR;
    posxy.flowVelEast = 0;
    posxy.flowVelNorth = 0;
    posxy.gyroRollInt = 0;
    posxy.gyroPitchInt = 0;
    posxy.gyroIntTime = 0;
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
        posxy.gpsFresh = false;
        posxy.flowFresh = false;
        posxy.clamped = false;
        DEBUG(POS_EST_XY, 7, 0);
        return;
    }

    kalmanPredict(&posxy.kfEast, dt, accelEast);
    kalmanPredict(&posxy.kfNorth, dt, accelNorth);

    // Integrate the body rate between flow samples (see the flow section)
    posxy.gyroRollInt += gyro.gyroADCf[FD_ROLL] * dt;
    posxy.gyroPitchInt += gyro.gyroADCf[FD_PITCH] * dt;
    posxy.gyroIntTime += dt;

#ifdef USE_GPS
    // --- GPS position/velocity: fused once per new GPS message ---
    if (xySource != XY_SOURCE_FLOW_ONLY && gpsIsUsable() &&
        gpsData.lastMessage != posxy.lastGpsStampMs)
    {
        posxy.lastGpsStampMs = gpsData.lastMessage;

        if (!posxy.originSet) {
            // Anchor the local frame so that this fix maps onto the current
            // filter position: the state (flow dead-reckoned since arming)
            // stays continuous whether the first fix arrives before or
            // after the model has moved.
            posxy.lonScale = cos_approx(DEGREES_TO_RADIANS(gpsSol.llh.lat * 1e-7f));
            posxy.originLat = gpsSol.llh.lat -
                lrintf(kalmanGetPosition(&posxy.kfNorth) / EARTH_CM_PER_DEG7);
            posxy.originLon = gpsSol.llh.lon -
                lrintf(kalmanGetPosition(&posxy.kfEast) / (EARTH_CM_PER_DEG7 * posxy.lonScale));
            posxy.originSet = true;
        }

        const float north = (gpsSol.llh.lat - posxy.originLat) * EARTH_CM_PER_DEG7;
        const float east  = (gpsSol.llh.lon - posxy.originLon) * EARTH_CM_PER_DEG7 * posxy.lonScale;

        const float rPos = gpsMeasurementR(positionConfig()->est_r_gps_pos, gpsSol.hdop, gpsSol.hAcc, GPS_ASSUMED_HACC_CM);
        kalmanUpdatePosition(&posxy.kfEast, east, rPos);
        kalmanUpdatePosition(&posxy.kfNorth, north, rPos);

        // Doppler North/East velocity straight from the receiver when it
        // reports it (u-blox); ground course is undefined at hover speeds,
        // so decomposing speed and course is only the NMEA fallback.
        float velEast, velNorth;
        if (gpsSol.velNEValid) {
            velEast = gpsSol.velE;
            velNorth = gpsSol.velN;
        }
        else {
            // groundSpeed is cm/s (both parsers); groundCourse is decidegrees from North
            const float speedCms = gpsSol.groundSpeed;
            const float courseRad = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
            velEast = speedCms * sin_approx(courseRad);
            velNorth = speedCms * cos_approx(courseRad);
        }
        const float rVel = gpsMeasurementR(positionConfig()->est_r_gps_vel, gpsSol.hdop, gpsSol.sAcc, GPS_ASSUMED_SACC_CMS);
        kalmanUpdateVelocity(&posxy.kfEast, velEast, rVel);
        kalmanUpdateVelocity(&posxy.kfNorth, velNorth, rVel);

        posxy.lastGpsFuseMs = nowMs;
    }
#endif

    // --- Optical flow velocity: fused once per new sample ---
    if (xySource == XY_SOURCE_GPS_ONLY) {
        posxy.flowStatus = FLOW_STATUS_DISABLED;
    }
    else if (!sensors(SENSOR_OPTICAL_FLOW) || !opticalFlowIsHealthy()) {
        posxy.flowStatus = FLOW_STATUS_NO_SENSOR;
    }
    else if (opticalFlowGetLastUpdateMs() != posxy.lastFlowSampleMs) {
        posxy.lastFlowSampleMs = opticalFlowGetLastUpdateMs();

        // Mean body rate (deg/s) over the interval this flow sample covers
        float rollRate = 0.0f;
        float pitchRate = 0.0f;
        if (posxy.gyroIntTime > 0.0f) {
            rollRate = posxy.gyroRollInt / posxy.gyroIntTime;
            pitchRate = posxy.gyroPitchInt / posxy.gyroIntTime;
        }
        posxy.gyroRollInt = 0.0f;
        posxy.gyroPitchInt = 0.0f;
        posxy.gyroIntTime = 0.0f;

        const uint8_t quality = opticalFlowGetLatestQuality();

        // Beyond ~45 deg of tilt the flat-ground geometry below stops being
        // trustworthy, so such samples are not fused at all
        const float cosTilt = getCosTiltAngle();

        if (!isAGLAltitudeValid()) {
            posxy.flowStatus = FLOW_STATUS_NO_AGL;
        }
        else if (quality <= FLOW_QUALITY_MIN) {
            posxy.flowStatus = FLOW_STATUS_LOW_QUALITY;
        }
        else if (cosTilt < FLOW_MIN_COS_TILT) {
            posxy.flowStatus = FLOW_STATUS_TILT;
        }
        else {
            // Continuous quality->noise scaling on top of the hard floor above
            const float qualityNorm = constrainf(
                (float)(quality - FLOW_QUALITY_MIN) / (255.0f - FLOW_QUALITY_MIN),
                0.01f, 1.0f);
            const float flowR = positionConfig()->est_r_flow_vel / qualityNorm;

            // Body-rate compensation.
            //
            // The module reports raw angular flow in "cm/s at 1m", i.e. the
            // angular rate of the image times 100 cm.  A camera that rotates
            // sees the ground move as well: in the FLU body frame (x forward,
            // y left, z up; the frame rMat and the gyro use) a fixed ground
            // point at (0, 0, -h) moves at -w x r = (h*wy, -h*wx, 0), and
            // the module reports the opposite (the vehicle's motion):
            //
            //     flow_x(rot) = -100 * wy      flow_y(rot) = +100 * wx
            //
            // with wx, wy the roll and pitch rate in rad/s (pitch positive
            // nose down).  Subtracting that leaves the translation.  The
            // rate is the mean over the flow interval, since the module
            // integrates over its frame time.  flow_gyro_comp is in percent
            // of the physical value; a negative value handles a module whose
            // axes are mirrored against the body frame.
            const float comp = positionConfig()->flow_gyro_comp;   // cm/s@1m per rad/s at 100 %
            const float flowFwd  = opticalFlowGetLatestX() + comp * DEGREES_TO_RADIANS(pitchRate);
            const float flowLeft = opticalFlowGetLatestY() - comp * DEGREES_TO_RADIANS(rollRate);

            // Flow -> ground velocity.
            //
            // For a sensor tilted by t from vertical over flat ground at
            // vertical height h, the boresight slant range is D = h/cos(t) and
            // the angular rate produced by a horizontal velocity v is
            //
            //     w = v * cos(t) / D    =>    v = w * D / cos(t) = w * h / cos(t)^2
            //
            // so the flow scales with the VERTICAL height and is *divided* by
            // cos(t)^2.
            //
            // h comes from getAGLAltitude() rather than the driver's raw lidar
            // reading: it is median-filtered, range-gated and tilt-compensated
            // in sensors/rangefinder.c, so one bad lidar sample cannot spike
            // the velocity estimate.
            const float heightM = getAGLAltitude();
            const float flowScale = heightM / (cosTilt * cosTilt);

            // Body frame is x-forward, y-left (FLU, matching rMat's NWU
            // convention); the sensor must be mounted/configured
            // accordingly - verify signs with the debug values before
            // first flight.
            const float velFwd   =  flowFwd * flowScale;
            const float velRight = -flowLeft * flowScale;

            // Rotate heading frame -> earth frame (yaw is compass convention)
            const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
            const float cosYaw = cos_approx(yawRad);
            const float sinYaw = sin_approx(yawRad);
            const float velEast  = velFwd * sinYaw + velRight * cosYaw;
            const float velNorth = velFwd * cosYaw - velRight * sinYaw;

            kalmanUpdateVelocity(&posxy.kfEast, velEast, flowR);
            kalmanUpdateVelocity(&posxy.kfNorth, velNorth, flowR);

            posxy.lastFlowFuseMs = nowMs;
            posxy.flowStatus = FLOW_STATUS_FUSED;
            posxy.flowVelEast = velEast;
            posxy.flowVelNorth = velNorth;

            DEBUG(OPTICAL_FLOW, 3, lrintf(heightM * 100));
            DEBUG(OPTICAL_FLOW, 4, lrintf(flowScale * 100));
            DEBUG(OPTICAL_FLOW, 5, lrintf(velFwd));
            DEBUG(OPTICAL_FLOW, 6, lrintf(velRight));
        }
    }
    DEBUG(OPTICAL_FLOW, 7, posxy.flowStatus);

    const bool gpsFresh =
#ifdef USE_GPS
        posxy.lastGpsFuseMs != 0 && (nowMs - posxy.lastGpsFuseMs) < XY_GPS_TIMEOUT_MS;
#else
        false;
#endif
    const bool flowFresh =
        posxy.lastFlowFuseMs != 0 && (nowMs - posxy.lastFlowFuseMs) < XY_MEASUREMENT_TIMEOUT_MS;

    // Hard dead-reckoning bound: without an absolute (GPS) anchor the position
    // is velocity-integrated only, so clamp its radius.  Never applied while
    // GPS is anchoring - flying further than this from the arm point is then
    // perfectly legitimate.
    bool clamped = false;
    if (!gpsFresh) {
        const float px = kalmanGetPosition(&posxy.kfEast);
        const float py = kalmanGetPosition(&posxy.kfNorth);
        const float dist = sqrtf(px * px + py * py);
        if (dist > POSXY_MAX_DEADRECKONING_CM) {
            const float scale = POSXY_MAX_DEADRECKONING_CM / dist;
            posxy.kfEast.x[0] *= scale;
            posxy.kfNorth.x[0] *= scale;
            clamped = true;
        }
    }

    posxy.valid = gpsFresh || flowFresh;
    posxy.gpsFresh = gpsFresh;
    posxy.flowFresh = flowFresh;
    posxy.clamped = clamped;

    posxy.posX = kalmanGetPosition(&posxy.kfEast);
    posxy.posY = kalmanGetPosition(&posxy.kfNorth);
    posxy.velX = kalmanGetVelocity(&posxy.kfEast);
    posxy.velY = kalmanGetVelocity(&posxy.kfNorth);

    // Everything needed to tune the XY filters from one blackbox log.  GPS
    // position/speed/course are in the blackbox GPS frames already.
    uint32_t xyFlags = (uint32_t)posxy.flowStatus << 8;
    if (posxy.valid) {
        xyFlags |= POS_EST_XY_VALID;
    }
    if (gpsFresh) {
        xyFlags |= POS_EST_XY_GPS_FRESH;
    }
    if (flowFresh) {
        xyFlags |= POS_EST_XY_FLOW_FRESH;
    }
    if (clamped) {
        xyFlags |= POS_EST_XY_CLAMPED;
    }
#ifdef USE_GPS
    if (posxy.originSet) {
        xyFlags |= POS_EST_XY_GPS_ORIGIN;
    }
#endif

    DEBUG(POS_EST_XY, 0, lrintf(posxy.posX));
    DEBUG(POS_EST_XY, 1, lrintf(posxy.posY));
    DEBUG(POS_EST_XY, 2, lrintf(posxy.velX));
    DEBUG(POS_EST_XY, 3, lrintf(posxy.velY));
    DEBUG(POS_EST_XY, 4, lrintf(posxy.flowVelEast));
    DEBUG(POS_EST_XY, 5, lrintf(posxy.flowVelNorth));
    DEBUG(POS_EST_XY, 6, lrintf(sqrtf(fmaxf(0.5f * (kalmanGetPositionVariance(&posxy.kfEast) +
                                                   kalmanGetPositionVariance(&posxy.kfNorth)), 0.0f))));
    DEBUG(POS_EST_XY, 7, (int32_t)xyFlags);
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
    // Which sensors reach the Z filter.  BARO_ONLY and GPS_ONLY exclude the
    // other one; DEFAULT and LIDAR_ONLY fuse everything (LIDAR_ONLY only
    // changes the weighting and the displayed altitude), so the fused
    // altitude and the hard deck survive the lidar losing the ground.
#ifdef USE_BARO
    if (alt.source != ALT_SOURCE_GPS_ONLY) {
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
        alt.haveBaroAlt = false;
    }
#endif

#ifdef USE_GPS
    if (alt.source != ALT_SOURCE_BARO_ONLY) {
        if (gpsIsUsable()) {
            alt.gpsAlt = filterApply(&alt.gpsFilter, gpsSol.llh.altCm / 100.0f);
            alt.haveGpsAlt = true;
        }
        else {
            alt.haveGpsAlt = false;
        }
    }
    else {
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
    //
    // positionUpdate() runs every PID loop, but the rangefinder only produces
    // a new reading at its task rate and holds the last one in between.  Only
    // act on new samples, otherwise the reliability ramp would run at loop
    // rate and the vario would differentiate a staircase.
    if (sensors(SENSOR_RANGEFINDER)) {
        const uint32_t sampleCount = rangefinderGetSampleCount();

        if (sampleCount != agl.lastSampleCount) {
            agl.lastSampleCount = sampleCount;

            const int32_t rawAlt = rangefinderGetLatestAltitude(); // tilt-compensated cm

            if (rawAlt > 0) {
                // Valid reading: update AGL estimate and increase reliability
                const timeUs_t nowUs = micros();
                const float newAgl = rawAlt / 100.0f; // convert cm to m
                const timeDelta_t gapUs = cmpTimeUs(nowUs, agl.lastValidUs);

                if (agl.lastValid && gapUs > 0 && gapUs <= AGL_MAX_SAMPLE_GAP_US) {
                    // Differentiate at the actual sample rate
                    difFilterUpdate(&agl.varioFilter, AGL_VARIO_CUTOFF_HZ, 1e6f / gapUs);
                    agl.aglVario = difFilterApply(&agl.varioFilter, newAgl);
                }
                else {
                    // First sample after a gap: restart the differentiator
                    // instead of producing a step from a stale value
                    agl.varioFilter.x1 = newAgl;
                    agl.varioFilter.y1 = 0.0f;
                    agl.aglVario = 0.0f;
                }

                agl.aglAlt = newAgl;
                agl.lastValidUs = nowUs;
                agl.lastValid = true;
                agl.validSamples++;
                agl.reliability = MIN(1.0f, agl.reliability + AGL_RELIABILITY_INCREMENT);
            } else {
                // No valid reading: decay reliability
                agl.lastValid = false;
                agl.reliability = MAX(0.0f, agl.reliability - AGL_RELIABILITY_DECREMENT);
            }
        }
    } else {
        agl.lastValid = false;
        agl.reliability = 0.0f;
    }

    // Fields 1-3 are written by sensors/rangefinder.c
    DEBUG(RANGEFINDER, 4, lrintf(agl.aglAlt * 100));
    DEBUG(RANGEFINDER, 5, lrintf(agl.aglVario * 100));
    DEBUG(RANGEFINDER, 6, lrintf(agl.reliability * 1000));
    DEBUG(RANGEFINDER, 7, isAGLAltitudeValid() ? 1 : 0);
#endif

    // --- Kalman estimator (decimated to 100Hz internally) ---
    estimatorUpdate();

#ifdef USE_RANGEFINDER
    // When configured for LIDAR_ONLY, the general altitude/vario estimate
    // (used by OSD, blackbox, telemetry, etc.) is terrain-relative: it comes
    // straight from the rangefinder AGL estimate rather than the KF output.
    // The KF itself keeps fusing everything (see positionUpdate() above).
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

}

void positionGetStatus(positionStatus_t *st)
{
    uint16_t flags = 0;

    if (alt.altValid) {
        flags |= POS_STATUS_ALT_VALID;
    }
    if (alt.kfValid) {
        flags |= POS_STATUS_KF_VALID;
    }
    if (alt.anchorFresh) {
        flags |= POS_STATUS_ANCHOR_FRESH;
    }
    if (alt.thrustDir == THRUST_INVERTED) {
        flags |= POS_STATUS_INVERTED;
    }
    if (alt.haveBaroAlt && alt.baroGated) {
        flags |= POS_STATUS_BARO_GATED;
    }
    if (alt.haveBaroAlt) {
        flags |= POS_STATUS_HAVE_BARO;
    }
    if (alt.haveGpsAlt) {
        flags |= POS_STATUS_HAVE_GPS_ALT;
    }

    st->altitudeCm = alt.altitude * 100.0f;
    st->kfAltCm = altKalmanGetAltitude(&alt.kfUp);
    st->kfVarioCms = altKalmanGetVelocity(&alt.kfUp);
    st->kfSigmaCm = sqrtf(fmaxf(altKalmanGetAltitudeVariance(&alt.kfUp), 0.0f));
    st->baroBiasCm = altKalmanGetBias(&alt.kfUp);
    st->disturbance = alt.disturbance;
    st->baroMeasCm = alt.dbgBaroCm;
    st->gpsMeasCm = alt.dbgGpsCm;
    st->rfMeasCm = alt.dbgRfCm;
    st->terrainCm = altKalmanGetTerrain(&alt.kfUp);
    st->terrainSigmaCm = sqrtf(fmaxf(altKalmanGetTerrainVariance(&alt.kfUp), 0.0f));

#ifdef USE_RANGEFINDER
    if (sensors(SENSOR_RANGEFINDER)) {
        flags |= POS_STATUS_RANGEFINDER;
    }
    if (isAGLAltitudeValid()) {
        flags |= POS_STATUS_AGL_VALID;
    }
    st->aglAltCm = agl.aglAlt * 100.0f;
    st->aglVarioCms = agl.aglVario * 100.0f;
    st->aglReliability = agl.reliability;
    st->rangefinderRawCm = rangefinderGetLatestRawAltitude();
#else
    st->aglAltCm = 0;
    st->aglVarioCms = 0;
    st->aglReliability = 0;
    st->rangefinderRawCm = 0;
#endif

#ifdef USE_OPTICAL_FLOW
    if (sensors(SENSOR_OPTICAL_FLOW)) {
        flags |= POS_STATUS_FLOW;
        if (opticalFlowIsHealthy()) {
            flags |= POS_STATUS_FLOW_HEALTHY;
        }
    }
    if (posxy.valid) {
        flags |= POS_STATUS_XY_VALID;
    }
    if (posxy.gpsFresh) {
        flags |= POS_STATUS_XY_GPS_FRESH;
    }
    if (posxy.flowFresh) {
        flags |= POS_STATUS_XY_FLOW_FRESH;
    }
    if (posxy.clamped) {
        flags |= POS_STATUS_XY_CLAMPED;
    }
#ifdef USE_GPS
    if (posxy.originSet) {
        flags |= POS_STATUS_XY_GPS_ORIGIN;
    }
#endif
    st->flowX = opticalFlowGetLatestX();
    st->flowY = opticalFlowGetLatestY();
    st->flowQuality = opticalFlowGetLatestQuality();
    st->flowStatus = posxy.flowStatus;
    st->posEastCm = posxy.posX;
    st->posNorthCm = posxy.posY;
    st->velEastCms = posxy.velX;
    st->velNorthCms = posxy.velY;
    st->posSigmaCm = sqrtf(fmaxf(0.5f * (kalmanGetPositionVariance(&posxy.kfEast) +
                                        kalmanGetPositionVariance(&posxy.kfNorth)), 0.0f));
    st->flowVelEastCms = posxy.flowVelEast;
    st->flowVelNorthCms = posxy.flowVelNorth;
#else
    st->flowX = 0;
    st->flowY = 0;
    st->flowQuality = 0;
    st->flowStatus = 0;
    st->posEastCm = 0;
    st->posNorthCm = 0;
    st->velEastCms = 0;
    st->velNorthCms = 0;
    st->posSigmaCm = 0;
    st->flowVelEastCms = 0;
    st->flowVelNorthCms = 0;
#endif

    st->flags = flags;
}

void INIT_CODE positionInit(void)
{
    alt.source = positionConfig()->alt_source;

    lowpassFilterInit(&alt.gpsFilter, LPF_PT2, positionConfig()->gps_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroFilter, LPF_PT2, positionConfig()->baro_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);

    lowpassFilterInit(&alt.gpsOffsetFilter, LPF_PT2, positionConfig()->gps_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroOffsetFilter, LPF_PT2, positionConfig()->baro_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);

    altKalmanInit(&alt.kfUp, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_BIAS_VAR, INITIAL_TERRAIN_VAR,
                  positionConfig()->est_q_accel_z, positionConfig()->est_q_baro_bias);
    estimatorResetZBias();
    alt.lastZMeasMs = 0;
    alt.lastZAnchorMs = 0;
    alt.altValid = false;
    alt.kfValid = false;
    alt.disturbance = 0.0f;
    alt.lastBaroSample = 0;
#ifdef USE_GPS
    alt.lastGpsStampMs = 0;
#endif

    estimatorLastUs = 0;
    estimatorWasArmed = false;

#ifdef USE_RANGEFINDER
    difFilterInit(&agl.varioFilter, AGL_VARIO_CUTOFF_HZ, 50.0f);
    agl.reliability = 0.0f;
    agl.aglAlt = 0.0f;
    agl.aglVario = 0.0f;
    agl.lastSampleCount = 0;
    agl.validSamples = 0;
    agl.lastValidUs = 0;
    agl.lastValid = false;
    alt.rfAltOffset = 0.0f;
    alt.rfOffsetSet = false;
    alt.lastRfSample = 0;
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
