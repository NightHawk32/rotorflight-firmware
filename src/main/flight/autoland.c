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
 * Auto land and auto takeoff.
 *
 * Both are built on altitude hold and position hold: this module moves the
 * altitude target, and takes the collective over itself only where a
 * closed altitude loop cannot work (ground effect and the ground).
 *
 * AUTO LAND, one switch (box AUTOLAND), engages both holds itself:
 *
 *   OFF ──switch on, LIDAR + GPS ready──> SETTLE        hold, >= autoland_settle_time,
 *                                           │            until vario and speed are small
 *                                           ▼
 *                                         DESCEND       autoland_descent_rate, only while
 *                                           │            the AGL is valid and above
 *                                           │            autoland_slow_height (or no AGL
 *                                           ▼            but high above the arm point)
 *                                         FINAL         autoland_final_rate
 *                                           │ AGL < autoland_ge_height, or AGL lost
 *                                           ▼ just above it
 *                                         GROUND_EFFECT collective slews down to
 *                                           │            learned hover - autoland_ge_push
 *                                           ▼ touchdown detector
 *                                         TOUCHDOWN     collective ramps to
 *                                           │            autoland_ground_collective in 1 s
 *                                           ▼
 *                                         LANDED        negative pitch, motor running,
 *                                           │            for autoland_motor_off_delay
 *                                           ▼
 *                                         MOTOR_OFF     throttle cut, negative pitch,
 *                                           │            disarm after autoland_disarm_delay
 *                                           ▼
 *                                         disarmed (arm switch must be toggled)
 *
 * The LIDAR may be invalid close to the ground (minimum range, grass), so no
 * decision on the ground needs a valid AGL.  Touchdown is "pushing down but
 * not sinking": descent commanded, |vario| small, collective well below the
 * hover collective learned in SETTLE, model level, for AL_TD_CONFIRM_MS.
 *
 * Ground effect: the baro reads up to 1-1.5 m low within ~30 cm of the
 * ground (2026-10-30 log), and the cushion lets the model float.  So inside
 * autoland_ge_height the altitude loop is opened and the collective is
 * pushed below the learned hover instead.  If the model then sinks fast
 * (the LIDAR saw an obstacle, not the ground), the push is abandoned and
 * FINAL resumes.
 *
 * Switch off while flying (SETTLE to GROUND_EFFECT): altitude hold takes
 * over at the current altitude if ALTHOLD is on, otherwise the pilot gets
 * the collective back through the althold exit fade.  Switching on again
 * starts over with SETTLE.  From TOUCHDOWN on the switch is ignored, and
 * after the motor cut nothing brings the motor back before the disarm.
 *
 * Rescue, GPS rescue and failsafe take priority in flight; the landing
 * restarts with SETTLE afterwards if the switch is still on.
 *
 * AUTO TAKEOFF, with the ALTHOLD switch.  The model counts as landed from
 * arming until it lifts off.  ALTHOLD on while landed does not hold an
 * altitude (a few cm up would lift the model or push it into the ground):
 *
 *   OFF ──ALTHOLD on, landed──> GROUND_IDLE    pilot collective, capped below half
 *                                 │             the hover collective (no hop)
 *                                 ▼ collective stick above +50 % and back below +25 %,
 *                                 │ spooled up, LIDAR alive, altitude valid
 *                               TAKEOFF_RAMP   cyclic level, collective ramps to hover - 10 %
 *                                 │             in takeoff_ramp_time, then creeps up
 *                                 ▼ liftoff (vario, accZ, AGL or altitude rise, 150 ms)
 *                               TAKEOFF_CLIMB  altitude hold seeded with the liftoff
 *                                 │             collective, climbs at takeoff_climb_rate
 *                                 │             (twice that within autoland_ge_height)
 *                                 ▼ takeoff_altitude above the takeoff point
 *                               OFF            altitude hold (and position hold) holds
 *
 * The ramp aborts (back to GROUND_IDLE, new push needed) when the collective
 * reaches hover + takeoff_max_extra without a liftoff, the model tilts more
 * than 15 degrees, the stick goes fully down, ALTHOLD goes off, the rotor
 * is no longer spooled up, or a rescue starts.  A manual liftoff (ALTHOLD
 * off) also ends "landed".  Switching AUTOLAND on during the climb lands.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"

#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/airborne.h"
#include "flight/althold.h"
#include "flight/governor.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rescue.h"
#include "flight/autoland.h"

#include "io/beeper.h"

#include "sensors/sensors.h"
#ifdef USE_RANGEFINDER
#include "sensors/rangefinder.h"
#endif

#include "pg/pid.h"


#define AL_SETTLE_MAX_EXTRA_MS      10000   // SETTLE gives up waiting for calm after this
#define AL_SETTLE_VARIO             0.20f   // m/s
#define AL_SETTLE_SPEED             30.0f   // cm/s, horizontal

#define AL_NO_AGL_FAST_MIN_ALT      8.0f    // m above the arm point: fast descent without AGL

#define AL_TD_VARIO                 0.12f   // m/s, "not sinking"
#define AL_TD_COLL_MARGIN           40.0f   // collective below the learned hover
#define AL_TD_COS_TILT              0.966f  // 15 deg
#define AL_TD_CONFIRM_MS            400
#define AL_TD_RAMP_MS               1000
#define AL_TD_FALL_VARIO            -0.30f  // sinking in TOUCHDOWN: still airborne

#define AL_GE_TIMEOUT_MS            4000    // push-down without a detected touchdown
#define AL_GE_TIMEOUT_VARIO         -0.30f
#define AL_GE_FALL_VARIO            -1.0f   // push-down sinks this fast: not near the ground
#define AL_GE_LOST_AGL_FACTOR       2.0f    // AGL lost below this x ge height = ground effect
#define AL_GE_REENTRY_MS            1000
#define AL_GE_CREEP_RATE            20.0f   // collective per s below the push target

#define TO_IDLE_CAP_FRACTION        0.5f    // GROUND_IDLE: collective below half the hover
#define TO_IDLE_SLEW                500.0f  // collective per s, downwards
#define TO_PUSH_STICK               0.50f   // collective stick deflection for the trigger
#define TO_RETURN_STICK             0.25f
#define TO_ABORT_STICK              -0.80f
#define TO_RAMP_BELOW_HOVER         100.0f  // ramp target: hover - this
#define TO_CREEP_RATE               50.0f   // collective per s after the ramp
#define TO_LIFTOFF_VARIO            0.20f   // m/s
#define TO_LIFTOFF_AGL_RISE         0.08f   // m
#define TO_LIFTOFF_ALT_RISE         0.25f   // m, fused altitude
#define TO_LIFTOFF_CONFIRM_MS       150
#define TO_ABORT_COS_TILT           0.966f  // 15 deg
#define TO_GE_CLIMB_FACTOR          2.0f    // climb faster through ground effect

#define TO_GROUND_VARIO             0.30f   // m/s: "looks on the ground"
#define TO_GROUND_AGL_MARGIN        0.20f   // m above the AGL seen on the ground
#define TO_GROUND_ALT_MARGIN        0.50f   // m above the arm altitude (no AGL)
#define TO_MANUAL_LIFTOFF_VARIO     0.30f   // m/s, while landed with ALTHOLD off
#define TO_MANUAL_LIFTOFF_MS        300
#define TO_MANUAL_LIFTOFF_AGL       0.30f   // m above the AGL seen on the ground
#define TO_ARM_MAX_AGL              0.30f   // landed at arming: AGL below this (or no AGL and
                                            // not moving), so a re-arm in the air never
                                            // caps the collective
#define AL_HOVER_LPF_TAU            1.0f    // s, learned hover filter in SETTLE


typedef struct {
    // Config
    uint32_t    settleTimeMs;
    float       descentRate;        // m/s
    float       finalRate;          // m/s
    float       slowHeight;         // m
    float       geHeight;           // m
    float       gePush;             // collective
    float       gePushRate;         // collective per s
    float       groundCollective;   // collective, -1000..1000
    uint32_t    motorOffDelayMs;
    uint32_t    disarmDelayMs;
    float       takeoffAltitude;    // m above the takeoff point
    float       takeoffClimbRate;   // m/s
    uint32_t    takeoffRampMs;
    float       takeoffMaxExtra;    // collective above hover
    float       hoverCollective;    // althold_hover_collective

    // State
    autolandState_e state;
    timeMs_t    stateEntryMs;
    bool        switchOn;
    bool        refused;            // switched on while unavailable, until switched off
    bool        resumePending;      // yielded to a rescue with the switch on
    bool        available;
    float       learnedHover;       // collective flown in SETTLE (filtered, tilt removed)
    float       climbRate;          // m/s, for altitude hold
    float       collective;         // last collective output
    float       rampFrom;           // collective when TOUCHDOWN/TAKEOFF_RAMP started
    float       geCollective;       // pushed collective in GROUND_EFFECT
    timeMs_t    tdStartMs;          // touchdown/liftoff conditions true since (0 = not)
    timeMs_t    geBlockedUntilMs;
    bool        prevAglValid;
    float       prevAgl;
    uint16_t    flags;

    // Takeoff
    bool        armCheckPending;    // decide "landed" on the first armed cycle
    bool        landed;             // on the ground since arming
    bool        armAltSet;
    float       armAlt;             // fused altitude on the ground after arming
    bool        groundAglSet;
    float       groundAgl;          // lowest AGL seen while landed
    timeMs_t    manualLiftoffMs;
    bool        stickWasLow;
    bool        stickPushed;
    bool        takeoffAborted;
    float       idleCollective;     // GROUND_IDLE output
    float       groundAlt;          // fused altitude of the takeoff point
    bool        rampAglValid;
    float       rampAgl;
    float       climbHeight;        // m above the takeoff point
} autolandData_t;

static FAST_DATA_ZERO_INIT autolandData_t al;


static inline timeDelta_t stateTimeMs(void)
{
    return cmp32(millis(), al.stateEntryMs);
}

static bool rescueInControl(void)
{
    return FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE) ||
           getRescueState() != RESCUE_STATE_OFF;
}

static bool lidarReady(void)
{
#ifdef USE_RANGEFINDER
    // Healthy includes out-of-range frames: the sensor is alive at any height
    return sensors(SENSOR_RANGEFINDER) && rangefinderIsHealthy();
#else
    return false;
#endif
}

static bool aglValid(void)
{
#ifdef USE_RANGEFINDER
    return isAGLAltitudeValid();
#else
    return false;
#endif
}

static float aglHeight(void)
{
#ifdef USE_RANGEFINDER
    return getAGLAltitude();
#else
    return 0;
#endif
}

static bool xyReady(void)
{
#ifdef USE_OPTICAL_FLOW
    return isPositionXYValid();
#else
    return false;
#endif
}

static float horizontalSpeed(void)
{
#ifdef USE_OPTICAL_FLOW
    if (isPositionXYValid()) {
        return sqrtf(sq(getVelocityXCms()) + sq(getVelocityYCms()));
    }
#endif
    return 0;
}

static bool sensorsReady(void)
{
    return lidarReady() && isPositionGpsReady() && isAltitudeValid() && xyReady();
}

static bool canEngage(void)
{
    return ARMING_FLAG(ARMED) && isSpooledUp() && isAirborne() &&
           !rescueInControl() && sensorsReady();
}

// Takeoff needs no GPS: position hold works on the flow low down
static bool takeoffSensorsReady(void)
{
    return lidarReady() && isAltitudeValid();
}

static void setState(autolandState_e state)
{
    al.state = state;
    al.stateEntryMs = millis();
    al.tdStartMs = 0;
    al.rampFrom = al.collective;
    al.geCollective = al.collective;
    al.idleCollective = al.collective;
}

static bool isFlyingState(autolandState_e state)
{
    return state >= AUTOLAND_STATE_SETTLE && state <= AUTOLAND_STATE_GROUND_EFFECT;
}

static bool isOnGroundState(autolandState_e state)
{
    return state >= AUTOLAND_STATE_TOUCHDOWN && state <= AUTOLAND_STATE_MOTOR_OFF;
}

static bool isTakeoffGroundState(autolandState_e state)
{
    return state == AUTOLAND_STATE_GROUND_IDLE || state == AUTOLAND_STATE_TAKEOFF_RAMP;
}

// Fast or slow descent.  Without a valid AGL the fast rate is only used high
// above the arm point (the LIDAR is just out of range); the fallback is the
// slow rate, never the fast one.
static autolandState_e selectDescentState(void)
{
    if (aglValid()) {
        return (aglHeight() > al.slowHeight) ? AUTOLAND_STATE_DESCEND : AUTOLAND_STATE_FINAL;
    }
    if (isAltitudeValid() && getAltitude() > AL_NO_AGL_FAST_MIN_ALT) {
        return AUTOLAND_STATE_DESCEND;
    }
    return AUTOLAND_STATE_FINAL;
}

static float tiltFactor(void)
{
    const float tilt = getCosTiltAngle();
    return tilt * tilt;
}

// Detector B: pushing down but not sinking (no AGL needed)
static bool touchdownDetected(bool descentCommanded)
{
    const bool vario = fabsf(getVario()) < AL_TD_VARIO;
    const bool coll = al.collective < al.learnedHover * tiltFactor() - AL_TD_COLL_MARGIN;
    const bool tilt = getCosTiltAngle() > AL_TD_COS_TILT;

    if (vario)
        al.flags |= AL_STATUS_TD_VARIO;
    if (coll)
        al.flags |= AL_STATUS_TD_COLL;
    if (tilt)
        al.flags |= AL_STATUS_TD_TILT;

    if (descentCommanded && vario && coll && tilt) {
        if (al.tdStartMs == 0) {
            al.tdStartMs = millis();
        }
        return cmp32(millis(), al.tdStartMs) >= AL_TD_CONFIRM_MS;
    }

    al.tdStartMs = 0;
    return false;
}

static void enterGroundEffectIfLow(void)
{
    if (cmp32(millis(), al.geBlockedUntilMs) < 0) {
        return;
    }

    const bool valid = aglValid();
    if ((valid && aglHeight() < al.geHeight) ||
        (!valid && al.prevAglValid && al.prevAgl < al.geHeight * AL_GE_LOST_AGL_FACTOR)) {
        setState(AUTOLAND_STATE_GROUND_EFFECT);
    }
}

static void revertToFinal(void)
{
    al.flags |= AL_STATUS_FALLING;
    al.geBlockedUntilMs = millis() + AL_GE_REENTRY_MS;
    setState(AUTOLAND_STATE_FINAL);
}

static void autolandOff(void)
{
    al.state = AUTOLAND_STATE_OFF;
    al.stateEntryMs = millis();
    al.climbRate = 0;
    al.tdStartMs = 0;
}

/*
 * Takeoff
 */

// Still on the ground?  Used before capping the collective, so a model that
// was lifted off by hand (and not detected) is never pushed down in the air.
static bool looksOnGround(void)
{
    if (fabsf(getVario()) > TO_GROUND_VARIO) {
        return false;
    }
    if (aglValid() && al.groundAglSet) {
        return aglHeight() < al.groundAgl + TO_GROUND_AGL_MARGIN;
    }
    return !al.armAltSet || !isAltitudeValid() ||
           getAltitude() - al.armAlt < TO_GROUND_ALT_MARGIN;
}

// Ground references, and the end of "landed" after a manual liftoff
static void updateLanded(void)
{
    if (al.armCheckPending) {
        al.armCheckPending = false;
        al.landed = aglValid() ? aglHeight() < TO_ARM_MAX_AGL : fabsf(getVario()) < TO_GROUND_VARIO;
    }

    if (!al.armAltSet && isAltitudeValid()) {
        al.armAlt = getAltitude();
        al.armAltSet = true;
    }

    if (!al.landed) {
        return;
    }

    const bool valid = aglValid();
    const float agl = aglHeight();

    if (al.state == AUTOLAND_STATE_OFF || al.state == AUTOLAND_STATE_GROUND_IDLE) {
        if (valid && (!al.groundAglSet || agl < al.groundAgl)) {
            al.groundAgl = agl;
            al.groundAglSet = true;
        }

        // Manual liftoff (ALTHOLD off, the collective passes through)
        const bool aglHigh = valid && al.groundAglSet && agl > al.groundAgl + TO_MANUAL_LIFTOFF_AGL;
        if (getVario() > TO_MANUAL_LIFTOFF_VARIO) {
            if (al.manualLiftoffMs == 0) {
                al.manualLiftoffMs = millis();
            }
        }
        else {
            al.manualLiftoffMs = 0;
        }
        if (aglHigh || (al.manualLiftoffMs != 0 &&
                        cmp32(millis(), al.manualLiftoffMs) >= TO_MANUAL_LIFTOFF_MS)) {
            al.landed = false;
        }
    }
}

static void abortTakeoff(void)
{
    al.takeoffAborted = true;
    beeper(BEEPER_ACC_CALIBRATION_FAIL);
    if (FLIGHT_MODE(ALTHOLD_MODE) && !rescueInControl()) {
        setState(AUTOLAND_STATE_GROUND_IDLE);
        al.stickWasLow = false;
        al.stickPushed = false;
    }
    else {
        autolandOff();
    }
}

static void enterGroundIdle(void)
{
    setState(AUTOLAND_STATE_GROUND_IDLE);
    al.stickWasLow = false;
    al.stickPushed = false;
}

static void groundIdleUpdate(void)
{
    al.climbRate = 0;

    if (!FLIGHT_MODE(ALTHOLD_MODE) || rescueInControl()) {
        autolandOff();
        return;
    }

    if (!al.landed) {
        // Rising although capped (lifted by hand, gust): hold from here,
        // starting from the collective actually applied
        altHoldSetEngageCollective(al.collective);
        autolandOff();
        return;
    }

    // Push the collective stick above +50 % and back: the edge counts, so a
    // stick left high at arming, or held high, never starts a takeoff
    const float stick = getRcDeflection(FD_COLL);
    if (stick < TO_RETURN_STICK) {
        if (al.stickPushed) {
            al.stickPushed = false;
            if (isSpooledUp() && takeoffSensorsReady()) {
                al.takeoffAborted = false;
                al.groundAlt = getAltitude();
                al.rampAglValid = aglValid();
                al.rampAgl = aglHeight();
                setState(AUTOLAND_STATE_TAKEOFF_RAMP);
                return;
            }
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
        }
        al.stickWasLow = true;
    }
    else if (stick > TO_PUSH_STICK && al.stickWasLow) {
        al.stickPushed = true;
    }
}

static bool liftoffDetected(void)
{
    const bool vario = getVario() > TO_LIFTOFF_VARIO;
    const bool accel = airborneAccZLiftoff();
    const bool agl = al.rampAglValid && aglValid() && aglHeight() > al.rampAgl + TO_LIFTOFF_AGL_RISE;
    const bool alt = isAltitudeValid() && getAltitude() - al.groundAlt > TO_LIFTOFF_ALT_RISE;

    if (vario || accel || agl || alt) {
        if (al.tdStartMs == 0) {
            al.tdStartMs = millis();
        }
        return cmp32(millis(), al.tdStartMs) >= TO_LIFTOFF_CONFIRM_MS;
    }

    al.tdStartMs = 0;
    return false;
}

static void takeoffRampUpdate(void)
{
    al.climbRate = 0;

    if (!FLIGHT_MODE(ALTHOLD_MODE) || rescueInControl() || !isSpooledUp() ||
        getRcDeflection(FD_COLL) < TO_ABORT_STICK ||
        getCosTiltAngle() < TO_ABORT_COS_TILT ||
        al.collective >= al.hoverCollective + al.takeoffMaxExtra) {
        abortTakeoff();
        return;
    }

    if (liftoffDetected()) {
        // Altitude hold starts from the collective that lifted the model:
        // the real hover point of this flight
        al.landed = false;
        altHoldSetEngageCollective(al.collective);
        al.climbHeight = 0;
        setState(AUTOLAND_STATE_TAKEOFF_CLIMB);
    }
}

static void takeoffClimbUpdate(void)
{
    if (!FLIGHT_MODE(ALTHOLD_MODE) || rescueInControl() || !isAltitudeValid()) {
        // Altitude hold exit fade, or the rescue takes over
        autolandOff();
        return;
    }

    al.climbHeight = getAltitude() - al.groundAlt;
    if (al.climbHeight >= al.takeoffAltitude) {
        // Done: altitude hold latches the target where the model stops
        autolandOff();
        return;
    }

    al.climbRate = (al.climbHeight < al.geHeight) ?
        al.takeoffClimbRate * TO_GE_CLIMB_FACTOR : al.takeoffClimbRate;
}

static float takeoffRampCollective(void)
{
    const float t = stateTimeMs() / 1000.0f;
    const float base = al.hoverCollective - TO_RAMP_BELOW_HOVER;
    const float rampTime = al.takeoffRampMs / 1000.0f;

    if (al.rampFrom < base && rampTime > 0 && t < rampTime) {
        return al.rampFrom + (base - al.rampFrom) * t / rampTime;
    }

    // Then creep up until the model lifts off (or the ramp aborts)
    const float creepFrom = fmaxf(al.rampFrom, base);
    const float creepTime = (al.rampFrom < base) ? t - rampTime : t;
    return creepFrom + TO_CREEP_RATE * fmaxf(creepTime, 0.0f);
}

static void autolandDebug(void)
{
    DEBUG(AUTOLAND, 0, al.state);
    DEBUG(AUTOLAND, 1, aglValid() ? lrintf(aglHeight() * 100) : -1);
    DEBUG(AUTOLAND, 2, lrintf(getVario() * 100));
    DEBUG(AUTOLAND, 3, lrintf(al.collective));
    DEBUG(AUTOLAND, 4, lrintf(al.learnedHover));
    DEBUG(AUTOLAND, 5, lrintf(al.climbRate * 100));
    DEBUG(AUTOLAND, 6, (al.state == AUTOLAND_STATE_OFF) ? 0 : MIN(stateTimeMs(), 60000));
    DEBUG(AUTOLAND, 7, al.flags);
}

void autolandUpdate(void)
{
    const bool switchOn = FLIGHT_MODE(AUTOLAND_MODE);
    const bool switchedOn = switchOn && !al.switchOn;
    al.switchOn = switchOn;

    al.flags = 0;
    if (switchOn)
        al.flags |= AL_STATUS_SWITCH;
    if (lidarReady())
        al.flags |= AL_STATUS_LIDAR;
    if (isPositionGpsReady())
        al.flags |= AL_STATUS_GPS;

    const bool valid = aglValid();
    const float agl = aglHeight();
    if (valid)
        al.flags |= AL_STATUS_AGL_VALID;

    al.available = canEngage();
    if (al.available)
        al.flags |= AL_STATUS_AVAILABLE;

    if (!ARMING_FLAG(ARMED)) {
        // Disarmed (by the autoland or otherwise): start from scratch.  An
        // AUTOLAND switch left on needs to be toggled after the next arming.
        autolandOff();
        al.refused = switchOn;
        al.resumePending = false;
        al.collective = 0;
        al.learnedHover = 0;
        al.geBlockedUntilMs = 0;
        al.prevAglValid = false;
        al.landed = false;
        al.armCheckPending = true;
        al.armAltSet = false;
        al.groundAglSet = false;
        al.manualLiftoffMs = 0;
        al.takeoffAborted = false;
        autolandDebug();
        return;
    }

    updateLanded();

    if (!switchOn) {
        al.refused = false;
        al.resumePending = false;
    }

    // Pilot interrupt and rescue priority, while landing in the air
    if (isFlyingState(al.state)) {
        if (!switchOn) {
            autolandOff();
        }
        else if (rescueInControl()) {
            autolandOff();
            al.resumePending = true;
        }
    }

    // Switching AUTOLAND on during the takeoff climb lands
    if (al.state == AUTOLAND_STATE_TAKEOFF_CLIMB && switchedOn && al.available) {
        autolandOff();
    }

    switch (al.state) {
        case AUTOLAND_STATE_OFF:
            if (switchOn && !al.refused && (switchedOn || al.resumePending) && !rescueInControl()) {
                al.resumePending = false;
                if (al.available) {
                    al.geBlockedUntilMs = 0;
                    al.learnedHover = 0;
                    setState(AUTOLAND_STATE_SETTLE);
                }
                else {
                    al.refused = true;
                    beeper(BEEPER_ACC_CALIBRATION_FAIL);
                }
            }
            else if (al.landed && FLIGHT_MODE(ALTHOLD_MODE) && !rescueInControl()) {
                if (looksOnGround()) {
                    enterGroundIdle();
                }
                else {
                    // ALTHOLD switched on in the air: a normal hold
                    al.landed = false;
                }
            }
            break;

        case AUTOLAND_STATE_GROUND_IDLE:
            groundIdleUpdate();
            break;

        case AUTOLAND_STATE_TAKEOFF_RAMP:
            takeoffRampUpdate();
            break;

        case AUTOLAND_STATE_TAKEOFF_CLIMB:
            takeoffClimbUpdate();
            break;

        case AUTOLAND_STATE_SETTLE: {
            al.climbRate = 0;
            // Learn the hover collective from what is actually flown: the
            // althold integrator may still be trimming (after an auto
            // takeoff it starts from the collective in ground effect)
            const float hover = al.collective / fmaxf(tiltFactor(), 0.5f);
            if (al.learnedHover <= 0) {
                al.learnedHover = hover;
            }
            else {
                al.learnedHover += (hover - al.learnedHover) * pidGetDT() / AL_HOVER_LPF_TAU;
            }
            const timeDelta_t t = stateTimeMs();
            const bool calm = fabsf(getVario()) < AL_SETTLE_VARIO &&
                              horizontalSpeed() < AL_SETTLE_SPEED;
            if ((t >= (timeDelta_t)al.settleTimeMs && calm) ||
                t >= (timeDelta_t)(al.settleTimeMs + AL_SETTLE_MAX_EXTRA_MS)) {
                // learnedHover: the hover collective of this flight, out of
                // ground effect if the landing started high enough
                setState(selectDescentState());
            }
            break;
        }

        case AUTOLAND_STATE_DESCEND:
            al.climbRate = -al.descentRate;
            if (selectDescentState() == AUTOLAND_STATE_FINAL) {
                setState(AUTOLAND_STATE_FINAL);
            }
            break;

        case AUTOLAND_STATE_FINAL:
            al.climbRate = -al.finalRate;
            if (touchdownDetected(altHoldGetCommandedRate() < 0)) {
                setState(AUTOLAND_STATE_TOUCHDOWN);
            }
            else {
                enterGroundEffectIfLow();
            }
            break;

        case AUTOLAND_STATE_GROUND_EFFECT:
            al.climbRate = -al.finalRate;
            if (getVario() < AL_GE_FALL_VARIO) {
                // Pushed down while still high (obstacle under the LIDAR)
                revertToFinal();
            }
            else if (touchdownDetected(true)) {
                setState(AUTOLAND_STATE_TOUCHDOWN);
            }
            else if (stateTimeMs() >= AL_GE_TIMEOUT_MS && getVario() > AL_GE_TIMEOUT_VARIO) {
                setState(AUTOLAND_STATE_TOUCHDOWN);
            }
            break;

        case AUTOLAND_STATE_TOUCHDOWN:
            al.climbRate = 0;
            if (getVario() < AL_TD_FALL_VARIO) {
                // Sinking: it was still airborne
                revertToFinal();
            }
            else if (stateTimeMs() >= AL_TD_RAMP_MS) {
                setState(AUTOLAND_STATE_LANDED);
            }
            break;

        case AUTOLAND_STATE_LANDED:
            al.climbRate = 0;
            if (stateTimeMs() >= (timeDelta_t)al.motorOffDelayMs) {
                setState(AUTOLAND_STATE_MOTOR_OFF);
            }
            break;

        case AUTOLAND_STATE_MOTOR_OFF:
            al.climbRate = 0;
            if (stateTimeMs() >= (timeDelta_t)al.disarmDelayMs) {
                // Re-arming needs the arm switch toggled, as after GPS rescue
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
                disarm(DISARM_REASON_AUTOLAND);
                autolandOff();
            }
            break;

        default:
            autolandOff();
            break;
    }

    if (al.refused)
        al.flags |= AL_STATUS_REFUSED;
    if (al.resumePending)
        al.flags |= AL_STATUS_YIELDED;
    if (al.state == AUTOLAND_STATE_MOTOR_OFF)
        al.flags |= AL_STATUS_MOTOR_CUT;
    if (autolandControlsAltitude() && altHoldGetCommandedRate() != al.climbRate)
        al.flags |= AL_STATUS_STICK;
    if (al.landed)
        al.flags |= AL_STATUS_LANDED;
    if (al.stickPushed)
        al.flags |= AL_STATUS_PUSHED;
    if (al.takeoffAborted)
        al.flags |= AL_STATUS_TO_ABORTED;

    al.prevAglValid = valid;
    al.prevAgl = agl;

    autolandDebug();
}

float autolandApplyCollective(float collective)
{
    float output = collective;

    switch (al.state) {
        case AUTOLAND_STATE_GROUND_IDLE: {
            // The pilot's collective, capped so it cannot lift the model;
            // after an aborted ramp it comes down at a limited rate
            const float target = fminf(collective, al.hoverCollective * TO_IDLE_CAP_FRACTION);
            if (target >= al.idleCollective) {
                al.idleCollective = target;
            }
            else {
                al.idleCollective = fmaxf(target, al.idleCollective - TO_IDLE_SLEW * pidGetDT());
            }
            output = al.idleCollective;
            break;
        }

        case AUTOLAND_STATE_TAKEOFF_RAMP:
            output = takeoffRampCollective();
            break;

        case AUTOLAND_STATE_GROUND_EFFECT: {
            // Slew below the hover collective, never up.  Below the push
            // target keep creeping down slowly, so a strong cushion cannot
            // hold the model up; the touchdown detector ends it.
            const float target = al.learnedHover * tiltFactor() - al.gePush;
            const float rate = (al.geCollective > target) ? al.gePushRate : AL_GE_CREEP_RATE;
            al.geCollective = fmaxf(al.geCollective - rate * pidGetDT(), al.groundCollective);
            output = al.geCollective;
            break;
        }

        case AUTOLAND_STATE_TOUCHDOWN: {
            const float f = constrainf((float)stateTimeMs() / AL_TD_RAMP_MS, 0.0f, 1.0f);
            output = al.rampFrom + (al.groundCollective - al.rampFrom) * f;
            break;
        }

        case AUTOLAND_STATE_LANDED:
        case AUTOLAND_STATE_MOTOR_OFF:
            output = al.groundCollective;
            break;

        default:
            break;
    }

    al.collective = output;
    return output;
}

autolandState_e getAutolandState(void)
{
    return al.state;
}

void autolandGetStatus(autolandStatus_t *status)
{
    status->state = al.state;
    status->flags = al.flags;
    status->stateTimeMs = (al.state == AUTOLAND_STATE_OFF) ? 0 : stateTimeMs();
    status->learnedHover = al.learnedHover;
    status->collective = al.collective;
    status->climbRate = al.climbRate;
}

bool autolandIsActive(void)
{
    return al.state != AUTOLAND_STATE_OFF;
}

bool autolandIsTakeoff(void)
{
    return al.state >= AUTOLAND_STATE_GROUND_IDLE && al.state <= AUTOLAND_STATE_TAKEOFF_CLIMB;
}

bool autolandControlsAltitude(void)
{
    return (al.state >= AUTOLAND_STATE_SETTLE && al.state <= AUTOLAND_STATE_TOUCHDOWN) ||
           al.state == AUTOLAND_STATE_TAKEOFF_CLIMB;
}

float autolandGetClimbRate(void)
{
    return autolandControlsAltitude() ? al.climbRate : 0;
}

bool autolandOverridesCollective(void)
{
    return (al.state >= AUTOLAND_STATE_GROUND_EFFECT && al.state <= AUTOLAND_STATE_MOTOR_OFF) ||
           isTakeoffGroundState(al.state);
}

bool autolandSuppressesAltHold(void)
{
    return al.state == AUTOLAND_STATE_LANDED || al.state == AUTOLAND_STATE_MOTOR_OFF ||
           isTakeoffGroundState(al.state);
}

bool autolandWantsPosHold(void)
{
    return isFlyingState(al.state) || isOnGroundState(al.state) ||
           al.state == AUTOLAND_STATE_TAKEOFF_RAMP || al.state == AUTOLAND_STATE_TAKEOFF_CLIMB;
}

autolandPosHoldMode_e autolandPosHoldMode(void)
{
    if (isOnGroundState(al.state) || isTakeoffGroundState(al.state))
        return AUTOLAND_POSHOLD_GROUND;
    if (al.state == AUTOLAND_STATE_GROUND_EFFECT)
        return AUTOLAND_POSHOLD_NEAR_GROUND;
    if (al.state == AUTOLAND_STATE_TAKEOFF_CLIMB && al.climbHeight < al.geHeight)
        return AUTOLAND_POSHOLD_NEAR_GROUND;
    return AUTOLAND_POSHOLD_NORMAL;
}

bool autolandKeepsAirborne(void)
{
    return isFlyingState(al.state) || al.state == AUTOLAND_STATE_TAKEOFF_CLIMB;
}

bool autolandIsMotorCut(void)
{
    return al.state == AUTOLAND_STATE_MOTOR_OFF && ARMING_FLAG(ARMED);
}

void INIT_CODE autolandInitProfile(const pidProfile_t *pidProfile)
{
    const pidAutolandConfig_t *cfg = &pidProfile->autoland;

    al.settleTimeMs     = cfg->settle_time * 100;       // ds
    al.descentRate      = cfg->descent_rate / 100.0f;   // cm/s
    al.finalRate        = cfg->final_rate / 100.0f;     // cm/s
    al.slowHeight       = cfg->slow_height / 100.0f;    // cm
    al.geHeight         = cfg->ge_height / 100.0f;      // cm
    al.gePush           = cfg->ge_push;
    al.gePushRate       = cfg->ge_push_rate;
    al.groundCollective = cfg->ground_collective;
    al.motorOffDelayMs  = cfg->motor_off_delay * 100;   // ds
    al.disarmDelayMs    = cfg->disarm_delay * 100;      // ds
    al.takeoffAltitude  = cfg->takeoff_altitude / 100.0f;   // cm
    al.takeoffClimbRate = cfg->takeoff_climb_rate / 100.0f; // cm/s
    al.takeoffRampMs    = cfg->takeoff_ramp_time * 100;     // ds
    al.takeoffMaxExtra  = cfg->takeoff_max_extra;
    al.hoverCollective  = pidProfile->althold.hover_collective;
}
