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
 * Auto land.
 *
 * One switch (box AUTOLAND).  The landing is built on altitude hold and
 * position hold: this module moves the altitude target, and takes the
 * collective over itself only where a closed altitude loop cannot work
 * (ground effect and the ground).  Both holds are engaged by the autoland,
 * whatever their own switches say.
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
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"

#include "drivers/time.h"

#include "fc/core.h"
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

    // State
    autolandState_e state;
    timeMs_t    stateEntryMs;
    bool        switchOn;
    bool        refused;            // switched on while unavailable, until switched off
    bool        resumePending;      // yielded to a rescue with the switch on
    bool        available;
    float       learnedHover;       // hover + I of altitude hold, end of SETTLE
    float       climbRate;          // m/s, for altitude hold
    float       collective;         // last collective output
    float       rampFrom;           // collective when TOUCHDOWN started
    float       geCollective;       // pushed collective in GROUND_EFFECT
    timeMs_t    tdStartMs;          // touchdown conditions true since (0 = not)
    timeMs_t    geBlockedUntilMs;
    bool        prevAglValid;
    float       prevAgl;
    uint16_t    flags;
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

static void setState(autolandState_e state)
{
    al.state = state;
    al.stateEntryMs = millis();
    al.tdStartMs = 0;
    al.rampFrom = al.collective;
    al.geCollective = al.collective;
}

static bool isFlyingState(autolandState_e state)
{
    return state >= AUTOLAND_STATE_SETTLE && state <= AUTOLAND_STATE_GROUND_EFFECT;
}

static bool isOnGroundState(autolandState_e state)
{
    return state >= AUTOLAND_STATE_TOUCHDOWN && state <= AUTOLAND_STATE_MOTOR_OFF;
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
        autolandDebug();
        return;
    }

    if (!switchOn) {
        al.refused = false;
        al.resumePending = false;
    }

    // Pilot interrupt and rescue priority, while still flying
    if (isFlyingState(al.state)) {
        if (!switchOn) {
            autolandOff();
        }
        else if (rescueInControl()) {
            autolandOff();
            al.resumePending = true;
        }
    }

    switch (al.state) {
        case AUTOLAND_STATE_OFF:
            if (switchOn && !al.refused && (switchedOn || al.resumePending) && !rescueInControl()) {
                al.resumePending = false;
                if (al.available) {
                    al.geBlockedUntilMs = 0;
                    setState(AUTOLAND_STATE_SETTLE);
                }
                else {
                    al.refused = true;
                    beeper(BEEPER_ACC_CALIBRATION_FAIL);
                }
            }
            break;

        case AUTOLAND_STATE_SETTLE: {
            al.climbRate = 0;
            const timeDelta_t t = stateTimeMs();
            const bool calm = fabsf(getVario()) < AL_SETTLE_VARIO &&
                              horizontalSpeed() < AL_SETTLE_SPEED;
            if ((t >= (timeDelta_t)al.settleTimeMs && calm) ||
                t >= (timeDelta_t)(al.settleTimeMs + AL_SETTLE_MAX_EXTRA_MS)) {
                // The hover collective of this flight, out of ground effect
                // if the landing started high enough
                al.learnedHover = altHoldGetHoverCollective();
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
            // Takeoff states: not implemented yet
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

    al.prevAglValid = valid;
    al.prevAgl = agl;

    autolandDebug();
}

float autolandApplyCollective(float collective)
{
    float output = collective;

    switch (al.state) {
        case AUTOLAND_STATE_GROUND_EFFECT: {
            // Slew below the hover collective; never push up
            const float floor = al.learnedHover * tiltFactor() - al.gePush;
            if (al.geCollective > floor) {
                al.geCollective = fmaxf(al.geCollective - al.gePushRate * pidGetDT(), floor);
            }
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

bool autolandControlsAltitude(void)
{
    return al.state >= AUTOLAND_STATE_SETTLE && al.state <= AUTOLAND_STATE_TOUCHDOWN;
}

float autolandGetClimbRate(void)
{
    return autolandControlsAltitude() ? al.climbRate : 0;
}

bool autolandOverridesCollective(void)
{
    return al.state >= AUTOLAND_STATE_GROUND_EFFECT && al.state <= AUTOLAND_STATE_MOTOR_OFF;
}

bool autolandSuppressesAltHold(void)
{
    return al.state == AUTOLAND_STATE_LANDED || al.state == AUTOLAND_STATE_MOTOR_OFF;
}

bool autolandWantsPosHold(void)
{
    return isFlyingState(al.state) || isOnGroundState(al.state);
}

autolandPosHoldMode_e autolandPosHoldMode(void)
{
    if (isOnGroundState(al.state))
        return AUTOLAND_POSHOLD_GROUND;
    if (al.state == AUTOLAND_STATE_GROUND_EFFECT)
        return AUTOLAND_POSHOLD_NEAR_GROUND;
    return AUTOLAND_POSHOLD_NORMAL;
}

bool autolandKeepsAirborne(void)
{
    return isFlyingState(al.state);
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
}
