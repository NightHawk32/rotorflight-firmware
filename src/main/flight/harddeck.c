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
 * Hard deck - a training aid that keeps the helicopter above a set altitude.
 *
 * While HARDDECK mode is on the pilot flies normally (any mode, upright or
 * inverted).  The controller continuously predicts the lowest altitude the
 * helicopter would reach if a recovery started now, and takes over before
 * that prediction crosses the deck:
 *
 *   predicted = altitude - k*sigma - v*t_react - v^2 / (2*a_recovery)
 *
 *     sigma      1-sigma uncertainty of the fused altitude estimate, so the
 *                margin grows by itself when GPS is poor or the baro is
 *                disturbed by downwash
 *     v          descent rate (IMU/GPS-Doppler/baro fused vario)
 *     t_react    reaction time + time needed to rotate to the nearest level
 *                attitude (knife-edge is the worst case, no vertical thrust)
 *
 * Recovery sequence (rescue-style, but self-contained):
 *
 *   PULLUP  level to the nearest horizon (upright or inverted) and apply
 *           collective with the sign that pushes away from the ground
 *   FLIP    if inverted, roll/pitch over to upright
 *   CLIMB   climb back to deck + recovery margin, brake horizontal drift
 *   HOLD    hold altitude and position (GPS / optical flow) above the deck;
 *           sticks nudge the hold target, collective can never take it below
 *           the deck
 *   EXIT    blend back to the pilot when the switch is turned off, or when
 *           the pilot climbs the hold target to the release altitude
 *
 * The deck only arms once the helicopter has climbed above deck + arm margin,
 * so the mode can be switched on while still on the ground.
 *
 * Altitude comes from the fused Z Kalman filter in position.c (IMU accel +
 * baro with an estimated downwash bias + GPS altitude + GPS Doppler vertical
 * velocity + rangefinder).  When a rangefinder reading is valid the lower of
 * the fused altitude and the AGL reading is used, so rising terrain is also
 * respected.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/harddeck.h"

#include "sensors/acceleration.h"

#include "pg/pid.h"


#define HARDDECK_ARM_DELAY_MS       1000    // must stay above deck + margin this long
#define HARDDECK_LEVELED_COS        0.866f  // < 30 deg from a level attitude
#define HARDDECK_UPRIGHT_COS        0.95f   // flip complete
#define HARDDECK_STOPPED_VARIO      -0.5f   // m/s: descent considered arrested
#define HARDDECK_GENTLE_VARIO       -2.0f   // m/s: slow enough to recover without pull-up
#define HARDDECK_CLIMB_DONE_ERR     0.5f    // m: close enough to the recovery target
#define HARDDECK_MIN_FLIP_RATE      50.0f   // deg/s, lower bound for t_react
#define HARDDECK_PULLUP_TIMEOUT     2000    // ms: flip even if the descent is not yet arrested
#define HARDDECK_MIN_TILT_COMP      0.7f    // limit on 1/cos tilt compensation
#define HARDDECK_MAX_TILT_DECIDEG   450     // never command more than 45 deg of tilt

// Altitude cascade.  The velocity loop gain is normalised by the hover
// collective (collective per m/s = hover/g * bandwidth), so the loop behaves
// the same on any helicopter once rescue_hover_collective is set correctly.
#define HARDDECK_ALT_BANDWIDTH      1.0f    // 1/s: altitude error -> climb rate
#define HARDDECK_VEL_BANDWIDTH      3.0f    // 1/s: climb rate error -> vertical accel
#define HARDDECK_VEL_I_RATIO        0.5f    // 1/s: integrator relative to the P path
#define GRAVITY_MSS                 9.80665f


typedef struct {

    /* Config */

    float           deckAlt;            // m
    float           armMargin;          // m
    float           recoveryMargin;     // m
    float           releaseAlt;         // m above deck, 0 = only the switch releases
    float           recoveryAccel;      // m/s^2
    float           reactionTime;       // s
    float           sigmaFactor;
    bool            useAgl;

    // Borrowed from the rescue profile (attitude recovery)
    float           levelGain;
    float           flipGain;
    float           maxRate;
    float           maxAccel;
    float           pullUpCollective;
    float           hoverCollective;
    float           maxCollective;
    timeDelta_t     pullUpTime;
    timeDelta_t     flipTime;
    timeDelta_t     exitTime;
    float           vel_Kv;             // collective per m/s of climb rate error
    float           vel_Kiv;            // per-cycle integrator gain

    // Borrowed from the althold / poshold profiles (hold phase)
    float           collDeadband;
    float           maxClimbRate;       // m/s
    float           pos_Kp;
    float           vel_Kp;
    float           vel_Ki;
    float           maxHorizSpeed;      // cm/s
    float           maxTiltDeg;
    float           cyclicDeadband;

    /* State */

    hardDeckState_e state;
    hardDeckState_e exitTo;
    timeMs_t        stateEntryTime;
    timeMs_t        armCandidateTime;

    float           targetAlt;          // m, arm-point frame
    float           altIterm;

    float           holdX;              // cm East
    float           holdY;              // cm North
    float           iTermEast;          // deg
    float           iTermNorth;         // deg
    bool            holdXYValid;

    bool            yawPassthrough;

    float           predictedAlt;       // m (debug)

    float           setpoint[4];
    float           prevSetpoint[4];

} hardDeckState_t;

static FAST_DATA_ZERO_INIT hardDeckState_t hd;


//// Helpers

static inline void hdChangeState(hardDeckState_e newState)
{
    hd.state = newState;
    hd.stateEntryTime = millis();
}

static inline timeDelta_t hdStateTime(void)
{
    return cmp32(millis(), hd.stateEntryTime);
}

static inline bool hdModeActive(void)
{
    return ARMING_FLAG(ARMED) && FLIGHT_MODE(HARDDECK_MODE);
}

static inline int wrap180(int angle)
{
    while (angle > 1800)
        angle -= 3600;
    while (angle < -1800)
        angle += 3600;
    return angle;
}

/*
 * Altitude used for the deck decision.  Returns false if no trustworthy
 * altitude is available.
 */
static bool hdGetAltitude(float *armAlt, float *effAlt, float *vario, float *sigma)
{
    const bool valid = getAltitudeEstimate(armAlt, vario, sigma);

    *effAlt = *armAlt;

#ifdef USE_RANGEFINDER
    // Terrain-relative reading: whichever is lower wins
    if (hd.useAgl && isAGLAltitudeValid()) {
        *effAlt = fminf(*effAlt, getAGLAltitude());
    }
#endif

    return valid;
}

// Angle to the nearest level attitude (upright or inverted), degrees 0..90
static float hdAngleToNearestLevel(void)
{
    const float c = constrainf(fabsf(getCosTiltAngle()), 0.0f, 1.0f);
    return acos_approx(c) / RAD;
}

/*
 * Lowest altitude reached if a recovery started now.
 *
 * The descent is stopped in the nearest level attitude first (inverted
 * thrust works fine for that), so only the rotation to the nearest level
 * adds to the reaction time.  When inverted, the half-roll to upright that
 * follows is flown with little vertical thrust, so the sag during that flip
 * (roughly g/4 * t_flip^2) is added as well.
 */
static float hdPredictMinAltitude(float effAlt, float vario, float sigma)
{
    const float flipRate = fmaxf(hd.maxRate, HARDDECK_MIN_FLIP_RATE);
    const float descent = fmaxf(-vario, 0.0f);
    const float tReact = hd.reactionTime + hdAngleToNearestLevel() / flipRate;
    const float stopDist = descent * tReact + descent * descent / (2.0f * hd.recoveryAccel);

    float flipSag = 0.0f;
    if (getCosTiltAngle() < 0) {
        const float tFlip = 180.0f / flipRate;
        flipSag = 0.25f * GRAVITY_MSS * tFlip * tFlip;
    }

    return effAlt - hd.sigmaFactor * sigma - stopDist - flipSag;
}

static void hdApplyLimits(void)
{
    for (int i = 0; i < 3; i++) {
        hd.setpoint[i] = limitf(hd.setpoint[i], hd.maxRate);
        hd.setpoint[i] = slewLimit(hd.prevSetpoint[i], hd.setpoint[i], hd.maxAccel);
    }
    for (int i = 0; i < 4; i++) {
        hd.prevSetpoint[i] = hd.setpoint[i];
    }
}

/*
 * Rotate towards a level attitude.  With allowInverted the nearest horizon
 * (upright or inverted) is the target, otherwise always upright.
 */
static void hdApplyLeveling(bool allowInverted, float gain)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    const int rollAngle = wrap180(attitude.values.roll - trim->values.roll);
    const int pitchAngle = wrap180(attitude.values.pitch - trim->values.pitch);

    int rollError, pitchError;

    if (allowInverted && (rollAngle > 900 || rollAngle < -900)) {
        pitchError = pitchAngle;
        rollError = -rollAngle + 1800;
    }
    else {
        pitchError = -pitchAngle;
        rollError = -rollAngle;
    }

    pitchError = wrap180(pitchError);
    rollError = wrap180(rollError);

    // Avoid "gimbal lock"
    if (attitude.values.pitch > 800 || attitude.values.pitch < -800) {
        rollError = 0;
    }

    hd.setpoint[FD_ROLL] = rollError * gain;
    hd.setpoint[FD_PITCH] = pitchError * gain;
    hd.setpoint[FD_YAW] = 0;
    hd.yawPassthrough = false;
}

// Collective with the sign that produces upward thrust in the current attitude
static void hdApplyPullUpCollective(void)
{
    const float c = getCosTiltAngle();
    hd.setpoint[FD_COLL] = hd.pullUpCollective * c * fabsf(c);
}

static float hdStickRate(float deflection, float deadband, float maxRate)
{
    if (fabsf(deflection) <= deadband) {
        return 0.0f;
    }
    const float rate = (fabsf(deflection) - deadband) / (1.0f - deadband);
    return copysignf(rate * maxRate, deflection);
}

/*
 * Horizontal controller.  With a valid XY estimate this is the poshold
 * cascade (position -> velocity -> tilt, earth-frame I-term for wind); with
 * holdPosition false the velocity target is zero, i.e. it only brakes.
 * Without an XY estimate it simply levels.
 *
 * Outputs body-frame tilt targets in degrees (roll right, pitch forward).
 */
static void hdHorizontalControl(bool holdPosition, float *rollDeg, float *pitchDeg)
{
    *rollDeg = 0.0f;
    *pitchDeg = 0.0f;

#ifdef USE_OPTICAL_FLOW
    if (!isPositionXYValid()) {
        hd.holdXYValid = false;
        return;
    }

    const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    const float cosYaw = cos_approx(yawRad);
    const float sinYaw = sin_approx(yawRad);

    const float dT = pidGetDT();

    float velCmdEast = 0.0f;
    float velCmdNorth = 0.0f;

    if (holdPosition) {
        if (!hd.holdXYValid) {
            hd.holdX = getPositionXCm();
            hd.holdY = getPositionYCm();
            hd.holdXYValid = true;
        }

        // Cyclic sticks move the hold target (body frame -> earth frame)
        const float rightRate = hdStickRate(getRcDeflection(FD_ROLL), hd.cyclicDeadband, hd.maxHorizSpeed);
        const float fwdRate = hdStickRate(getRcDeflection(FD_PITCH), hd.cyclicDeadband, hd.maxHorizSpeed);

        hd.holdX += (fwdRate * sinYaw + rightRate * cosYaw) * dT;
        hd.holdY += (fwdRate * cosYaw - rightRate * sinYaw) * dT;

        velCmdEast = constrainf(hd.pos_Kp * (hd.holdX - getPositionXCm()), -hd.maxHorizSpeed, hd.maxHorizSpeed);
        velCmdNorth = constrainf(hd.pos_Kp * (hd.holdY - getPositionYCm()), -hd.maxHorizSpeed, hd.maxHorizSpeed);
    }

    const float velErrEast = velCmdEast - getVelocityXCms();
    const float velErrNorth = velCmdNorth - getVelocityYCms();

    const float tiltEast = hd.vel_Kp * velErrEast + hd.iTermEast;
    const float tiltNorth = hd.vel_Kp * velErrNorth + hd.iTermNorth;

    const float tiltFwd = tiltEast * sinYaw + tiltNorth * cosYaw;
    const float tiltRight = tiltEast * cosYaw - tiltNorth * sinYaw;

    *rollDeg = constrainf(tiltRight, -hd.maxTiltDeg, hd.maxTiltDeg);
    *pitchDeg = constrainf(tiltFwd, -hd.maxTiltDeg, hd.maxTiltDeg);

    // Wind trim only while holding, with anti-windup on clipping
    if (holdPosition && *rollDeg == tiltRight && *pitchDeg == tiltFwd) {
        hd.iTermEast += hd.vel_Ki * velErrEast;
        hd.iTermNorth += hd.vel_Ki * velErrNorth;

        const float iMag = sqrtf(sq(hd.iTermEast) + sq(hd.iTermNorth));
        if (iMag > hd.maxTiltDeg) {
            const float scale = hd.maxTiltDeg / iMag;
            hd.iTermEast *= scale;
            hd.iTermNorth *= scale;
        }
    }
#else
    UNUSED(holdPosition);
#endif
}

// Angle-mode style stabilisation to the given tilt targets; pilot keeps yaw
static void hdApplyStabilisation(float rollDeg, float pitchDeg)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    const int rollAngle = wrap180(attitude.values.roll - trim->values.roll);
    const int pitchAngle = wrap180(attitude.values.pitch - trim->values.pitch);

    const int rollTarget = constrain(lrintf(rollDeg * 10), -HARDDECK_MAX_TILT_DECIDEG, HARDDECK_MAX_TILT_DECIDEG);
    const int pitchTarget = constrain(lrintf(pitchDeg * 10), -HARDDECK_MAX_TILT_DECIDEG, HARDDECK_MAX_TILT_DECIDEG);

    int rollError = wrap180(rollTarget - rollAngle);
    int pitchError = wrap180(pitchTarget - pitchAngle);

    if (attitude.values.pitch > 800 || attitude.values.pitch < -800) {
        rollError = 0;
    }

    hd.setpoint[FD_ROLL] = rollError * hd.levelGain;
    hd.setpoint[FD_PITCH] = pitchError * hd.levelGain;
    hd.setpoint[FD_YAW] = 0;
    hd.yawPassthrough = true;
}

/*
 * Altitude cascade: altitude error -> rate-limited climb rate -> collective,
 * with the integrator (initialised to the hover collective) trimming the
 * hover point.  Upright-only collective with tilt compensation.  Falls back
 * to the trimmed hover collective when the altitude estimate is lost.
 */
static void hdApplyAltitudeControl(bool altValid, float armAlt, float vario)
{
    const float c = getCosTiltAngle();
    const float tiltComp = 1.0f / fmaxf(c, HARDDECK_MIN_TILT_COMP);

    float output;

    if (altValid) {
        const float velCmd = constrainf((hd.targetAlt - armAlt) * HARDDECK_ALT_BANDWIDTH,
                                        -hd.maxClimbRate, hd.maxClimbRate);
        const float velErr = velCmd - vario;

        hd.altIterm = constrainf(hd.altIterm + velErr * hd.vel_Kiv, 0.0f, hd.maxCollective);

        output = hd.altIterm + velErr * hd.vel_Kv;
    }
    else {
        output = hd.altIterm;
    }

    hd.setpoint[FD_COLL] = constrainf(output * tiltComp, 0.0f, hd.maxCollective);
}

static void hdStartRecovery(float armAlt, float effAlt, float vario)
{
    // Recover to deck + recovery margin.  The target is expressed in the
    // arm-point frame; if the AGL reading is the lower one, shift it by the
    // difference so the margin is kept above the terrain.
    hd.targetAlt = hd.deckAlt + hd.recoveryMargin + (armAlt - effAlt);

    hd.altIterm = hd.hoverCollective;
    hd.iTermEast = 0;
    hd.iTermNorth = 0;
    hd.holdXYValid = false;

    // Upright and only sinking slowly (e.g. hovering onto the deck): the
    // altitude controller alone is enough, skip the pull-up punch.
    if (getCosTiltAngle() > HARDDECK_LEVELED_COS && vario > HARDDECK_GENTLE_VARIO) {
        hdChangeState(HARDDECK_STATE_CLIMB);
    }
    else {
        hdChangeState(HARDDECK_STATE_PULLUP);
    }
}

static void hdStartExit(hardDeckState_e exitTo)
{
    hd.exitTo = exitTo;
    hdChangeState(HARDDECK_STATE_EXIT);
}


//// State machine

static void hdUpdateState(void)
{
    float armAlt, effAlt, vario, sigma;
    const bool altValid = hdGetAltitude(&armAlt, &effAlt, &vario, &sigma);

    const float c = getCosTiltAngle();
    const bool modeOn = hdModeActive();

    hd.predictedAlt = altValid ? hdPredictMinAltitude(effAlt, vario, sigma) : 0.0f;

    switch (hd.state)
    {
        case HARDDECK_STATE_OFF:
            if (modeOn) {
                hd.armCandidateTime = 0;
                hdChangeState(HARDDECK_STATE_WAIT);
            }
            break;

        case HARDDECK_STATE_WAIT:
            if (!modeOn) {
                hdChangeState(HARDDECK_STATE_OFF);
            }
            else if (altValid && effAlt - hd.sigmaFactor * sigma > hd.deckAlt + hd.armMargin) {
                const timeMs_t now = millis();
                if (hd.armCandidateTime == 0) {
                    hd.armCandidateTime = now ? now : 1;
                }
                else if (cmp32(now, hd.armCandidateTime) > HARDDECK_ARM_DELAY_MS) {
                    hdChangeState(HARDDECK_STATE_WATCH);
                }
            }
            else {
                hd.armCandidateTime = 0;
            }
            break;

        case HARDDECK_STATE_WATCH:
            if (!modeOn) {
                hdChangeState(HARDDECK_STATE_OFF);
            }
            else if (altValid && hd.predictedAlt < hd.deckAlt) {
                hdStartRecovery(armAlt, effAlt, vario);
            }
            break;

        case HARDDECK_STATE_PULLUP:
            hdApplyLeveling(true, hd.flipGain);
            hdApplyPullUpCollective();
            hdApplyLimits();
            if (!modeOn) {
                hdStartExit(HARDDECK_STATE_OFF);
            }
            else if (fabsf(c) > HARDDECK_LEVELED_COS && hdStateTime() > hd.pullUpTime) {
                if (c > 0) {
                    // Upright: the climb controller takes it from here
                    hdChangeState(HARDDECK_STATE_CLIMB);
                }
                else if (vario > HARDDECK_STOPPED_VARIO || hdStateTime() > HARDDECK_PULLUP_TIMEOUT) {
                    // Inverted: stop the descent on inverted thrust first,
                    // then roll upright
                    hdChangeState(HARDDECK_STATE_FLIP);
                }
            }
            break;

        case HARDDECK_STATE_FLIP:
            hdApplyLeveling(false, hd.flipGain);
            hdApplyPullUpCollective();
            hdApplyLimits();
            if (!modeOn) {
                hdStartExit(HARDDECK_STATE_OFF);
            }
            else if (c > HARDDECK_UPRIGHT_COS) {
                hdChangeState(HARDDECK_STATE_CLIMB);
            }
            else if (hdStateTime() > hd.flipTime) {
                // Flip stalled: go back to the nearest horizon and try again
                hdChangeState((c > 0.5f) ? HARDDECK_STATE_CLIMB : HARDDECK_STATE_PULLUP);
            }
            break;

        case HARDDECK_STATE_CLIMB:
        {
            float rollDeg, pitchDeg;
            hdHorizontalControl(false, &rollDeg, &pitchDeg);
            hdApplyStabilisation(rollDeg, pitchDeg);
            hdApplyAltitudeControl(altValid, armAlt, vario);
            hdApplyLimits();
            if (!modeOn) {
                hdStartExit(HARDDECK_STATE_OFF);
            }
            else if (c < 0) {
                // Upset during the climb (unlikely) - start over
                hdChangeState(HARDDECK_STATE_PULLUP);
            }
            else if (!altValid || fabsf(hd.targetAlt - armAlt) < HARDDECK_CLIMB_DONE_ERR) {
                hdChangeState(HARDDECK_STATE_HOLD);
            }
            break;
        }

        case HARDDECK_STATE_HOLD:
        {
            // Collective stick moves the hold altitude, but never below
            // deck + recovery margin (translated into the arm-point frame,
            // which also follows rising terrain when the AGL reading is lower)
            if (altValid) {
                const float floorAlt = hd.deckAlt + hd.recoveryMargin + (armAlt - effAlt);
                hd.targetAlt += hdStickRate(getRcDeflection(FD_COLL), hd.collDeadband, hd.maxClimbRate) * pidGetDT();
                hd.targetAlt = fmaxf(hd.targetAlt, floorAlt);
            }

            float rollDeg, pitchDeg;
            hdHorizontalControl(true, &rollDeg, &pitchDeg);
            hdApplyStabilisation(rollDeg, pitchDeg);
            hdApplyAltitudeControl(altValid, armAlt, vario);
            hdApplyLimits();

            if (!modeOn) {
                hdStartExit(HARDDECK_STATE_OFF);
            }
            else if (c < 0) {
                hdChangeState(HARDDECK_STATE_PULLUP);
            }
            else if (hd.releaseAlt > 0 && altValid &&
                     effAlt > hd.deckAlt + hd.releaseAlt &&
                     effAlt - hd.sigmaFactor * sigma > hd.deckAlt + hd.armMargin) {
                // Pilot climbed out: hand back control with the deck re-armed
                hdStartExit(HARDDECK_STATE_WATCH);
            }
            break;
        }

        case HARDDECK_STATE_EXIT:
            // Follow the switch during the blend: off -> release completely,
            // back on -> keep watching (the deck was already armed this flight)
            hd.exitTo = modeOn ? HARDDECK_STATE_WATCH : HARDDECK_STATE_OFF;

            if (modeOn && altValid && hd.predictedAlt < hd.deckAlt) {
                // Heading for the deck again while still blending out
                hdStartRecovery(armAlt, effAlt, vario);
            }
            else if (hdStateTime() > hd.exitTime) {
                hdChangeState(hd.exitTo);
            }
            break;
    }
}


//// Interface functions

int getHardDeckState(void)
{
    return hd.state;
}

bool hardDeckIsIntervening(void)
{
    return hd.state >= HARDDECK_STATE_PULLUP;
}

void hardDeckUpdate(void)
{
    if (!ARMING_FLAG(ARMED)) {
        hd.state = HARDDECK_STATE_OFF;
    }
    else {
        hdUpdateState();
    }

    float armAlt, vario, sigma;
    getAltitudeEstimate(&armAlt, &vario, &sigma);

    DEBUG(HARDDECK, 0, hd.state);
    DEBUG(HARDDECK, 1, lrintf(armAlt * 100));
    DEBUG(HARDDECK, 2, lrintf(sigma * 100));
    DEBUG(HARDDECK, 3, lrintf(hd.predictedAlt * 100));
    DEBUG(HARDDECK, 4, lrintf(hd.targetAlt * 100));
    DEBUG(HARDDECK, 5, lrintf(getBaroBias() * 100));
    DEBUG(HARDDECK, 6, lrintf(getBaroDisturbance() * 100));
    DEBUG(HARDDECK, 7, lrintf(hd.setpoint[FD_COLL]));
}

float hardDeckApply(uint8_t axis, float setpoint)
{
    switch (hd.state) {
        case HARDDECK_STATE_OFF:
        case HARDDECK_STATE_WAIT:
        case HARDDECK_STATE_WATCH:
            // Track the pilot so a takeover starts from the current setpoint
            hd.prevSetpoint[axis] = hd.setpoint[axis] = setpoint;
            return setpoint;

        case HARDDECK_STATE_EXIT:
        {
            const float alpha = constrainf((float)hdStateTime() / (float)(hd.exitTime + 1), 0.0f, 1.0f);
            if (axis == FD_YAW && hd.yawPassthrough) {
                return setpoint;
            }
            return alpha * setpoint + (1.0f - alpha) * hd.setpoint[axis];
        }

        default:
            if (axis == FD_YAW && hd.yawPassthrough) {
                hd.prevSetpoint[axis] = hd.setpoint[axis] = setpoint;
                return setpoint;
            }
            return hd.setpoint[axis];
    }
}

void INIT_CODE hardDeckInitProfile(const pidProfile_t *pidProfile)
{
    const pidHardDeckConfig_t *cfg = &pidProfile->harddeck;

    hd.deckAlt = cfg->altitude / 10.0f;
    hd.armMargin = cfg->arm_margin / 10.0f;
    hd.recoveryMargin = cfg->recovery_margin / 10.0f;
    hd.releaseAlt = cfg->release_altitude / 10.0f;
    hd.recoveryAccel = fmaxf(cfg->recovery_accel / 10.0f, 0.5f);
    hd.reactionTime = cfg->reaction_time / 1000.0f;
    hd.sigmaFactor = cfg->sigma_factor / 10.0f;
    hd.useAgl = cfg->use_agl;

    hd.levelGain = pidProfile->rescue.level_gain / 250.0f;
    hd.flipGain = pidProfile->rescue.flip_gain / 250.0f;
    hd.maxRate = pidProfile->rescue.max_setpoint_rate;
    hd.maxAccel = pidProfile->rescue.max_setpoint_accel * pidGetDT() * 10.0f;
    hd.pullUpCollective = pidProfile->rescue.pull_up_collective;
    hd.hoverCollective = pidProfile->rescue.hover_collective;
    // Recovery may use the full pull-up collective, not just the rescue
    // altitude-hold limit
    hd.maxCollective = fmaxf(pidProfile->rescue.max_collective, pidProfile->rescue.pull_up_collective);
    hd.pullUpTime = pidProfile->rescue.pull_up_time * 100;
    hd.flipTime = pidProfile->rescue.flip_time * 100;
    hd.exitTime = pidProfile->rescue.exit_time * 100;
    hd.vel_Kv = hd.hoverCollective / GRAVITY_MSS * HARDDECK_VEL_BANDWIDTH;
    hd.vel_Kiv = hd.vel_Kv * HARDDECK_VEL_I_RATIO * pidGetDT();

    hd.collDeadband = pidProfile->althold.stick_deadband / 1000.0f;
    hd.maxClimbRate = pidProfile->althold.max_climb_rate / 100.0f;
    hd.pos_Kp = pidProfile->poshold.pos_p_gain / 100.0f;
    hd.vel_Kp = pidProfile->poshold.vel_p_gain / 100.0f;
    hd.vel_Ki = (pidProfile->poshold.vel_i_gain / 100.0f) * pidGetDT();
    hd.maxHorizSpeed = pidProfile->poshold.max_horiz_speed;
    hd.maxTiltDeg = pidProfile->poshold.max_tilt_angle / 10.0f;
    hd.cyclicDeadband = pidProfile->poshold.stick_deadband / 1000.0f;
}
