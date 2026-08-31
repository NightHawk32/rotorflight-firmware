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

#include "platform.h"

#ifdef USE_OPTICAL_FLOW

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/poshold.h"

#include "pg/pid.h"


// Angle output in centidegrees (same as gpsRescueAngle[])
int32_t posHoldAngle[2];


typedef struct {
    // Config
    float   Kp_pos;         // Position error (cm) → velocity (cm/s)
    float   Kp_vel;         // Velocity error (cm/s) → angle (degrees)
    float   Ki_vel;         // Velocity error (cm/s) → angle rate (deg per cycle)
    float   maxHorizSpeed;  // cm/s
    float   maxTiltDeg;     // degrees
    float   stickDeadband;  // fraction 0..1

    // State
    float   holdX;          // hold target East  (cm)
    float   holdY;          // hold target North (cm)
    float   iTermEast;      // wind trim, earth frame (degrees of tilt)
    float   iTermNorth;
    bool    active;
} posHoldState_t;

static FAST_DATA_ZERO_INIT posHoldState_t ph;


// True while the controller is engaged and producing valid output.  When this
// is false posHoldAngle[] is zero and leveling.c must fall back to normal
// stick-driven angle control so the pilot never loses authority.
bool posHoldIsActive(void)
{
    return ph.active;
}

void posHoldUpdate(void)
{
    // Clear output every cycle; only fill it when mode is active and valid
    posHoldAngle[AI_ROLL]  = 0;
    posHoldAngle[AI_PITCH] = 0;

    if (!ARMING_FLAG(ARMED) || !FLIGHT_MODE(POSHOLD_MODE)) {
        ph.active = false;
        return;
    }

    // Safety: require optical flow XY estimate and altitude hold
    if (!isPositionXYValid() || !FLIGHT_MODE(ALTHOLD_MODE)) {
        ph.active = false;
        return;
    }

    // On first entry: latch current position as hold target
    if (!ph.active) {
        ph.holdX = getPositionXCm();
        ph.holdY = getPositionYCm();
        ph.iTermEast = 0;
        ph.iTermNorth = 0;
        ph.active = true;
    }

    // --- RC stick input: move hold target ---
    // getRcDeflection returns -1..+1
    const float stickRoll  = getRcDeflection(FD_ROLL);
    const float stickPitch = getRcDeflection(FD_PITCH);

    // Body-frame stick rates (right/forward, cm/s), before rotation to earth frame
    float bodyRightRate   = 0.0f;
    float bodyForwardRate = 0.0f;

    if (fabsf(stickRoll) > ph.stickDeadband) {
        const float sign = (stickRoll > 0) ? 1.0f : -1.0f;
        const float rate = (fabsf(stickRoll) - ph.stickDeadband) /
                           (1.0f - ph.stickDeadband);
        bodyRightRate = sign * rate * ph.maxHorizSpeed;
    }
    if (fabsf(stickPitch) > ph.stickDeadband) {
        const float sign = (stickPitch > 0) ? 1.0f : -1.0f;
        const float rate = (fabsf(stickPitch) - ph.stickDeadband) /
                           (1.0f - ph.stickDeadband);
        bodyForwardRate = sign * rate * ph.maxHorizSpeed;
    }

    // Heading rotation between the body frame (forward/right) and the
    // earth frame (North/East).  Note this 2x2 is its own inverse, so the
    // same cos/sin pair is used in both directions below.
    const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    const float cosYaw = cos_approx(yawRad);
    const float sinYaw = sin_approx(yawRad);

    if (bodyRightRate != 0.0f || bodyForwardRate != 0.0f) {
        // Rotate body-frame (right, forward) stick rates into earth-frame
        // (East, North) using the current heading, so the hold target moves
        // in the direction the pilot is actually commanding regardless of yaw.
        const float eastRate  = bodyForwardRate * sinYaw + bodyRightRate * cosYaw;
        const float northRate = bodyForwardRate * cosYaw - bodyRightRate * sinYaw;

        ph.holdX += eastRate  * pidGetDT();
        ph.holdY += northRate * pidGetDT();
    }

    // --- Outer loop: position error → velocity command ---
    const float posErrX = ph.holdX - getPositionXCm();
    const float posErrY = ph.holdY - getPositionYCm();

    float velCmdX = constrainf(ph.Kp_pos * posErrX, -ph.maxHorizSpeed, ph.maxHorizSpeed);
    float velCmdY = constrainf(ph.Kp_pos * posErrY, -ph.maxHorizSpeed, ph.maxHorizSpeed);

    // --- Inner loop: velocity error → angle command ---
    const float velErrEast  = velCmdX - getVelocityXCms();
    const float velErrNorth = velCmdY - getVelocityYCms();

    // P + I, accumulated in the EARTH frame.  Holding station in wind needs a
    // steady tilt into the wind; P alone can only produce that from a standing
    // position error, so a P-only cascade parks the aircraft permanently
    // downwind of the target.  The integrator supplies that trim instead, and
    // because velocity error is only zero when the position error is zero, it
    // drives the steady-state position error to zero.
    //
    // Earth-referenced (rather than roll/pitch-referenced) so the learned wind
    // trim survives a yaw change instead of being re-learned after every
    // pirouette.
    const float tiltEast  = ph.Kp_vel * velErrEast  + ph.iTermEast;
    const float tiltNorth = ph.Kp_vel * velErrNorth + ph.iTermNorth;

    // The demand is earth-frame (East/North) but the tilt command is
    // body-frame (roll = right, pitch = forward), so rotate by the current
    // heading.  Without this the controller only pushes in the right
    // direction while pointing North, and inverts near a South heading.
    const float tiltFwd   = tiltEast * sinYaw + tiltNorth * cosYaw;
    const float tiltRight = tiltEast * cosYaw - tiltNorth * sinYaw;

    // Rotorflight level-mode convention: positive roll angle = right,
    // positive pitch angle = nose down = forward (attitude.values.pitch is
    // negative nose-up), so both map directly onto the body-frame demand.
    float angleDegRoll  = constrainf(tiltRight, -ph.maxTiltDeg, ph.maxTiltDeg);
    float angleDegPitch = constrainf(tiltFwd,   -ph.maxTiltDeg, ph.maxTiltDeg);

    // Anti-windup: stop accumulating once the tilt command is clipped, and
    // bound the trim itself to the configured tilt limit.
    if (angleDegRoll == tiltRight && angleDegPitch == tiltFwd) {
        ph.iTermEast  += ph.Ki_vel * velErrEast;
        ph.iTermNorth += ph.Ki_vel * velErrNorth;

        const float iMag = sqrtf(ph.iTermEast * ph.iTermEast +
                                 ph.iTermNorth * ph.iTermNorth);
        if (iMag > ph.maxTiltDeg) {
            const float scale = ph.maxTiltDeg / iMag;
            ph.iTermEast  *= scale;
            ph.iTermNorth *= scale;
        }
    }

    // Convert to centidegrees and write to output
    posHoldAngle[AI_ROLL]  = (int32_t)(angleDegRoll  * 100.0f);
    posHoldAngle[AI_PITCH] = (int32_t)(angleDegPitch * 100.0f);

    // positionUpdate() runs earlier in the same cycle and owns DEBUG_POSHOLD
    // 0..5 (estimated position/velocity); only 6 and 7 are ours to write.
    DEBUG(POSHOLD, 6, posHoldAngle[AI_ROLL]);
    DEBUG(POSHOLD, 7, posHoldAngle[AI_PITCH]);
}

void INIT_CODE posHoldInitProfile(const pidProfile_t *pidProfile)
{
    const pidPosHoldConfig_t *cfg = &pidProfile->poshold;

    ph.Kp_pos       = cfg->pos_p_gain / 100.0f;
    ph.Kp_vel       = cfg->vel_p_gain / 100.0f;
    // Per-cycle increment: deg of tilt per (cm/s of error) per second, x dT
    ph.Ki_vel       = (cfg->vel_i_gain / 100.0f) * pidGetDT();
    ph.maxHorizSpeed = cfg->max_horiz_speed;
    ph.maxTiltDeg   = cfg->max_tilt_angle / 10.0f;
    ph.stickDeadband = cfg->stick_deadband / 1000.0f;

    ph.active = false;
    ph.holdX = 0;
    ph.holdY = 0;
    ph.iTermEast = 0;
    ph.iTermNorth = 0;
}

#endif // USE_OPTICAL_FLOW
