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
    float   maxHorizSpeed;  // cm/s
    float   maxTiltDeg;     // degrees
    float   stickDeadband;  // fraction 0..1

    // State
    float   holdX;          // hold target East  (cm)
    float   holdY;          // hold target North (cm)
    bool    active;
} posHoldState_t;

static FAST_DATA_ZERO_INIT posHoldState_t ph;


void posHoldUpdate(void)
{
    // Clear output every cycle; only fill it when mode is active and valid
    posHoldAngle[AI_ROLL]  = 0;
    posHoldAngle[AI_PITCH] = 0;

    if (!FLIGHT_MODE(POSHOLD_MODE)) {
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

    if (bodyRightRate != 0.0f || bodyForwardRate != 0.0f) {
        // Rotate body-frame (right, forward) stick rates into earth-frame
        // (East, North) using the current heading, so the hold target moves
        // in the direction the pilot is actually commanding regardless of yaw.
        const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
        const float cosYaw = cos_approx(yawRad);
        const float sinYaw = sin_approx(yawRad);

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
    const float velErrX = velCmdX - getVelocityXCms();
    const float velErrY = velCmdY - getVelocityYCms();

    float angleDegRoll  = constrainf(ph.Kp_vel * velErrX, -ph.maxTiltDeg, ph.maxTiltDeg);
    float angleDegPitch = constrainf(ph.Kp_vel * velErrY, -ph.maxTiltDeg, ph.maxTiltDeg);

    // Convert to centidegrees and write to output
    posHoldAngle[AI_ROLL]  = (int32_t)(angleDegRoll  * 100.0f);
    posHoldAngle[AI_PITCH] = (int32_t)(angleDegPitch * 100.0f);

    DEBUG(POSHOLD, 0, (int32_t)posErrX);
    DEBUG(POSHOLD, 1, (int32_t)posErrY);
    DEBUG(POSHOLD, 2, posHoldAngle[AI_ROLL]);
    DEBUG(POSHOLD, 3, posHoldAngle[AI_PITCH]);
    DEBUG(POSHOLD, 4, (int32_t)ph.holdX);
    DEBUG(POSHOLD, 5, (int32_t)ph.holdY);
}

void INIT_CODE posHoldInitProfile(const pidProfile_t *pidProfile)
{
    const pidPosHoldConfig_t *cfg = &pidProfile->poshold;

    ph.Kp_pos       = cfg->pos_p_gain / 100.0f;
    ph.Kp_vel       = cfg->vel_p_gain / 100.0f;
    ph.maxHorizSpeed = cfg->max_horiz_speed;
    ph.maxTiltDeg   = cfg->max_tilt_angle / 10.0f;
    ph.stickDeadband = cfg->stick_deadband / 1000.0f;

    ph.active = false;
    ph.holdX = 0;
    ph.holdY = 0;
}

#endif // USE_OPTICAL_FLOW
