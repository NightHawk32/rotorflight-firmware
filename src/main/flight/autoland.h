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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "flight/pid.h"

/*
 * The values are logged (DEBUG_AUTOLAND[0]) and sent over MSP, so they are
 * fixed: new states are appended.
 */
typedef enum {
    AUTOLAND_STATE_OFF = 0,         // not in control
    AUTOLAND_STATE_SETTLE,          // hold altitude and position, let the model settle
    AUTOLAND_STATE_DESCEND,         // descend at autoland_descent_rate (valid AGL, high)
    AUTOLAND_STATE_FINAL,           // descend at autoland_final_rate
    AUTOLAND_STATE_GROUND_EFFECT,   // collective pushed below hover, open loop
    AUTOLAND_STATE_TOUCHDOWN,       // collective ramps to autoland_ground_collective
    AUTOLAND_STATE_LANDED,          // on the ground, negative pitch, motor running
    AUTOLAND_STATE_MOTOR_OFF,       // throttle cut, negative pitch, disarm pending
    // Auto takeoff
    AUTOLAND_STATE_GROUND_IDLE,     // ALTHOLD on while landed: waiting for the stick push
    AUTOLAND_STATE_TAKEOFF_RAMP,    // collective ramps up until liftoff
    AUTOLAND_STATE_TAKEOFF_CLIMB,   // climbs to takeoff_altitude above the takeoff point
} autolandState_e;

// Position hold behaviour requested by the autoland
typedef enum {
    AUTOLAND_POSHOLD_NORMAL = 0,
    AUTOLAND_POSHOLD_NEAR_GROUND,   // integrator frozen, reduced tilt limit
    AUTOLAND_POSHOLD_GROUND,        // level, no position correction (skids on the ground)
} autolandPosHoldMode_e;

// AL_STATUS_* flags, as in DEBUG_AUTOLAND[7] and MSP
#define AL_STATUS_SWITCH        (1 << 0)    // AUTOLAND switch on
#define AL_STATUS_AVAILABLE     (1 << 1)    // could engage now
#define AL_STATUS_REFUSED       (1 << 2)    // switched on while unavailable
#define AL_STATUS_LIDAR         (1 << 3)    // rangefinder detected and alive
#define AL_STATUS_GPS           (1 << 4)    // GPS fix, sats, North-referenced heading
#define AL_STATUS_AGL_VALID     (1 << 5)
#define AL_STATUS_MOTOR_CUT     (1 << 6)
#define AL_STATUS_YIELDED       (1 << 7)    // a rescue or failsafe took over
#define AL_STATUS_TD_VARIO      (1 << 8)    // touchdown detector: not sinking
#define AL_STATUS_TD_COLL       (1 << 9)    // touchdown detector: collective below hover
#define AL_STATUS_TD_TILT       (1 << 10)   // touchdown detector: level
#define AL_STATUS_FALLING       (1 << 11)   // false touchdown / push-down too fast: reverted
#define AL_STATUS_STICK         (1 << 12)   // pilot moving the target
#define AL_STATUS_LANDED        (1 << 13)   // on the ground since arming: takeoff possible
#define AL_STATUS_PUSHED        (1 << 14)   // takeoff: stick pushed, waiting for the return
#define AL_STATUS_TO_ABORTED    (1 << 15)   // last takeoff ramp aborted

typedef struct {
    uint8_t     state;          // autolandState_e
    uint16_t    flags;          // AL_STATUS_*
    uint32_t    stateTimeMs;
    float       learnedHover;   // collective, 0..1000
    float       collective;     // last collective output
    float       climbRate;      // commanded climb rate, m/s
} autolandStatus_t;

void autolandInitProfile(const pidProfile_t *pidProfile);

// Setpoint task, before altHoldUpdate()
void autolandUpdate(void);

// PID loop: collective after altHoldApply()
float autolandApplyCollective(float collective);

autolandState_e getAutolandState(void);
void autolandGetStatus(autolandStatus_t *status);

// The autoland (or takeoff) is flying or holding the model on the ground
bool autolandIsActive(void);
// In one of the takeoff states
bool autolandIsTakeoff(void);
// Altitude hold must be engaged (even with ALTHOLD off)
bool autolandControlsAltitude(void);
// Climb rate for altitude hold (m/s, negative = down)
float autolandGetClimbRate(void);
// The autoland sets the collective itself: altitude hold freezes its integrator
bool autolandOverridesCollective(void);
// On the ground: altitude hold must not engage (even with ALTHOLD on)
bool autolandSuppressesAltHold(void);
// Position hold must be engaged (even with POSHOLD off)
bool autolandWantsPosHold(void);
autolandPosHoldMode_e autolandPosHoldMode(void);
// Still flying: the airborne detector must not report landed
bool autolandKeepsAirborne(void);
// Throttle is cut until disarm
bool autolandIsMotorCut(void);
