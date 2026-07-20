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
 * Altitude hold controller for Rotorflight using MicroLink LIDAR/baro.
 *
 * Architecture (cascade PID inspired by INAV navigation_multicopter.c):
 *
 *   Pilot stick (collective)
 *        │  outside deadband: move target at rate ∝ deflection
 *        │  inside  deadband: hold current target
 *        ▼
 *   targetAlt ──[P alt]──> velCmd ──[P+I+D vel]──> collOutput
 *                              │                         │
 *                         clamped to                applies tilt
 *                       maxClimbRate               compensation
 *
 * The output overrides the normal collective setpoint only when
 * ALTHOLD_MODE is active.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/althold.h"

#include "pg/pid.h"


typedef struct {
    // Config (loaded from profile)
    float       Kp_alt;         // P gain: altitude error → velocity (1/s)
    float       Kp_vel;         // P gain: velocity error → collective
    float       Ki_vel;         // I gain (per-sample, already scaled by dT)
    float       Kd_vel;         // D gain applied to vario (derivative)
    float       maxClimbRate;   // m/s
    float       stickDeadband;  // collective stick fraction 0..1
    float       hoverCollective;// feed-forward hover point (0..1000)

    // State
    float       targetAlt;      // hold target in meters
    float       velIterm;       // velocity integrator
    bool        active;         // was mode active last cycle (for init)
} altHoldState_t;

static FAST_DATA_ZERO_INIT altHoldState_t ah;


/*
 * Determine which altitude source to use.
 * Prefer LIDAR AGL when valid, fall back to baro/GPS.
 */
static float getCurrentAlt(void)
{
#ifdef USE_RANGEFINDER
    if (isAGLAltitudeValid()) {
        return getAGLAltitude();
    }
#endif
    return getAltitude();
}

static float getCurrentVario(void)
{
#ifdef USE_RANGEFINDER
    if (isAGLAltitudeValid()) {
        return getAGLVario();
    }
#endif
    return getVario();
}

/*
 * Called every PID loop iteration (8 kHz) to update hold state.
 * Reads RC collective stick to update the altitude target.
 */
void altHoldUpdate(void)
{
    if (!FLIGHT_MODE(ALTHOLD_MODE)) {
        ah.active = false;
        ah.velIterm = 0;
        return;
    }

    // On first entry: latch current altitude as target
    if (!ah.active) {
        ah.targetAlt = getCurrentAlt();
        ah.velIterm = 0;
        ah.active = true;
    }

    // RC collective stick: -1.0 (full down) to +1.0 (full up)
    // getRcDeflection returns -1..+1
    const float stickDeflection = getRcDeflection(FD_COLL);

    if (fabsf(stickDeflection) > ah.stickDeadband) {
        // Outside deadband: pilot is commanding a climb/descent rate
        const float sign = (stickDeflection > 0) ? 1.0f : -1.0f;
        const float rate = (fabsf(stickDeflection) - ah.stickDeadband) /
                           (1.0f - ah.stickDeadband);
        ah.targetAlt += sign * rate * ah.maxClimbRate * pidGetDT();
    }
    // Inside deadband: targetAlt stays frozen
}

/*
 * Called from pidApplyCollective() in pid.c.
 * Returns the altitude-hold collective override, or the original
 * setpoint when ALTHOLD_MODE is not active.
 */
float altHoldApply(float collective)
{
    if (!FLIGHT_MODE(ALTHOLD_MODE)) {
        return collective;
    }

    const float currentAlt  = getCurrentAlt();
    const float currentVario = getCurrentVario();
    const float tilt         = getCosTiltAngle();
    const float tiltFactor   = tilt * tilt; // gravity compensation

    // Outer loop: altitude error → velocity setpoint
    const float altError = ah.targetAlt - currentAlt;
    const float velCmd   = constrainf(ah.Kp_alt * altError,
                                      -ah.maxClimbRate, ah.maxClimbRate);

    // Inner loop: velocity error → collective
    const float velError = velCmd - currentVario;

    const float Pterm = ah.Kp_vel * velError;
    const float Dterm = ah.Kd_vel * currentVario; // D on measurement (no derivative kick)

    ah.velIterm = constrainf(ah.velIterm + ah.Ki_vel * velError,
                             0.0f, ah.hoverCollective);

    float output = (ah.hoverCollective + Pterm + ah.velIterm + Dterm) * tiltFactor;
    output = constrainf(output, 0.0f, 1000.0f);

    DEBUG(ALTHOLD, 4, (int32_t)(altError * 100));
    DEBUG(ALTHOLD, 5, (int32_t)(velCmd * 100));
    DEBUG(ALTHOLD, 6, (int32_t)output);
    DEBUG(ALTHOLD, 7, (int32_t)(ah.targetAlt * 100));

    return output;
}

void INIT_CODE altHoldInitProfile(const pidProfile_t *pidProfile)
{
    const pidAltHoldConfig_t *cfg = &pidProfile->althold;

    ah.Kp_alt        = cfg->alt_p_gain / 10.0f;
    ah.Kp_vel        = cfg->alt_p_gain / 10.0f; // reuse P for inner loop (tune separately)
    ah.Ki_vel        = cfg->alt_i_gain * pidGetDT() / 10.0f;
    ah.Kd_vel        = cfg->alt_d_gain / -10.0f; // negative: damping on climb
    ah.maxClimbRate  = cfg->max_climb_rate / 100.0f; // cm/s to m/s
    ah.stickDeadband = cfg->stick_deadband / 1000.0f;
    ah.hoverCollective = cfg->hover_collective;

    ah.active   = false;
    ah.velIterm = 0;
    ah.targetAlt = 0;
}
