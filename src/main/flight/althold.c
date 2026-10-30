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
 *   Pilot stick (collective), relative to where it was when the hold engaged
 *        │  outside deadband: move target at rate ∝ deflection
 *        │  inside  deadband: hold current target
 *        ▼
 *   targetAlt ──[alt_p]──> velCmd ──[vel gain + I]──> collOutput
 *                              │                          │
 *                         clamped to                 applies tilt
 *                       maxClimbRate                compensation
 *
 * Gains (per PID profile):
 *   althold_alt_p_gain / 10        altitude error (m) -> climb rate (m/s)
 *   althold_alt_d_gain * 5         climb rate error (m/s) -> collective
 *   althold_alt_i_gain * 10        integral of the climb rate error -> collective
 * Collective is in the 0..1000 units of the collective setpoint.
 *
 * Engaging is bumpless: the integrator starts at the collective the pilot
 * (or rescue) was flying with, so althold_hover_collective only matters as
 * the fallback while the altitude is invalid.  Switching the mode off blends
 * from the hold's collective back to the stick over AH_EXIT_BLEND_S: the
 * stick is usually not at the hover point then (the first flight dropped
 * 85 cm in 0.6 s when the hold was switched off with the stick in the
 * descend position).  Rescue and failsafe take over at once.
 *
 * The output overrides the normal collective setpoint only when
 * ALTHOLD_MODE is active and no rescue is in control of the collective.
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
#include "flight/rescue.h"
#include "flight/althold.h"

#include "pg/pid.h"


typedef struct {
    // Config (loaded from profile)
    float       Kp_alt;         // altitude error (m) → velocity (m/s), 1/s
    float       Kp_vel;         // velocity error (m/s) → collective
    float       Ki_vel;         // velocity error integral → collective (per sample, scaled by dT)
    float       maxClimbRate;   // m/s
    float       stickDeadband;  // collective stick fraction 0..1
    float       hoverCollective;// feed-forward hover point (0..1000)

    // State
    float       targetAlt;      // hold target in meters
    float       velIterm;       // velocity integrator
    float       lastAlt;        // previous cycle's measurement (frame tracking)
    bool        usingAgl;       // which frame targetAlt is expressed in
    bool        stickActive;    // collective outside the deadband this cycle
    float       stickRef;       // collective stick (-1..1) when the hold engaged
    bool        bumpless;       // preload the integrator on the next output

    // Last cycle, for altHoldGetStatus()
    uint8_t     statusFlags;
    float       statusOutput;

    // Smooth exit: output blends from exitFrom to the stick while
    // exitBlend runs from 1 to 0
    float       exitFrom;
    float       exitBlend;
    bool        active;         // was mode active last cycle (for init)
} altHoldState_t;

static FAST_DATA_ZERO_INIT altHoldState_t ah;

#define AH_EXIT_BLEND_S         1.0f    // stick takeover after switching off


/*
 * Determine which altitude source to use.
 * Prefer LIDAR AGL when valid, fall back to baro/GPS.
 */
static bool usingAglSource(void)
{
#ifdef USE_RANGEFINDER
    return isAGLAltitudeValid();
#else
    return false;
#endif
}

// True when the altitude actually being fed to the loop is trustworthy.
// getAltitude() returns 0 when the Z estimate is stale, which would otherwise
// look like a huge altitude error and command full collective.
static bool altSourceIsValid(void)
{
    return usingAglSource() || isAltitudeValid();
}

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
/*
 * Altitude hold only owns the collective when its mode is on and no rescue
 * function is flying the helicopter.  Rescue (including its exit blend), GPS
 * rescue and failsafe take priority; while they are in control the hold is
 * disengaged so it re-latches the altitude afterwards instead of resuming
 * with a stale target and a wound-up integrator.
 */
static bool altHoldEngaged(void)
{
    return FLIGHT_MODE(ALTHOLD_MODE) &&
           !FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE) &&
           getRescueState() == RESCUE_STATE_OFF;
}

void altHoldUpdate(void)
{
    if (!altHoldEngaged()) {
        ah.active = false;
        ah.velIterm = 0;
        ah.stickActive = false;
        return;
    }

    const bool usingAgl = usingAglSource();
    const float currentAlt = getCurrentAlt();

    // RC collective stick: -1.0 (full down) to +1.0 (full up)
    const float stick = getRcDeflection(FD_COLL);

    // On first entry: latch the current altitude as target and the current
    // stick position as "hold".  A helicopter hovers with the collective
    // stick well off centre, so the stick is read relative to this point.
    if (!ah.active) {
        ah.targetAlt = currentAlt;
        ah.velIterm = 0;
        ah.usingAgl = usingAgl;
        ah.lastAlt = currentAlt;
        ah.stickRef = constrainf(stick, -0.9f, 0.9f);
        ah.bumpless = true;
        ah.active = true;
    }

    // The AGL (terrain-relative) and baro/GPS (arm-relative) sources live in
    // different frames.  When the loop switches between them mid-flight, shift
    // the target by the frame offset so the altitude error - and therefore the
    // collective - stays continuous instead of stepping by the offset.
    if (usingAgl != ah.usingAgl) {
        ah.targetAlt += currentAlt - ah.lastAlt;
        ah.usingAgl = usingAgl;
    }
    ah.lastAlt = currentAlt;

    if (!altSourceIsValid()) {
        // No usable altitude: freeze the target rather than integrating the
        // stick into a target the loop cannot track.
        return;
    }

    // Stick travel from the engage point, -1..1: full up or down from there
    // commands the full climb or descent rate
    const float travel = (stick >= ah.stickRef) ?
        (stick - ah.stickRef) / (1.0f - ah.stickRef) :
        (stick - ah.stickRef) / (1.0f + ah.stickRef);

    ah.stickActive = fabsf(travel) > ah.stickDeadband;

    if (ah.stickActive) {
        // Outside deadband: pilot is commanding a climb/descent rate
        const float sign = (travel > 0) ? 1.0f : -1.0f;
        const float rate = (fabsf(travel) - ah.stickDeadband) /
                           (1.0f - ah.stickDeadband);
        ah.targetAlt += sign * rate * ah.maxClimbRate * pidGetDT();
    }
    // Inside deadband: targetAlt stays frozen
}

/*
 * DEBUG_ALTHOLD: everything needed to tune the loop from one blackbox log.
 * Terms are in collective units (0..1000) x10, the velocity command in cm/s.
 */
#define AH_FLAG_ENGAGED         AH_STATUS_ENGAGED
#define AH_FLAG_USING_AGL       AH_STATUS_USING_AGL
#define AH_FLAG_SOURCE_VALID    AH_STATUS_SOURCE_VALID
#define AH_FLAG_STICK           AH_STATUS_STICK     // pilot moving the target
#define AH_FLAG_YIELDED         AH_STATUS_YIELDED   // mode on, but a rescue is in control
#define AH_FLAG_EXIT_BLEND      AH_STATUS_EXIT_BLEND

static void altHoldDebug(float output, float Pterm, float velCmd, uint32_t flags)
{
    ah.statusFlags = flags;
    ah.statusOutput = output;

    DEBUG(ALTHOLD, 0, lrintf(ah.targetAlt * 100));
    DEBUG(ALTHOLD, 1, lrintf(getCurrentAlt() * 100));
    DEBUG(ALTHOLD, 2, lrintf(getCurrentVario() * 100));
    DEBUG(ALTHOLD, 3, lrintf(Pterm * 10));
    DEBUG(ALTHOLD, 4, lrintf(ah.velIterm * 10));
    DEBUG(ALTHOLD, 5, lrintf(velCmd * 100));
    DEBUG(ALTHOLD, 6, lrintf(output));
    DEBUG(ALTHOLD, 7, (int32_t)flags);
}

/*
 * Called from pidApplyCollective() in pid.c, after rescueApply().
 * Returns the altitude-hold collective override, or the incoming
 * (pilot or rescue) collective when the hold is not engaged.
 */
float altHoldApply(float collective)
{
    uint32_t flags = 0;
    if (usingAglSource()) {
        flags |= AH_FLAG_USING_AGL;
    }
    if (altSourceIsValid()) {
        flags |= AH_FLAG_SOURCE_VALID;
    }

    if (!altHoldEngaged()) {
        float output = collective;

        if (FLIGHT_MODE(ALTHOLD_MODE)) {
            // A rescue or failsafe is in control: it takes over at once
            flags |= AH_FLAG_YIELDED;
            ah.exitBlend = 0;
        }
        else if (ah.exitBlend > 0) {
            // Switched off by the pilot: hand the collective back gradually
            output = collective + (ah.exitFrom - collective) * ah.exitBlend;
            ah.exitBlend = fmaxf(ah.exitBlend - pidGetDT() / AH_EXIT_BLEND_S, 0.0f);
            flags |= AH_FLAG_EXIT_BLEND;
        }

        // Re-engaging during the blend continues from this output
        ah.exitFrom = output;
        altHoldDebug(output, 0, 0, flags);
        return output;
    }

    flags |= AH_FLAG_ENGAGED;
    if (ah.stickActive) {
        flags |= AH_FLAG_STICK;
    }

    const float tilt         = getCosTiltAngle();
    const float tiltFactor   = tilt * tilt; // gravity compensation

    // Bumpless engage: start from the collective that was flying the model
    // (the previous output, which is the stick unless an exit blend was
    // still running), so the output does not step to althold_hover_collective
    if (ah.bumpless) {
        ah.velIterm = constrainf(ah.exitFrom / fmaxf(tiltFactor, 0.5f) - ah.hoverCollective,
                                 -ah.hoverCollective, ah.hoverCollective);
        ah.bumpless = false;
    }

    // Arm the smooth exit for when the mode is switched off
    ah.exitBlend = 1.0f;

    if (!altSourceIsValid()) {
        // Hold the last trimmed hover point; do not close the loop on a
        // stale/absent altitude estimate.
        const float output = constrainf((ah.hoverCollective + ah.velIterm) * tiltFactor, 0.0f, 1000.0f);
        ah.exitFrom = output;
        altHoldDebug(output, 0, 0, flags);
        return output;
    }

    const float currentAlt   = getCurrentAlt();
    const float currentVario = getCurrentVario();

    // Outer loop: altitude error → velocity setpoint
    const float altError = ah.targetAlt - currentAlt;
    const float velCmd   = constrainf(ah.Kp_alt * altError,
                                      -ah.maxClimbRate, ah.maxClimbRate);

    // Inner loop: velocity error → collective
    const float velError = velCmd - currentVario;

    const float Pterm = ah.Kp_vel * velError;

    // The integrator must be able to trim in both directions: clamped to
    // [0, hover] it could only ever add collective, so an over-estimated
    // hover_collective would leave a permanent climb it cannot cancel.
    ah.velIterm = constrainf(ah.velIterm + ah.Ki_vel * velError,
                             -ah.hoverCollective, ah.hoverCollective);

    float output = (ah.hoverCollective + Pterm + ah.velIterm) * tiltFactor;
    output = constrainf(output, 0.0f, 1000.0f);

    ah.exitFrom = output;
    altHoldDebug(output, Pterm, velCmd, flags);

    return output;
}

void altHoldGetStatus(uint8_t *flags, float *targetAlt, float *currentAlt, float *output)
{
    *flags = ah.statusFlags;
    *targetAlt = ah.targetAlt;
    *currentAlt = getCurrentAlt();
    *output = ah.statusOutput;
}

void INIT_CODE altHoldInitProfile(const pidProfile_t *pidProfile)
{
    const pidAltHoldConfig_t *cfg = &pidProfile->althold;

    ah.Kp_alt        = cfg->alt_p_gain / 10.0f;          // 10 -> 1 m/s per m
    ah.Kp_vel        = cfg->alt_d_gain * 5.0f;           // 15 -> 75 per m/s
    ah.Ki_vel        = cfg->alt_i_gain * 10.0f * pidGetDT(); // 5 -> 50 per m/s per s
    ah.maxClimbRate  = cfg->max_climb_rate / 100.0f; // cm/s to m/s
    ah.stickDeadband = cfg->stick_deadband / 1000.0f;
    ah.hoverCollective = cfg->hover_collective;

    ah.active   = false;
    ah.velIterm = 0;
    ah.bumpless = false;
    ah.stickRef = 0;
    ah.exitBlend = 0;
    ah.targetAlt = 0;
    ah.lastAlt = 0;
    ah.usingAgl = false;
}
