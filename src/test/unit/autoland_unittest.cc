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
 * Closed-loop tests for the auto land and auto takeoff: autoland.c and
 * althold.c fly a vertical point-mass helicopter with ground contact,
 * ground effect and a LIDAR that can be made invalid near the ground or
 * see an obstacle.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "common/axis.h"
#include "common/maths.h"
#include "build/debug.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/althold.h"
#include "flight/autoland.h"
#include "flight/rescue.h"
#include "io/beeper.h"
#include "pg/pid.h"
}

#define SIM_DT          0.001f
#define GRAVITY         9.81f
#define TRUE_HOVER      380.0f      // collective that hovers out of ground effect
#define GE_HEIGHT       0.5f        // ground effect below this (m)
#define GE_GAIN         0.15f       // extra thrust on the ground
#define LIDAR_OFFSET    0.05f       // LIDAR reads this on the ground

// ---- Simulated world ----

static struct {
    uint32_t    timeUs;
    float       h;              // m above the ground (arm point = ground)
    float       v;              // m/s
    float       coll;           // last collective applied
    float       stick;          // pilot collective deflection
    bool        lidarAlive;
    float       lidarMin;       // LIDAR invalid below this height
    float       obstacleAgl;    // > 0: LIDAR sees this AGL while h > obstacleAgl
    bool        gpsReady;
    bool        rescue;
    bool        disarmed;
    int         disarmReason;
    int         beeps;
    float       touchdownSpeed;
    bool        touched;
    bool        notSpooled;
    bool        tiedDown;       // skids held on the ground
    float       accZ;           // g, body Z (1 at rest)
} sim;

static pidProfile_t profile;

extern "C" {
uint8_t armingFlags;
uint32_t armingDisableFlags;
uint16_t flightModeFlags;
uint8_t debugMode;
int32_t debug[DEBUG_VALUE_COUNT];
attitudeEulerAngles_t attitude;

uint32_t millis(void) { return sim.timeUs / 1000; }
float pidGetDT(void) { return SIM_DT; }
float getRcDeflection(int axis) { return axis == FD_COLL ? sim.stick : 0; }
float getCosTiltAngle(void) { return 1.0f; }

bool sensors(uint32_t) { return true; }
bool rangefinderIsHealthy(void) { return sim.lidarAlive; }
bool isSpooledUp(void) { return ARMING_FLAG(ARMED) && !sim.notSpooled; }
bool airborneAccZLiftoff(void) { return sim.accZ > 1.15f; }
bool isAirborne(void) { return sim.h > 0.01f; }
int getRescueState(void) { return sim.rescue ? 1 : RESCUE_STATE_OFF; }

uint16_t enableFlightMode(flightModeFlags_e mask) { return flightModeFlags |= mask; }
uint16_t disableFlightMode(flightModeFlags_e mask) { return flightModeFlags &= ~mask; }

void setArmingDisabled(armingDisableFlags_e flag) { armingDisableFlags |= flag; }
void disarm(flightLogDisarmReason_e reason)
{
    DISABLE_ARMING_FLAG(ARMED);
    sim.disarmed = true;
    sim.disarmReason = reason;
}
void beeper(beeperMode_e) { sim.beeps++; }

static bool simAglValid(void)
{
    if (sim.obstacleAgl > 0 && sim.h > sim.obstacleAgl)
        return true;
    return sim.lidarAlive && sim.h + LIDAR_OFFSET >= sim.lidarMin && sim.h < 12.0f;
}

bool isAGLAltitudeValid(void) { return simAglValid(); }
float getAGLAltitude(void)
{
    if (sim.obstacleAgl > 0 && sim.h > sim.obstacleAgl)
        return sim.obstacleAgl;
    return simAglValid() ? sim.h + LIDAR_OFFSET : 0;
}
float getAGLVario(void) { return sim.v; }
float getAltitude(void) { return sim.h; }
float getVario(void) { return sim.v; }
bool isAltitudeValid(void) { return true; }

bool isPositionGpsReady(void) { return sim.gpsReady; }
bool isPositionXYValid(void) { return true; }
float getVelocityXCms(void) { return 0; }
float getVelocityYCms(void) { return 0; }
}

static void simStep(void)
{
    autolandUpdate();
    altHoldUpdate();
    float coll = altHoldApply(sim.stick * 1000.0f);
    coll = autolandApplyCollective(coll);
    sim.coll = coll;

    float accel = -GRAVITY;
    if (ARMING_FLAG(ARMED) && !autolandIsMotorCut() && isSpooledUp()) {
        const float ge = 1.0f + GE_GAIN * fmaxf(0.0f, 1.0f - sim.h / GE_HEIGHT);
        accel = GRAVITY * fmaxf(coll, 0.0f) / TRUE_HOVER * ge - GRAVITY - 0.5f * sim.v;
    }
    if (sim.tiedDown && sim.h <= 0) {
        accel = fminf(accel, 0.0f);
    }
    sim.v += accel * SIM_DT;
    // Accelerometer: the ground reaction cancels a downward net force
    sim.accZ = (sim.h > 0 || accel > 0) ? 1.0f + accel / GRAVITY : 1.0f;
    sim.h += sim.v * SIM_DT;
    if (sim.h <= 0) {
        if (!sim.touched && sim.v < 0) {
            sim.touched = true;
            sim.touchdownSpeed = -sim.v;
        }
        sim.h = 0;
        sim.v = 0;
    }

    sim.timeUs += 1000;
}

static void simRun(float seconds)
{
    for (int i = 0; i < lrintf(seconds / SIM_DT); i++) {
        simStep();
    }
}

// Run until the state is reached (or the time is up); returns the time taken
static float simRunUntil(autolandState_e state, float maxSeconds)
{
    for (int i = 0; i < lrintf(maxSeconds / SIM_DT); i++) {
        if (getAutolandState() == state)
            return i * SIM_DT;
        simStep();
    }
    return -1;
}

static void autolandSwitch(bool on)
{
    if (on)
        ENABLE_FLIGHT_MODE(AUTOLAND_MODE);
    else
        DISABLE_FLIGHT_MODE(AUTOLAND_MODE);
}

class AutolandTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&sim, 0, sizeof(sim));
        memset(&profile, 0, sizeof(profile));
        flightModeFlags = 0;
        armingFlags = 0;
        armingDisableFlags = 0;
        debugMode = DEBUG_AUTOLAND;

        profile.althold.alt_p_gain = 10;
        profile.althold.alt_i_gain = 5;
        profile.althold.alt_d_gain = 15;
        profile.althold.max_climb_rate = 200;
        profile.althold.stick_deadband = 100;
        profile.althold.hover_collective = 350;
        profile.althold.exit_time = 30;

        profile.autoland.settle_time = 50;
        profile.autoland.descent_rate = 80;
        profile.autoland.final_rate = 25;
        profile.autoland.slow_height = 150;
        profile.autoland.ge_height = 40;
        profile.autoland.ge_push = 30;
        profile.autoland.ge_push_rate = 150;
        profile.autoland.ground_collective = -100;
        profile.autoland.motor_off_delay = 20;
        profile.autoland.disarm_delay = 120;
        profile.autoland.takeoff_altitude = 150;
        profile.autoland.takeoff_climb_rate = 50;
        profile.autoland.takeoff_ramp_time = 15;
        profile.autoland.takeoff_max_extra = 150;

        altHoldInitProfile(&profile);
        autolandInitProfile(&profile);

        sim.lidarAlive = true;
        sim.lidarMin = 0.04f;
        sim.gpsReady = true;
        sim.stick = TRUE_HOVER / 1000.0f;   // pilot hovers by hand
        sim.timeUs = 1000000;

        // Module state is static: one disarmed cycle resets it, as in flight
        simRun(0.01f);
        sim.disarmed = false;

        ENABLE_ARMING_FLAG(ARMED);
        simRun(0.1f);   // disarmed -> armed handling in the autoland
    }

    // Hover at height h in altitude hold, settled
    void hoverAt(float h)
    {
        sim.h = h;
        sim.v = 0;
        sim.touched = false;    // record the next ground contact
        ENABLE_FLIGHT_MODE(ALTHOLD_MODE);
        simRun(5.0f);
    }
};

TEST_F(AutolandTest, FullSequenceWithLidar)
{
    hoverAt(5.0f);
    DISABLE_FLIGHT_MODE(ALTHOLD_MODE);      // one switch is enough

    autolandSwitch(true);
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_SETTLE, getAutolandState());

    // Settles for at least 5 s, at the same height
    simRun(4.9f);
    EXPECT_EQ(AUTOLAND_STATE_SETTLE, getAutolandState());
    EXPECT_NEAR(5.0f, sim.h, 0.1f);

    EXPECT_GE(simRunUntil(AUTOLAND_STATE_DESCEND, 2.0f), 0);
    simRun(1.0f);
    EXPECT_NEAR(-0.8f, sim.v, 0.15f);

    // Slow below 1.5 m
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_FINAL, 10.0f), 0);
    EXPECT_NEAR(1.5f, sim.h + LIDAR_OFFSET, 0.1f);
    simRun(1.0f);
    EXPECT_NEAR(-0.25f, sim.v, 0.1f);

    EXPECT_GE(simRunUntil(AUTOLAND_STATE_GROUND_EFFECT, 10.0f), 0);
    EXPECT_NEAR(0.4f, sim.h + LIDAR_OFFSET, 0.05f);

    EXPECT_GE(simRunUntil(AUTOLAND_STATE_TOUCHDOWN, 5.0f), 0);
    EXPECT_TRUE(sim.touched);
    EXPECT_LT(sim.touchdownSpeed, 0.5f);
    EXPECT_EQ(0.0f, sim.h);

    EXPECT_GE(simRunUntil(AUTOLAND_STATE_LANDED, 2.0f), 0);
    simRun(0.1f);
    EXPECT_FLOAT_EQ(-100.0f, sim.coll);
    EXPECT_FALSE(autolandIsMotorCut());

    // Motor off after 2 s, disarm 12 s later
    EXPECT_NEAR(2.0f, simRunUntil(AUTOLAND_STATE_MOTOR_OFF, 3.0f), 0.1f);
    EXPECT_TRUE(autolandIsMotorCut());
    simRun(11.9f);
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FLOAT_EQ(-100.0f, sim.coll);
    simRun(0.2f);
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_EQ(DISARM_REASON_AUTOLAND, sim.disarmReason);
    EXPECT_TRUE(armingDisableFlags & ARMING_DISABLED_ARM_SWITCH);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
    EXPECT_FALSE(autolandIsMotorCut());
}

TEST_F(AutolandTest, LidarInvalidNearGround)
{
    sim.lidarMin = 0.25f;           // no reading below ~20 cm (grass)
    profile.autoland.ge_height = 15;   // ground effect height below the LIDAR minimum
    autolandInitProfile(&profile);

    hoverAt(2.0f);
    autolandSwitch(true);

    // The AGL drops out at 20 cm, just above the ground effect height: push
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_GROUND_EFFECT, 30.0f), 0);
    EXPECT_LT(sim.h, 0.3f);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_MOTOR_OFF, 10.0f), 0);
    EXPECT_LT(sim.touchdownSpeed, 0.5f);
}

TEST_F(AutolandTest, NoGroundEffectPhaseStillLands)
{
    profile.autoland.ge_height = 0;     // detector B alone
    autolandInitProfile(&profile);

    hoverAt(2.0f);
    autolandSwitch(true);

    EXPECT_GE(simRunUntil(AUTOLAND_STATE_TOUCHDOWN, 40.0f), 0);
    EXPECT_EQ(0.0f, sim.h);
    EXPECT_LT(sim.touchdownSpeed, 0.5f);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_MOTOR_OFF, 5.0f), 0);
}

TEST_F(AutolandTest, ObstacleUnderLidarRevertsPush)
{
    hoverAt(2.0f);
    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_FINAL, 10.0f), 0);

    // A box under the LIDAR at 1.5 m: it reads 30 cm
    sim.obstacleAgl = 0.30f;
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_GROUND_EFFECT, 2.0f), 0);
    const float hPush = sim.h;

    // Sinks fast, so the push is abandoned before reaching the ground
    bool reverted = false;
    for (int i = 0; i < 3000 && !reverted; i++) {
        simStep();
        reverted = getAutolandState() == AUTOLAND_STATE_FINAL;
    }
    EXPECT_TRUE(reverted);
    EXPECT_GT(sim.h, 0.3f);
    EXPECT_LT(sim.h, hPush);

    // Obstacle gone: lands normally
    sim.obstacleAgl = 0;
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_MOTOR_OFF, 30.0f), 0);
    EXPECT_LT(sim.touchdownSpeed, 0.5f);
}

TEST_F(AutolandTest, RefusedWithoutGpsUntilToggled)
{
    hoverAt(3.0f);
    sim.gpsReady = false;

    autolandSwitch(true);
    simRun(0.1f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
    EXPECT_EQ(1, sim.beeps);

    // Becoming available with the switch still on does not start a landing
    sim.gpsReady = true;
    simRun(1.0f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());

    autolandSwitch(false);
    simRun(0.1f);
    autolandSwitch(true);
    simRun(0.1f);
    EXPECT_EQ(AUTOLAND_STATE_SETTLE, getAutolandState());
}

TEST_F(AutolandTest, RefusedWithoutLidar)
{
    hoverAt(3.0f);
    sim.lidarAlive = false;
    autolandSwitch(true);
    simRun(0.1f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
}

TEST_F(AutolandTest, SwitchOffInDescentHoldsAltitude)
{
    hoverAt(5.0f);      // ALTHOLD on
    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_DESCEND, 10.0f), 0);
    simRun(1.5f);

    autolandSwitch(false);
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
    // Stops like after a stick release: the target is latched at most the
    // 30 cm leash below, and held there
    const float hOff = sim.h;
    simRun(3.0f);
    EXPECT_GT(sim.h, hOff - 0.35f);
    EXPECT_LT(sim.h, hOff);
    EXPECT_NEAR(0.0f, sim.v, 0.05f);
    const float hHeld = sim.h;
    simRun(3.0f);
    EXPECT_NEAR(hHeld, sim.h, 0.03f);

    // Switching on again starts over with SETTLE
    autolandSwitch(true);
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_SETTLE, getAutolandState());
}

TEST_F(AutolandTest, SwitchOffWithoutAltholdHandsBack)
{
    hoverAt(5.0f);
    DISABLE_FLIGHT_MODE(ALTHOLD_MODE);
    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_DESCEND, 10.0f), 0);
    simRun(1.0f);

    autolandSwitch(false);
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
    // Exit fade back to the stick (which is at the true hover): no step
    float minColl = 1000, maxColl = -1000;
    for (int i = 0; i < 4000; i++) {
        simStep();
        minColl = fminf(minColl, sim.coll);
        maxColl = fmaxf(maxColl, sim.coll);
    }
    EXPECT_NEAR(TRUE_HOVER, sim.coll, 1.0f);
    EXPECT_GT(minColl, 250.0f);
    EXPECT_LT(maxColl, 500.0f);
}

TEST_F(AutolandTest, SwitchIgnoredAfterTouchdown)
{
    hoverAt(1.0f);
    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_LANDED, 30.0f), 0);

    autolandSwitch(false);
    simRun(1.0f);
    EXPECT_EQ(AUTOLAND_STATE_LANDED, getAutolandState());
    EXPECT_EQ(0.0f, sim.h);

    EXPECT_GE(simRunUntil(AUTOLAND_STATE_MOTOR_OFF, 3.0f), 0);
    simRun(12.5f);
    EXPECT_FALSE(ARMING_FLAG(ARMED));
}

TEST_F(AutolandTest, RescueYieldsAndResumes)
{
    hoverAt(5.0f);
    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_DESCEND, 10.0f), 0);

    sim.rescue = true;
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
    simRun(1.0f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());

    sim.rescue = false;
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_SETTLE, getAutolandState());
}

TEST_F(AutolandTest, SwitchOnAtArmingNeedsToggle)
{
    DISABLE_ARMING_FLAG(ARMED);
    autolandSwitch(true);
    simRun(0.1f);
    sim.h = 3.0f;
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(ALTHOLD_MODE);
    simRun(1.0f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
}

TEST_F(AutolandTest, StickCannotSpeedUpFinal)
{
    hoverAt(1.2f);
    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_FINAL, 10.0f), 0);
    sim.stick = -1.0f;      // full down
    simRun(1.0f);
    EXPECT_GT(sim.v, -0.35f);
}

TEST_F(AutolandTest, StickUpPausesDescent)
{
    hoverAt(5.0f);
    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_DESCEND, 10.0f), 0);
    sim.stick = 0.9f;       // well up
    simRun(2.0f);
    EXPECT_GT(sim.v, 0.0f);
    EXPECT_EQ(AUTOLAND_STATE_DESCEND, getAutolandState());
}

// ---- Auto takeoff ----

class TakeoffTest : public AutolandTest {
protected:
    void SetUp() override
    {
        AutolandTest::SetUp();
        // Back on the ground, freshly armed, stick low, ALTHOLD on
        DISABLE_ARMING_FLAG(ARMED);
        sim.h = 0;
        sim.v = 0;
        sim.stick = -0.2f;
        simRun(0.01f);
        ENABLE_ARMING_FLAG(ARMED);
        ENABLE_FLIGHT_MODE(ALTHOLD_MODE);
        simRun(1.0f);
    }

    void pushStick(void)
    {
        sim.stick = 0.6f;
        simRun(0.3f);
        sim.stick = 0.1f;
        simRun(0.01f);
    }
};

TEST_F(TakeoffTest, IdleCapsCollectiveOnTheGround)
{
    EXPECT_EQ(AUTOLAND_STATE_GROUND_IDLE, getAutolandState());
    sim.stick = 0.6f;               // would lift it without the cap
    simRun(0.2f);
    EXPECT_LE(sim.coll, 175.0f);
    EXPECT_EQ(0.0f, sim.h);
    EXPECT_EQ(AUTOLAND_STATE_GROUND_IDLE, getAutolandState());
}

TEST_F(TakeoffTest, FullTakeoffToTakeoffAltitude)
{
    pushStick();
    EXPECT_EQ(AUTOLAND_STATE_TAKEOFF_RAMP, getAutolandState());

    const float tLiftoff = simRunUntil(AUTOLAND_STATE_TAKEOFF_CLIMB, 6.0f);
    EXPECT_GE(tLiftoff, 0.5f);
    EXPECT_LT(sim.h, 0.15f);        // detected right after leaving the ground

    EXPECT_GE(simRunUntil(AUTOLAND_STATE_OFF, 10.0f), 0);
    EXPECT_NEAR(1.5f, sim.h, 0.15f);

    // Altitude hold holds it there, from the liftoff collective
    simRun(5.0f);
    EXPECT_NEAR(1.5f, sim.h, 0.3f);
    EXPECT_NEAR(0.0f, sim.v, 0.05f);
    EXPECT_NEAR(TRUE_HOVER, sim.coll, 15.0f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
}

TEST_F(TakeoffTest, StickHighAtArmingDoesNotLaunch)
{
    DISABLE_ARMING_FLAG(ARMED);
    simRun(0.01f);
    sim.stick = 0.6f;
    ENABLE_ARMING_FLAG(ARMED);
    simRun(1.0f);
    sim.stick = 0.1f;
    simRun(2.0f);
    EXPECT_EQ(AUTOLAND_STATE_GROUND_IDLE, getAutolandState());
    EXPECT_EQ(0.0f, sim.h);
}

TEST_F(TakeoffTest, NotSpooledUpRefuses)
{
    sim.notSpooled = true;
    const int beeps = sim.beeps;
    pushStick();
    EXPECT_EQ(AUTOLAND_STATE_GROUND_IDLE, getAutolandState());
    EXPECT_EQ(beeps + 1, sim.beeps);
}

TEST_F(TakeoffTest, AbortsWithoutLiftoff)
{
    sim.tiedDown = true;
    pushStick();
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_GROUND_IDLE, 8.0f), 0);
    EXPECT_GE(sim.coll, 400.0f);    // reached hover + max extra
    simRun(1.0f);
    EXPECT_LE(sim.coll, 175.0f);    // and comes back down
    EXPECT_EQ(0.0f, sim.h);
    EXPECT_TRUE(autolandIsActive());

    // A new push tries again
    sim.tiedDown = false;
    pushStick();
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_TAKEOFF_CLIMB, 6.0f), 0);
}

TEST_F(TakeoffTest, StickDownAborts)
{
    pushStick();
    simRun(0.5f);
    sim.stick = -1.0f;
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_GROUND_IDLE, getAutolandState());
}

TEST_F(TakeoffTest, AltholdOffDuringRampHandsBack)
{
    pushStick();
    simRun(0.5f);
    DISABLE_FLIGHT_MODE(ALTHOLD_MODE);
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());
    EXPECT_NEAR(100.0f, sim.coll, 1.0f);    // the pilot collective
}

TEST_F(TakeoffTest, ManualLiftoffThenAltholdHolds)
{
    DISABLE_FLIGHT_MODE(ALTHOLD_MODE);
    simRun(0.1f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());

    // Pilot lifts off by hand and climbs
    sim.stick = 0.5f;
    simRun(1.5f);
    sim.stick = TRUE_HOVER / 1000.0f;
    simRun(0.5f);
    EXPECT_GT(sim.h, 0.5f);

    ENABLE_FLIGHT_MODE(ALTHOLD_MODE);
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_OFF, getAutolandState());   // no ground idle in the air
    const float h = sim.h;
    simRun(4.0f);
    EXPECT_NEAR(h, sim.h, 0.4f);
    EXPECT_GT(sim.coll, 300.0f);
}

TEST_F(TakeoffTest, AutolandDuringClimbLands)
{
    pushStick();
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_TAKEOFF_CLIMB, 6.0f), 0);
    simRun(1.0f);
    autolandSwitch(true);
    simRun(0.01f);
    EXPECT_EQ(AUTOLAND_STATE_SETTLE, getAutolandState());
}

TEST_F(TakeoffTest, TakeoffHoverLandCycle)
{
    pushStick();
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_OFF, 15.0f), 0);
    simRun(3.0f);
    sim.touched = false;

    autolandSwitch(true);
    EXPECT_GE(simRunUntil(AUTOLAND_STATE_MOTOR_OFF, 40.0f), 0);
    EXPECT_TRUE(sim.touched);
    EXPECT_LT(sim.touchdownSpeed, 0.5f);
    simRun(12.5f);
    EXPECT_FALSE(ARMING_FLAG(ARMED));
}
