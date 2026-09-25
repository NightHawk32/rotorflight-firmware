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
 * Closed-loop tests for the hard deck: a simple point-mass helicopter with
 * roll attitude and vertical dynamics is flown against harddeck.c.
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
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/harddeck.h"
#include "pg/pid.h"
#include "pg/accel.h"
}

#define SIM_DT      0.001f      // 1kHz PID loop
#define GRAVITY     9.81f

// ---- Simulated world ----

static struct {
    uint32_t    timeUs;
    float       alt;            // m
    float       vario;          // m/s
    float       roll;           // deg, 180 = inverted
    float       altStdDev;      // m
    bool        altValid;
    float       stick[4];       // pilot deflection -1..1
    float       pilotRate[3];   // pilot rate setpoint, deg/s
    float       pilotColl;      // pilot collective, -1000..1000
    float       minAlt;
} sim;

static pidProfile_t profile;

extern "C" {
uint8_t armingFlags;
uint16_t flightModeFlags;
uint8_t debugMode;
int32_t debug[DEBUG_VALUE_COUNT];
attitudeEulerAngles_t attitude;
accelerometerConfig_t accelerometerConfig_System;

uint32_t millis(void) { return sim.timeUs / 1000; }
float pidGetDT(void) { return SIM_DT; }
float getRcDeflection(int axis) { return sim.stick[axis]; }

float getCosTiltAngle(void) { return cosf(DEGREES_TO_RADIANS(sim.roll)); }

bool getAltitudeEstimate(float *altitudeM, float *varioMs, float *stdDevM)
{
    *altitudeM = sim.alt;
    *varioMs = sim.vario;
    *stdDevM = sim.altStdDev;
    return sim.altValid;
}
float getBaroBias(void) { return 0; }
float getBaroDisturbance(void) { return 0; }

bool isAGLAltitudeValid(void) { return false; }
float getAGLAltitude(void) { return 0; }

bool isPositionXYValid(void) { return false; }
float getPositionXCm(void) { return 0; }
float getPositionYCm(void) { return 0; }
float getVelocityXCms(void) { return 0; }
float getVelocityYCms(void) { return 0; }
}

static float wrapDeg(float a)
{
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
    return a;
}

// One PID-loop step: controller, then physics
static void simStep(void)
{
    attitude.values.roll = lrintf(wrapDeg(sim.roll) * 10);
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;

    hardDeckUpdate();

    const float rollRate = hardDeckApply(FD_ROLL, sim.pilotRate[FD_ROLL]);
    hardDeckApply(FD_PITCH, sim.pilotRate[FD_PITCH]);
    hardDeckApply(FD_YAW, sim.pilotRate[FD_YAW]);
    const float coll = hardDeckApply(FD_COLL, sim.pilotColl);

    // Heli: rate follows setpoint, thrust ∝ collective, 350 = hover
    sim.roll = wrapDeg(sim.roll + rollRate * SIM_DT);
    const float accel = GRAVITY * (coll / 350.0f) * getCosTiltAngle() - GRAVITY - 0.2f * sim.vario;
    sim.vario += accel * SIM_DT;
    sim.alt += sim.vario * SIM_DT;

    sim.timeUs += 1000;
    sim.minAlt = fminf(sim.minAlt, sim.alt);
}

static void simRun(float seconds)
{
    for (int i = 0; i < lrintf(seconds / SIM_DT); i++) {
        simStep();
    }
}

// Pilot collective that keeps the given vertical speed in the current attitude
static float pilotCollFor(float vario)
{
    return 350.0f / getCosTiltAngle() * (1.0f + 0.2f * vario / GRAVITY);
}

class HardDeckTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        memset(&sim, 0, sizeof(sim));
        memset(&profile, 0, sizeof(profile));
        memset(&accelerometerConfig_System, 0, sizeof(accelerometerConfig_System));

        sim.timeUs = 1000000;
        sim.altValid = true;
        sim.altStdDev = 0.3f;
        sim.minAlt = 1e9f;

        profile.rescue.flip_gain = 200;
        profile.rescue.level_gain = 100;
        profile.rescue.pull_up_time = 3;
        profile.rescue.flip_time = 20;
        profile.rescue.exit_time = 5;
        profile.rescue.pull_up_collective = 650;
        profile.rescue.hover_collective = 350;
        profile.rescue.alt_p_gain = 20;
        profile.rescue.alt_i_gain = 20;
        profile.rescue.alt_d_gain = 10;
        profile.rescue.max_collective = 500;
        profile.rescue.max_setpoint_rate = 300;
        profile.rescue.max_setpoint_accel = 3000;

        profile.althold.max_climb_rate = 200;
        profile.althold.stick_deadband = 100;
        profile.poshold.stick_deadband = 100;

        profile.harddeck.altitude = 100;
        profile.harddeck.arm_margin = 20;
        profile.harddeck.recovery_margin = 30;
        profile.harddeck.release_altitude = 0;
        profile.harddeck.recovery_accel = 50;
        profile.harddeck.reaction_time = 300;
        profile.harddeck.sigma_factor = 20;
        profile.harddeck.use_agl = 1;

        hardDeckInitProfile(&profile);

        armingFlags = 0;
        flightModeFlags = 0;
        simStep();      // disarmed step resets the state machine

        armingFlags = ARMED;
        flightModeFlags = HARDDECK_MODE;
    }

    // Climb from the ground through the arm altitude and settle
    void armDeckAt(float alt)
    {
        sim.alt = alt;
        sim.roll = 0;
        sim.vario = 0;
        sim.pilotColl = pilotCollFor(0);
        simRun(1.5f);
        sim.minAlt = sim.alt;
        ASSERT_EQ(HARDDECK_STATE_WATCH, getHardDeckState());
    }
};

TEST_F(HardDeckTest, WaitsBelowDeckAndArmsAboveIt)
{
    // On the ground with the switch on: must not interfere
    sim.alt = 0;
    sim.pilotColl = 0;
    sim.alt = 0;
    for (int i = 0; i < 2000; i++) {
        simStep();
        sim.alt = 0;
        sim.vario = 0;
    }
    EXPECT_EQ(HARDDECK_STATE_WAIT, getHardDeckState());
    EXPECT_FALSE(hardDeckIsIntervening());

    // Just above deck + margin, but within 2 sigma: still waiting
    sim.alt = 12.3f;
    sim.pilotColl = pilotCollFor(0);
    simRun(2.0f);
    EXPECT_EQ(HARDDECK_STATE_WAIT, getHardDeckState());

    // Clearly above: arms after the delay
    sim.alt = 15.0f;
    simRun(0.5f);
    EXPECT_EQ(HARDDECK_STATE_WAIT, getHardDeckState());
    simRun(1.0f);
    EXPECT_EQ(HARDDECK_STATE_WATCH, getHardDeckState());
}

TEST_F(HardDeckTest, PilotHasFullControlAboveDeck)
{
    armDeckAt(40.0f);

    sim.pilotRate[FD_ROLL] = 200;
    sim.pilotColl = 123;
    simStep();

    EXPECT_FLOAT_EQ(200, hardDeckApply(FD_ROLL, 200));
    EXPECT_FLOAT_EQ(123, hardDeckApply(FD_COLL, 123));
    EXPECT_FALSE(hardDeckIsIntervening());
}

TEST_F(HardDeckTest, InvertedDiveRecoversUprightAboveDeck)
{
    armDeckAt(40.0f);

    // Pilot rolls inverted and descends at 8 m/s
    sim.roll = 180;
    sim.vario = -8.0f;
    sim.pilotColl = pilotCollFor(-8.0f);

    bool sawFlip = false;
    for (int i = 0; i < 15000; i++) {
        simStep();
        sawFlip |= (getHardDeckState() == HARDDECK_STATE_FLIP);
    }

    EXPECT_TRUE(sawFlip);
    EXPECT_EQ(HARDDECK_STATE_HOLD, getHardDeckState());
    EXPECT_GT(sim.minAlt, 11.0f);                       // never below the deck, with margin
    EXPECT_GT(getCosTiltAngle(), 0.99f);                // upright
    EXPECT_NEAR(13.0f, sim.alt, 1.0f);                  // holding deck + margin
    EXPECT_NEAR(0.0f, sim.vario, 0.3f);
}

TEST_F(HardDeckTest, KnifeEdgeDiveRecovers)
{
    armDeckAt(50.0f);

    sim.roll = 90;
    sim.vario = -10.0f;
    sim.pilotColl = 0;

    simRun(15.0f);

    EXPECT_GT(sim.minAlt, 10.0f);
    EXPECT_EQ(HARDDECK_STATE_HOLD, getHardDeckState());
    EXPECT_GT(getCosTiltAngle(), 0.99f);
}

TEST_F(HardDeckTest, FasterDescentTriggersHigher)
{
    float triggerAlt[2];
    const float descent[2] = { 3.0f, 12.0f };

    for (int n = 0; n < 2; n++) {
        SetUp();
        armDeckAt(80.0f);
        sim.vario = -descent[n];
        sim.pilotColl = pilotCollFor(-descent[n]);
        while (!hardDeckIsIntervening() && sim.alt > 0) {
            simStep();
        }
        triggerAlt[n] = sim.alt;
    }

    EXPECT_GT(triggerAlt[0], 10.0f);
    EXPECT_GT(triggerAlt[1], triggerAlt[0] + 5.0f);
}

TEST_F(HardDeckTest, LargerUncertaintyTriggersHigher)
{
    float triggerAlt[2];
    const float stdDev[2] = { 0.2f, 2.0f };

    for (int n = 0; n < 2; n++) {
        SetUp();
        armDeckAt(40.0f);
        sim.altStdDev = stdDev[n];
        sim.vario = -2.0f;
        sim.pilotColl = pilotCollFor(-2.0f);
        while (!hardDeckIsIntervening() && sim.alt > 0) {
            simStep();
        }
        triggerAlt[n] = sim.alt;
    }

    EXPECT_NEAR(triggerAlt[1] - triggerAlt[0], 2.0f * (2.0f - 0.2f), 0.2f);
}

TEST_F(HardDeckTest, HoldCannotBeFlownBelowDeck)
{
    armDeckAt(20.0f);
    sim.vario = -3.0f;
    sim.pilotColl = pilotCollFor(-3.0f);
    simRun(10.0f);
    ASSERT_EQ(HARDDECK_STATE_HOLD, getHardDeckState());

    // Full down collective for a while
    sim.stick[FD_COLL] = -1.0f;
    simRun(10.0f);
    EXPECT_EQ(HARDDECK_STATE_HOLD, getHardDeckState());
    EXPECT_GT(sim.minAlt, 10.0f);
    EXPECT_NEAR(13.0f, sim.alt, 1.0f);

    // Up collective raises the hold
    sim.stick[FD_COLL] = 1.0f;
    simRun(3.0f);
    sim.stick[FD_COLL] = 0.0f;
    simRun(5.0f);
    EXPECT_GT(sim.alt, 16.0f);
}

TEST_F(HardDeckTest, SwitchOffReturnsControl)
{
    armDeckAt(20.0f);
    sim.vario = -3.0f;
    sim.pilotColl = pilotCollFor(-3.0f);
    simRun(10.0f);
    ASSERT_TRUE(hardDeckIsIntervening());

    flightModeFlags = 0;
    simStep();
    EXPECT_EQ(HARDDECK_STATE_EXIT, getHardDeckState());
    simRun(1.0f);
    EXPECT_EQ(HARDDECK_STATE_OFF, getHardDeckState());
    EXPECT_FLOAT_EQ(77, hardDeckApply(FD_COLL, 77));
}

TEST_F(HardDeckTest, ClimbingToReleaseAltitudeRearms)
{
    profile.harddeck.release_altitude = 60;    // 6 m above deck
    hardDeckInitProfile(&profile);

    armDeckAt(20.0f);
    sim.vario = -3.0f;
    sim.pilotColl = pilotCollFor(-3.0f);
    simRun(10.0f);
    ASSERT_EQ(HARDDECK_STATE_HOLD, getHardDeckState());

    sim.stick[FD_COLL] = 1.0f;
    simRun(6.0f);
    sim.stick[FD_COLL] = 0.0f;
    sim.pilotColl = pilotCollFor(0);
    simRun(2.0f);

    EXPECT_EQ(HARDDECK_STATE_WATCH, getHardDeckState());
    EXPECT_FALSE(hardDeckIsIntervening());
}

TEST_F(HardDeckTest, NoTriggerWithoutValidAltitude)
{
    armDeckAt(20.0f);
    sim.altValid = false;
    sim.vario = -3.0f;
    sim.pilotColl = pilotCollFor(-3.0f);
    simRun(1.0f);
    EXPECT_EQ(HARDDECK_STATE_WATCH, getHardDeckState());
}
