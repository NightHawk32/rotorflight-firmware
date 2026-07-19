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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/time.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "flight/governor.h"

#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "sensors/gyro.h"

#include "airborne.h"

#define FILTER_CUTOFF                       5.0f
#define GYRO_FILTER_CUTOFF                 10.0f

#define PEAK_UP_CUTOFF                     20.0f
#define PEAK_DN_CUTOFF                      0.5f

#define LIFTOFF_COS_ANGLE_THRESHOLD        0.80f
#define LANDING_COS_ANGLE_THRESHOLD        0.90f

#define LIFTOFF_MIN_TIME_MS                150

typedef enum {
    AIRBORNE_MODE_CONSERVATIVE = 0,
    AIRBORNE_MODE_STICK_RESPONSE,
} airborneMode_e;

typedef enum {
    AIRBORNE_STATE_INIT = 0,
    AIRBORNE_STATE_LANDED,
    AIRBORNE_STATE_AIRBORNE,
} airborneState_e;

typedef struct
{
    airborneState_e state;
    airborneMode_e  mode;

    float liftoffThreshold[4];
    float landingThreshold[4];
    float gyroThreshold;

    pt1Filter_t  filter[4];
    peakFilter_t peakDeflection[4];

    pt1Filter_t  gyroFilter[2];
    peakFilter_t peakGyroRate[2];

    timeMs_t    liftoffEntryTime;

} airborneData_t;

static FAST_DATA_ZERO_INIT airborneData_t airborne;


INIT_CODE void airborneInit(void)
{
    airborne.state = AIRBORNE_STATE_LANDED;
    airborne.mode  = rcControlsConfig()->airborne_mode;
    airborne.gyroThreshold = rcControlsConfig()->airborne_gyro_threshold;

    for (int axis = 0; axis < 4; axis++) {
        pt1FilterInit(&airborne.filter[axis], FILTER_CUTOFF, pidGetPidFrequency());
        peakFilterInit(&airborne.peakDeflection[axis], PEAK_UP_CUTOFF, PEAK_DN_CUTOFF, pidGetPidFrequency());
        airborne.liftoffThreshold[axis] = rcControlsConfig()->rc_threshold[axis] / 1000.0f;
        airborne.landingThreshold[axis] = rcControlsConfig()->rc_threshold[axis] / 1500.0f;
    }

    for (int axis = 0; axis < 2; axis++) {
        pt1FilterInit(&airborne.gyroFilter[axis], GYRO_FILTER_CUTOFF, pidGetPidFrequency());
        peakFilterInit(&airborne.peakGyroRate[axis], PEAK_UP_CUTOFF, PEAK_DN_CUTOFF, pidGetPidFrequency());
    }
}

static bool isOverThreshold(const float *threshold)
{
    return (
        peakFilterOutput(&airborne.peakDeflection[FD_ROLL]) > threshold[FD_ROLL] ||
        peakFilterOutput(&airborne.peakDeflection[FD_PITCH]) > threshold[FD_PITCH] ||
        peakFilterOutput(&airborne.peakDeflection[FD_YAW]) > threshold[FD_YAW] ||
        peakFilterOutput(&airborne.peakDeflection[FD_COLL]) > threshold[FD_COLL]
    );
}

static bool liftoff(void)
{
    return (
        ARMING_FLAG(ARMED) &&
        isSpooledUp() &&
        (
            isOverThreshold(airborne.liftoffThreshold) ||
            getCosTiltAngle() < LIFTOFF_COS_ANGLE_THRESHOLD ||
            FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)
        )
    );
}

static bool touchdown(void)
{
    return !(
        ARMING_FLAG(ARMED) &&
        isSpooledUp() &&
        (
            isOverThreshold(airborne.landingThreshold) ||
            getCosTiltAngle() < LANDING_COS_ANGLE_THRESHOLD ||
            FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)
        )
    );
}

static void updateStickDeflection(const float rc[4])
{
    for (int axis = 0; axis < 4; axis++) {
        float stick = pt1FilterApply(&airborne.filter[axis], rc[axis]);
        if (axis == FD_COLL)
            stick = fmaxf(stick, 0);
        peakFilterApply(&airborne.peakDeflection[axis], fabsf(stick));
    }
}

static void updateGyroRate(void)
{
    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        float rate = pt1FilterApply(&airborne.gyroFilter[axis], gyro.gyroADCf[axis]);
        peakFilterApply(&airborne.peakGyroRate[axis], fabsf(rate));
    }
}

// STICK_RESPONSE: liftoff when the heli demonstrably follows cyclic stick input
static bool liftoffByStickResponse(void)
{
    // Require stick deflection AND corresponding gyro response on the same cyclic axis
    bool responding = false;
    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        if (peakFilterOutput(&airborne.peakDeflection[axis]) > airborne.liftoffThreshold[axis] &&
            peakFilterOutput(&airborne.peakGyroRate[axis])   > airborne.gyroThreshold) {
            responding = true;
            break;
        }
    }

    return (
        ARMING_FLAG(ARMED) &&
        isSpooledUp() &&
        (
            responding ||
            getCosTiltAngle() < LIFTOFF_COS_ANGLE_THRESHOLD ||
            FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)
        )
    );
}

void airborneUpdate(const float rc[4])
{
    updateStickDeflection(rc);
    updateGyroRate();

    const bool liftoffCondition = (airborne.mode == AIRBORNE_MODE_STICK_RESPONSE)
        ? liftoffByStickResponse()
        : liftoff();
    const bool touchdownCondition = touchdown();

    switch (airborne.state) {
        case AIRBORNE_STATE_INIT:
            break;
        case AIRBORNE_STATE_LANDED:
            if (liftoffCondition) {
                if (airborne.liftoffEntryTime == 0)
                    airborne.liftoffEntryTime = millis();
                if (cmp32(millis(), airborne.liftoffEntryTime) >= LIFTOFF_MIN_TIME_MS)
                    airborne.state = AIRBORNE_STATE_AIRBORNE;
            } else {
                airborne.liftoffEntryTime = 0;
            }
            break;
        case AIRBORNE_STATE_AIRBORNE:
            if (touchdownCondition) {
                airborne.liftoffEntryTime = 0;
                airborne.state = AIRBORNE_STATE_LANDED;
            }
            break;
    }

    DEBUG(AIRBORNE, 0, peakFilterOutput(&airborne.peakDeflection[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, peakFilterOutput(&airborne.peakDeflection[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, peakFilterOutput(&airborne.peakGyroRate[FD_ROLL]) * 10);
    DEBUG(AIRBORNE, 3, peakFilterOutput(&airborne.peakGyroRate[FD_PITCH]) * 10);
    DEBUG(AIRBORNE, 4, getCosTiltAngle() * 1000);
    DEBUG(AIRBORNE, 5, isSpooledUp());
    DEBUG(AIRBORNE, 6, (liftoffCondition ? 1 : 0) | (touchdownCondition ? 2 : 0));
    DEBUG(AIRBORNE, 7, airborne.state);
}

bool isAirborne(void)
{
    return (airborne.state == AIRBORNE_STATE_AIRBORNE);
}

bool isHandsOn(void)
{
    return isOverThreshold(airborne.liftoffThreshold);
}