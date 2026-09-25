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

#include "platform.h"

#include "flight/pid.h"

typedef enum {
    HARDDECK_STATE_OFF = 0,     // mode switch off or disarmed
    HARDDECK_STATE_WAIT,        // mode on, waiting to climb above the deck
    HARDDECK_STATE_WATCH,       // deck armed, pilot has full control
    HARDDECK_STATE_PULLUP,      // level to the nearest horizon, thrust away from the ground
    HARDDECK_STATE_FLIP,        // inverted: roll/pitch over to upright
    HARDDECK_STATE_CLIMB,       // upright: climb back above the deck, brake drift
    HARDDECK_STATE_HOLD,        // altitude + position hold above the deck
    HARDDECK_STATE_EXIT,        // blend control back to the pilot
} hardDeckState_e;

int getHardDeckState(void);

// True while the hard deck is overriding the pilot (recovery, hold or exit blend)
bool hardDeckIsIntervening(void);

void hardDeckUpdate(void);
float hardDeckApply(uint8_t axis, float setpoint);
void hardDeckInitProfile(const pidProfile_t *pidProfile);
