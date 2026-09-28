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

// AH_STATUS_* flags, as in DEBUG_ALTHOLD[7]
#define AH_STATUS_ENGAGED       (1 << 0)
#define AH_STATUS_USING_AGL     (1 << 1)
#define AH_STATUS_SOURCE_VALID  (1 << 2)
#define AH_STATUS_STICK         (1 << 3)
#define AH_STATUS_YIELDED       (1 << 4)

// Last controller cycle, for MSP (altitudes in m, output 0..1000)
void altHoldGetStatus(uint8_t *flags, float *targetAlt, float *currentAlt, float *output);

void altHoldUpdate(void);
float altHoldApply(float collective);
void altHoldInitProfile(const pidProfile_t *pidProfile);
