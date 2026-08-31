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

// Angle correction in centidegrees, added to target angle in leveling.c
// (same convention as gpsRescueAngle[])
extern int32_t posHoldAngle[2]; // [AI_ROLL, AI_PITCH]

bool posHoldIsActive(void);
void posHoldUpdate(void);
void posHoldInitProfile(const pidProfile_t *pidProfile);
