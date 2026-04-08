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

#include "pg/external_temperature.h"

typedef struct externalTemperatureState_s {
    uint8_t source;
    bool configured;
    bool available;
    uint16_t adcValue;
    uint16_t fbusAppId;
    int temperature;
} externalTemperatureState_t;

bool externalTemperatureGet(externalTemperatureState_t *state);
int externalTemperatureGetValue(void);
bool externalTemperatureIsActive(void);
