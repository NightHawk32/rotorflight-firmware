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

#include <stdint.h>

#include "pg/pg.h"

enum {
    EXTERNAL_TEMPERATURE_SOURCE_NONE = 0,
    EXTERNAL_TEMPERATURE_SOURCE_FBUS,
    EXTERNAL_TEMPERATURE_SOURCE_ADC,
};

enum {
    EXTERNAL_TEMPERATURE_NTC_TYPE_10K = 0,
    EXTERNAL_TEMPERATURE_NTC_TYPE_100K,
};

typedef struct externalTemperatureConfig_s {
    uint8_t source;
    uint16_t fbusAppId;
    uint8_t adcNtcType;
    uint16_t adcResistor;
    uint16_t adcBeta;
} externalTemperatureConfig_t;

PG_DECLARE(externalTemperatureConfig_t, externalTemperatureConfig);
