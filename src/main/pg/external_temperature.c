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

#include "types.h"
#include "platform.h"

#include "drivers/fbus_sensor.h"

#include "pg/pg_ids.h"
#include "pg/external_temperature.h"

PG_REGISTER_WITH_RESET_FN(externalTemperatureConfig_t, externalTemperatureConfig, PG_EXTERNAL_TEMPERATURE_CONFIG, 0);

void pgResetFn_externalTemperatureConfig(externalTemperatureConfig_t *config)
{
    config->source = EXTERNAL_TEMPERATURE_SOURCE_NONE;
    config->fbusAppId = FBUS_TEMPERATURE1_BASE;
    config->adcNtcType = EXTERNAL_TEMPERATURE_NTC_TYPE_10K;
    config->adcResistor = 10000;
    config->adcBeta = 3950;
}

void pgResetTemplate_externalTemperatureConfig(externalTemperatureConfig_t *config)
{
    pgResetFn_externalTemperatureConfig(config);
}
