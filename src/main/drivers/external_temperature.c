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

#include <math.h>
#include <string.h>

#include "common/utils.h"

#include "drivers/adc.h"
#include "drivers/external_temperature.h"
#include "drivers/fbus_sensor.h"

#include "pg/external_temperature.h"

#ifdef USE_ADC
static bool externalTemperatureGetFromAdc(externalTemperatureState_t *state)
{
    if (!adcIsEnabled(ADC_TEMP)) {
        return false;
    }

    const externalTemperatureConfig_t *config = externalTemperatureConfig();
    const uint16_t adcValue = adcGetChannel(ADC_TEMP);

    const float normalized = (float)adcValue / 4095.0f;
    const float clamped = fminf(fmaxf(normalized, 0.0001f), 0.9999f);
    const float resistor = (float)config->adcResistor;

    if (resistor <= 0.0f) {
        return false;
    }

    const float ntcResistance = (clamped * resistor) / (1.0f - clamped);
    if (ntcResistance <= 0.0f) {
        return false;
    }

    const float nominalResistance = config->adcNtcType == EXTERNAL_TEMPERATURE_NTC_TYPE_100K ? 100000.0f : 10000.0f;
    const float nominalTemperatureKelvin = 298.15f;
    const float beta = config->adcBeta > 0 ? (float)config->adcBeta : 3950.0f;
    const float temperatureKelvin = 1.0f / ((1.0f / nominalTemperatureKelvin) + (logf(ntcResistance / nominalResistance) / beta));

    state->adcValue = adcValue;
    state->temperature = lrintf(temperatureKelvin - 273.15f);
    state->available = true;
    return true;
}
#endif

#ifdef USE_FBUS_MASTER
static bool externalTemperatureGetFromFbus(externalTemperatureState_t *state)
{
    state->fbusAppId = externalTemperatureConfig()->fbusAppId;
    if (!fbusSensorGetTemperatureByAppId(state->fbusAppId, &state->temperature)) {
        return false;
    }

    state->available = true;
    return true;
}
#endif

bool externalTemperatureGet(externalTemperatureState_t *state)
{
    if (!state) {
        return false;
    }

    memset(state, 0, sizeof(*state));
    state->source = externalTemperatureConfig()->source;
    state->configured = state->source != EXTERNAL_TEMPERATURE_SOURCE_NONE;

    switch (state->source) {
#ifdef USE_FBUS_MASTER
    case EXTERNAL_TEMPERATURE_SOURCE_FBUS:
        return externalTemperatureGetFromFbus(state);
#endif
#ifdef USE_ADC
    case EXTERNAL_TEMPERATURE_SOURCE_ADC:
        return externalTemperatureGetFromAdc(state);
#endif
    default:
        return false;
    }
}

int externalTemperatureGetValue(void)
{
    externalTemperatureState_t state;
    return externalTemperatureGet(&state) ? state.temperature : 0;
}

bool externalTemperatureIsActive(void)
{
    externalTemperatureState_t state;
    if (externalTemperatureGet(&state)) {
        return true;
    }

    return externalTemperatureConfig()->source != EXTERNAL_TEMPERATURE_SOURCE_NONE;
}
