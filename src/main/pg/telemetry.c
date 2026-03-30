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

#include <string.h>

#ifdef USE_TELEMETRY

#include "common/unit.h"

#include "drivers/fbus_sensor.h"

#include "common/maths.h"

#include "pg/pg_ids.h"
#include "pg/telemetry.h"


PG_REGISTER_WITH_RESET_FN(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 6);

void pgResetFn_telemetryConfig(telemetryConfig_t *telemetryConfig)
{
    telemetryConfig->telemetry_inverted = false;
    telemetryConfig->halfDuplex = 1;
    telemetryConfig->pinSwap = 0;
    telemetryConfig->gpsNoFixLatitude = 0;
    telemetryConfig->gpsNoFixLongitude = 0;
    telemetryConfig->frsky_coordinate_format = FRSKY_FORMAT_DMS;
    telemetryConfig->frsky_unit = UNIT_METRIC;
    telemetryConfig->frsky_vfas_precision = 0;
    telemetryConfig->hottAlarmSoundInterval = 5;
    telemetryConfig->report_cell_voltage = false;
    telemetryConfig->flysky_sensors[0] = IBUS_SENSOR_TYPE_TEMPERATURE;
    telemetryConfig->flysky_sensors[1] = IBUS_SENSOR_TYPE_RPM_FLYSKY;
    telemetryConfig->flysky_sensors[2] = IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE;
    telemetryConfig->mavlink_mah_as_heading_divisor = 0;
    telemetryConfig->crsf_telemetry_mode = CRSF_TELEMETRY_MODE_NATIVE;
    telemetryConfig->crsf_telemetry_link_rate = 250;
    telemetryConfig->crsf_telemetry_link_ratio = 8;
    telemetryConfig->externalMotorTempSource = EXTERNAL_MOTOR_TEMP_SOURCE_NONE;
    telemetryConfig->externalMotorTempFbusAppId = FBUS_TEMPERATURE1_BASE;
    telemetryConfig->externalMotorTempAdcMin = 0;
    telemetryConfig->externalMotorTempAdcMax = 4095;
    telemetryConfig->externalMotorTempMin = 0;
    telemetryConfig->externalMotorTempMax = 150;
    memset(telemetryConfig->telemetry_sensors, 0, sizeof(telemetryConfig->telemetry_sensors));
    memset(telemetryConfig->telemetry_interval, 0, sizeof(telemetryConfig->telemetry_interval));

    if (telemetryConfig->externalMotorTempAdcMax <= telemetryConfig->externalMotorTempAdcMin) {
        telemetryConfig->externalMotorTempAdcMax = telemetryConfig->externalMotorTempAdcMin + 1;
    }

    if (telemetryConfig->externalMotorTempMax <= telemetryConfig->externalMotorTempMin) {
        telemetryConfig->externalMotorTempMax = telemetryConfig->externalMotorTempMin + 1;
    }
}

void pgResetTemplate_telemetryConfig(telemetryConfig_t *telemetryConfig)
{
    pgResetFn_telemetryConfig(telemetryConfig);
}

#endif
