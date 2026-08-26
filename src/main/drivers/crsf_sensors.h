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

#include "common/time.h"

#define CRSF_SENSORS_TIMEOUT_MS_DEFAULT 2000
#define CRSF_SENSORS_TIMEOUT_MS_MIN 500
#define CRSF_SENSORS_TIMEOUT_MS_MAX 10000

typedef struct crsfSensorsGpsData_s {
    int32_t latitude;
    int32_t longitude;
    uint16_t groundspeedCmS;
    uint16_t headingDeg10;
    int32_t altitudeCm;
    uint8_t satellites;
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsGpsData_t;

typedef struct crsfSensorsBatteryData_s {
    uint32_t voltageMv;
    uint32_t currentMa;
    uint32_t capacityMah;
    uint8_t remainingPct;
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsBatteryData_t;

typedef struct crsfSensorsBaroData_s {
    int32_t altitudeCm;
    int16_t verticalSpeedCmS;
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsBaroData_t;

void crsfSensorsInit(void);
void crsfSensorsUpdate(timeUs_t currentTimeUs);
bool crsfSensorsIsEnabled(void);

bool crsfSensorsGetGpsData(crsfSensorsGpsData_t *data);
bool crsfSensorsHasGpsData(void);

bool crsfSensorsGetBatteryData(crsfSensorsBatteryData_t *data);
bool crsfSensorsHasBatteryData(void);

bool crsfSensorsGetBaroData(crsfSensorsBaroData_t *data);
bool crsfSensorsHasBaroData(void);

void crsfSensorsSetBaroUse(bool enabled);
bool crsfSensorsGetBaroUse(void);
