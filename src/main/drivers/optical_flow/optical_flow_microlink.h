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

#include "drivers/optical_flow/optical_flow.h"

// MicroLink protocol types
typedef enum {
    MICROLINK_PROTOCOL_MICROLINK = 0,
    MICROLINK_PROTOCOL_MSP = 1,
    MICROLINK_PROTOCOL_MAV_APM = 2,
    MICROLINK_PROTOCOL_MAV_PX4 = 3
} microlinkProtocol_e;

// MicroLink sensor orientation
typedef enum {
    MICROLINK_ORIENTATION_0_DEG = 0,
    MICROLINK_ORIENTATION_90_DEG = 1,
    MICROLINK_ORIENTATION_180_DEG = 2,
    MICROLINK_ORIENTATION_270_DEG = 3
} microlinkOrientation_e;

// MicroLink configuration structure
typedef struct {
    microlinkProtocol_e protocol;
    microlinkOrientation_e orientation;
    uint16_t flowScale;  // Flow scale factor (e.g., 100 or 10000)
} microlinkConfig_t;

bool opticalFlowMicrolinkDetect(opticalFlowDev_t *dev);

// Get LIDAR/rangefinder data from MicroLink sensor
uint32_t opticalFlowMicrolinkGetDistance(void);
uint8_t opticalFlowMicrolinkGetStrength(void);

// Configure MicroLink sensor
bool opticalFlowMicrolinkSetConfig(const microlinkConfig_t *config);
