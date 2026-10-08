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

#include "types.h"
#include "platform.h"

#include "pg/pg.h"

typedef enum {
    OPTICAL_FLOW_NONE        = 0,
    OPTICAL_FLOW_MICROLINK   = 1,
} opticalFlowType_e;

// Mounting of the flow sensor, seen from above: CWn = the sensor's X axis
// points n degrees clockwise from the nose.  FLIP mirrors the sensor's Y
// axis first, for a module whose axes are left-handed in the body frame.
typedef enum {
    OPTICAL_FLOW_ALIGN_CW0       = 0,
    OPTICAL_FLOW_ALIGN_CW90      = 1,
    OPTICAL_FLOW_ALIGN_CW180     = 2,
    OPTICAL_FLOW_ALIGN_CW270     = 3,
    OPTICAL_FLOW_ALIGN_CW0FLIP   = 4,
    OPTICAL_FLOW_ALIGN_CW90FLIP  = 5,
    OPTICAL_FLOW_ALIGN_CW180FLIP = 6,
    OPTICAL_FLOW_ALIGN_CW270FLIP = 7,
    OPTICAL_FLOW_ALIGN_COUNT
} opticalFlowAlign_e;

typedef struct {
    uint8_t optical_flow_hardware;
    uint8_t optical_flow_align;     // opticalFlowAlign_e
} opticalFlowConfig_t;

PG_DECLARE(opticalFlowConfig_t, opticalFlowConfig);