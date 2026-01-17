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

#define OPTICAL_FLOW_OUT_OF_RANGE        (-1)
#define OPTICAL_FLOW_HARDWARE_FAILURE    (-2)
#define OPTICAL_FLOW_NO_NEW_DATA         (-3)

struct opticalFlowDev_s;
typedef void (*opticalFlowOpInitFuncPtr)(struct opticalFlowDev_s * dev);
typedef void (*opticalFlowOpUpdateFuncPtr)(struct opticalFlowDev_s * dev);
typedef bool (*opticalFlowOpReadFuncPtr)(struct opticalFlowDev_s * dev, int16_t *flowX, int16_t *flowY, uint8_t *quality);

typedef struct opticalFlowDev_s {
    // function pointers
    opticalFlowOpInitFuncPtr init;
    opticalFlowOpUpdateFuncPtr update;
    opticalFlowOpReadFuncPtr read;
} opticalFlowDev_t;