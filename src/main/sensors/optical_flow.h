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

#include "drivers/optical_flow/optical_flow.h"

#include "pg/optical_flow.h"


typedef struct opticalFlow_s {
    opticalFlowDev_t dev;
    int16_t flowX;
    int16_t flowY;
    uint8_t quality;
    timeMs_t lastValidResponseTimeMs;
} opticalFlow_t;

bool opticalFlowInit(void);

int16_t opticalFlowGetLatestX(void);
int16_t opticalFlowGetLatestY(void);
uint8_t opticalFlowGetLatestQuality(void);

void opticalFlowUpdate(void);
bool opticalFlowIsHealthy(void);