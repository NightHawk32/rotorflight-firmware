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
#include <string.h>

#include "platform.h"

#ifdef USE_OPTICAL_FLOW

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"

#include "drivers/optical_flow/optical_flow.h"
#include "drivers/optical_flow/optical_flow_microlink.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/optical_flow.h"

#define OPTICAL_FLOW_HARDWARE_TIMEOUT_MS    500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

opticalFlow_t opticalFlow;

/*
 * Detect which optical flow sensor is present
 */
static bool opticalFlowDetect(opticalFlowDev_t * dev, uint8_t opticalFlowHardwareToUse)
{
    opticalFlowType_e opticalFlowHardware = OPTICAL_FLOW_NONE;
    requestedSensors[SENSOR_INDEX_OPTICAL_FLOW] = opticalFlowHardwareToUse;

    switch (opticalFlowHardwareToUse) {
        case OPTICAL_FLOW_MICROLINK:
            if (opticalFlowMicrolinkDetect(dev)) {
                opticalFlowHardware = OPTICAL_FLOW_MICROLINK;
                rescheduleTask(TASK_OPTICAL_FLOW, TASK_PERIOD_MS(20)); // 50Hz
            }
            break;

        case OPTICAL_FLOW_NONE:
            opticalFlowHardware = OPTICAL_FLOW_NONE;
            break;
    }

    if (opticalFlowHardware == OPTICAL_FLOW_NONE) {
        sensorsClear(SENSOR_OPTICAL_FLOW);
        return false;
    }

    detectedSensors[SENSOR_INDEX_OPTICAL_FLOW] = opticalFlowHardware;
    sensorsSet(SENSOR_OPTICAL_FLOW);
    return true;
}

bool opticalFlowInit(void)
{
    if (!opticalFlowDetect(&opticalFlow.dev, opticalFlowConfig()->optical_flow_hardware)) {
        return false;
    }

    if (opticalFlow.dev.init) {
        opticalFlow.dev.init(&opticalFlow.dev);
    }
    opticalFlow.flowX = 0;
    opticalFlow.flowY = 0;
    opticalFlow.quality = 0;
    opticalFlow.lastValidResponseTimeMs = millis();

    return true;
}

/*
 * Rotate a sensor-frame flow sample into the body frame (x forward, y left)
 * according to optical_flow_align.  CWn: the sensor's X axis points n
 * degrees clockwise from the nose, seen from above.  FLIP mirrors the
 * sensor's Y axis before the rotation.
 */
static void opticalFlowAlign(int16_t *flowX, int16_t *flowY)
{
    const uint8_t align = opticalFlowConfig()->optical_flow_align;
    const int16_t sx = *flowX;
    const int16_t sy = (align >= OPTICAL_FLOW_ALIGN_CW0FLIP) ? -*flowY : *flowY;

    switch (align % 4) {
        case 1:     // CW90: sensor X = body right, sensor Y = body forward
            *flowX = sy;
            *flowY = -sx;
            break;
        case 2:     // CW180
            *flowX = -sx;
            *flowY = -sy;
            break;
        case 3:     // CW270: sensor X = body left, sensor Y = body backward
            *flowX = -sy;
            *flowY = sx;
            break;
        default:    // CW0
            *flowX = sx;
            *flowY = sy;
            break;
    }
}

/*
 * This is called periodically by the scheduler
 */
void opticalFlowUpdate(void)
{
    if (opticalFlow.dev.update) {
        opticalFlow.dev.update(&opticalFlow.dev);
    }
    
    // Try to read new data
    if (opticalFlow.dev.read) {
        int16_t flowX, flowY;
        uint8_t quality;
        
        if (opticalFlow.dev.read(&opticalFlow.dev, &flowX, &flowY, &quality)) {
            opticalFlowAlign(&flowX, &flowY);
            opticalFlow.lastValidResponseTimeMs = millis();
            opticalFlow.flowX = flowX;
            opticalFlow.flowY = flowY;
            opticalFlow.quality = quality;
            
            DEBUG_SET(DEBUG_OPTICAL_FLOW, 0, flowX);
            DEBUG_SET(DEBUG_OPTICAL_FLOW, 1, flowY);
            DEBUG_SET(DEBUG_OPTICAL_FLOW, 2, quality);
        }
    }
}

/**
 * Get the latest flow X value
 */
int16_t opticalFlowGetLatestX(void)
{
    return opticalFlow.flowX;
}

/**
 * Get the latest flow Y value
 */
int16_t opticalFlowGetLatestY(void)
{
    return opticalFlow.flowY;
}

/**
 * Get the latest quality value
 */
uint8_t opticalFlowGetLatestQuality(void)
{
    return opticalFlow.quality;
}

/**
 * Timestamp of the most recent successfully decoded sample.
 * Lets consumers (the position estimator) fuse each sample exactly once.
 */
timeMs_t opticalFlowGetLastUpdateMs(void)
{
    return opticalFlow.lastValidResponseTimeMs;
}

bool opticalFlowIsHealthy(void)
{
    return (millis() - opticalFlow.lastValidResponseTimeMs) < OPTICAL_FLOW_HARDWARE_TIMEOUT_MS;
}

#endif