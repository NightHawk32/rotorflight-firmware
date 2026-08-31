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

#if defined(USE_RANGEFINDER) && defined(USE_OPTICAL_FLOW)

#include "build/debug.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_microlink.h"
#include "drivers/optical_flow/optical_flow_microlink.h"

#include "io/serial.h"

#include "pg/optical_flow.h"

// MicroLink MTF-01/MTF-02 LIDAR specifications
#define MICROLINK_RANGE_MIN 40      // 4cm minimum range
#define MICROLINK_RANGE_MAX 12000   // 12m maximum range (12000mm)
#define MICROLINK_DETECTION_CONE_DECIDEGREES 900  // 90 degrees
// The module streams at 50Hz; no frame for this long means the link is dead
#define MICROLINK_FRAME_TIMEOUT_MS  200

static void rangefinderMicrolinkInit(rangefinderDev_t *dev)
{
    UNUSED(dev);
    // No initialization needed - optical flow driver handles serial port
}

static void rangefinderMicrolinkUpdate(rangefinderDev_t *dev)
{
    UNUSED(dev);
    // No update needed - optical flow driver handles data reception
}

// Return distance in centimeters
// Returns RANGEFINDER_NO_NEW_DATA if no new data available
static int32_t rangefinderMicrolinkRead(rangefinderDev_t *dev)
{
    UNUSED(dev);

    // The shared parser lives in the optical-flow driver, so this read() is
    // polled independently of frame arrival.  Report NO_NEW_DATA for a repeat
    // of a sample already consumed, and HARDWARE_FAILURE once the stream has
    // stopped - otherwise sensors/rangefinder.c keeps refreshing its
    // last-valid-response timestamp and a dead lidar reads as a perfectly
    // healthy, perfectly constant altitude.
    static uint32_t lastFrameCount = 0;

    const uint32_t frameCount = opticalFlowMicrolinkGetFrameCount();

    if (frameCount == lastFrameCount) {
        const timeMs_t lastFrameMs = opticalFlowMicrolinkGetLastFrameMs();
        if (lastFrameMs == 0 || (millis() - lastFrameMs) > MICROLINK_FRAME_TIMEOUT_MS) {
            return RANGEFINDER_HARDWARE_FAILURE;
        }
        return RANGEFINDER_NO_NEW_DATA;
    }
    lastFrameCount = frameCount;

    // Get distance from optical flow driver (in mm)
    uint32_t distanceMm = opticalFlowMicrolinkGetDistance();
    
    // Check if distance is valid
    if (distanceMm == 0) {
        // 0 indicates unavailable/out of range
        return RANGEFINDER_OUT_OF_RANGE;
    }
    
    // Range-check in mm before the lossy conversion to cm
    if (distanceMm < MICROLINK_RANGE_MIN || distanceMm > MICROLINK_RANGE_MAX) {
        return RANGEFINDER_OUT_OF_RANGE;
    }

    return (int32_t)(distanceMm / 10);
}

// Get MicroLink signal strength (quality indicator)
uint8_t rangefinderMicrolinkGetQuality(void)
{
    return opticalFlowMicrolinkGetStrength();
}

bool rangefinderMicrolinkDetect(rangefinderDev_t *dev)
{
    // The MicroLink LIDAR data is received via the optical-flow driver's
    // shared UART/parser. Only report the rangefinder as present if that
    // UART is actually wired up and the optical-flow sensor is configured
    // to use it, otherwise nothing will ever open the port or feed data in.
    if (!findSerialPortConfig(FUNCTION_MICROLINK) ||
        opticalFlowConfig()->optical_flow_hardware != OPTICAL_FLOW_MICROLINK) {
        return false;
    }

    dev->delayMs = 10;  // 10ms delay between readings (100Hz update rate)
    dev->maxRangeCm = MICROLINK_RANGE_MAX / 10;  // Convert mm to cm
    dev->detectionConeDeciDegrees = MICROLINK_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = MICROLINK_DETECTION_CONE_DECIDEGREES;
    
    dev->init = &rangefinderMicrolinkInit;
    dev->update = &rangefinderMicrolinkUpdate;
    dev->read = &rangefinderMicrolinkRead;
    
    return true;
}

#endif // USE_RANGEFINDER && USE_OPTICAL_FLOW
