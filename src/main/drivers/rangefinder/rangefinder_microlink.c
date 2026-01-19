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

// MicroLink MTF-01/MTF-02 LIDAR specifications
#define MICROLINK_RANGE_MIN 40      // 4cm minimum range
#define MICROLINK_RANGE_MAX 12000   // 12m maximum range (12000mm)
#define MICROLINK_DETECTION_CONE_DECIDEGREES 900  // 90 degrees

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
    
    // Get distance from optical flow driver (in mm)
    uint32_t distanceMm = opticalFlowMicrolinkGetDistance();
    
    // Check if distance is valid
    if (distanceMm == 0) {
        // 0 indicates unavailable/out of range
        return RANGEFINDER_OUT_OF_RANGE;
    }
    
    // Convert mm to cm
    int32_t distanceCm = distanceMm / 10;
    
    // Check if within valid range
    if (distanceCm < (MICROLINK_RANGE_MIN / 10) || distanceCm > (MICROLINK_RANGE_MAX / 10)) {
        return RANGEFINDER_OUT_OF_RANGE;
    }
    
    return distanceCm;
}

// Get MicroLink signal strength (quality indicator)
uint8_t rangefinderMicrolinkGetQuality(void)
{
    return opticalFlowMicrolinkGetStrength();
}

bool rangefinderMicrolinkDetect(rangefinderDev_t *dev)
{
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
