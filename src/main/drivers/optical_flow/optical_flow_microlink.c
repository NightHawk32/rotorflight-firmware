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

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/optical_flow/optical_flow.h"
#include "drivers/optical_flow/optical_flow_microlink.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

// MicroLink Protocol Definitions (MTF-01/MTF-02)
// UART: 115200 baud, 8N1
// Update rate: 50Hz

#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             (MICOLINK_MAX_PAYLOAD_LEN + 7)

// Message IDs
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51
#define MICOLINK_MSG_ID_CONFIG       0x02

// Configuration payload length
#define MICOLINK_CONFIG_PAYLOAD_LEN  18

// Parser states
typedef enum {
    MICOLINK_STATE_IDLE = 0,
    MICOLINK_STATE_DEV_ID,
    MICOLINK_STATE_SYS_ID,
    MICOLINK_STATE_MSG_ID,
    MICOLINK_STATE_SEQ,
    MICOLINK_STATE_LEN,
    MICOLINK_STATE_PAYLOAD,
    MICOLINK_STATE_CHECKSUM
} micolinkState_e;

// Message structure
typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;
    
    micolinkState_e state;
    uint8_t payload_cnt;
} micolinkMsg_t;

// Range sensor payload (packed structure)
typedef struct __attribute__((packed)) {
    uint32_t time_ms;        // System time in ms
    uint32_t distance;       // distance (mm), 0 indicates unavailable
    uint8_t  strength;       // signal strength
    uint8_t  precision;      // distance precision
    uint8_t  dis_status;     // distance status
    uint8_t  reserved1;      // reserved
    int16_t  flow_vel_x;     // optical flow velocity in x (cm/s@1m)
    int16_t  flow_vel_y;     // optical flow velocity in y (cm/s@1m)
    uint8_t  flow_quality;   // optical flow quality
    uint8_t  flow_status;    // optical flow status
    uint16_t reserved2;      // reserved
} micolinkPayloadRangeSensor_t;

// Driver data
typedef struct {
    int16_t flowX;          // Flow in X direction (scaled to cm/s)
    int16_t flowY;          // Flow in Y direction (scaled to cm/s)
    uint8_t quality;
    uint32_t distance;       // LIDAR distance in mm
    uint8_t strength;        // LIDAR signal strength
    bool newData;
    
    micolinkMsg_t msg;
    serialPort_t *serialPort;
} opticalFlowMicrolinkData_t;

static opticalFlowMicrolinkData_t microlinkData = {0};

// Calculate checksum
static bool micolinkCheckSum(const micolinkMsg_t *msg)
{
    uint8_t checksum = 0;
    
    checksum += msg->head;
    checksum += msg->dev_id;
    checksum += msg->sys_id;
    checksum += msg->msg_id;
    checksum += msg->seq;
    checksum += msg->len;
    
    for (uint8_t i = 0; i < msg->len; i++) {
        checksum += msg->payload[i];
    }
    
    return (checksum == msg->checksum);
}

// Process received message
static void micolinkProcessMessage(const micolinkMsg_t *msg)
{
    if (msg->msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
        if (msg->len == sizeof(micolinkPayloadRangeSensor_t)) {
            micolinkPayloadRangeSensor_t payload;
            memcpy(&payload, msg->payload, sizeof(payload));
            
            // Update LIDAR/rangefinder data first
            microlinkData.distance = payload.distance;
            microlinkData.strength = payload.strength;
            
            // Convert optical flow data: flow_vel is in cm/s@1m, need to scale by actual distance
            // True flow (cm/s) = flow_vel_x * distance(m)
            // Distance is in mm, so distance(m) = distance / 1000
            if (payload.distance > 0) {
                // Scale flow by distance: flowX = flow_vel_x * (distance_mm / 1000)
                microlinkData.flowX = (payload.flow_vel_x * (int32_t)payload.distance) / 1000;
                microlinkData.flowY = (payload.flow_vel_y * (int32_t)payload.distance) / 1000;
            } else {
                // No valid distance, can't scale flow properly
                microlinkData.flowX = 0;
                microlinkData.flowY = 0;
            }
            microlinkData.quality = payload.flow_quality;
            
            microlinkData.newData = true;
        }
    }
}

// Parse incoming byte
static bool micolinkParseChar(micolinkMsg_t *msg, uint8_t data)
{
    switch (msg->state) {
        case MICOLINK_STATE_IDLE:
            if (data == MICOLINK_MSG_HEAD) {
                msg->head = data;
                msg->state = MICOLINK_STATE_DEV_ID;
            }
            break;
            
        case MICOLINK_STATE_DEV_ID:
            msg->dev_id = data;
            msg->state = MICOLINK_STATE_SYS_ID;
            break;
        
        case MICOLINK_STATE_SYS_ID:
            msg->sys_id = data;
            msg->state = MICOLINK_STATE_MSG_ID;
            break;
        
        case MICOLINK_STATE_MSG_ID:
            msg->msg_id = data;
            msg->state = MICOLINK_STATE_SEQ;
            break;
        
        case MICOLINK_STATE_SEQ:
            msg->seq = data;
            msg->state = MICOLINK_STATE_LEN;
            break;
        
        case MICOLINK_STATE_LEN:
            msg->len = data;
            if (msg->len == 0) {
                msg->state = MICOLINK_STATE_CHECKSUM;
            } else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) {
                msg->state = MICOLINK_STATE_IDLE;
            } else {
                msg->payload_cnt = 0;
                msg->state = MICOLINK_STATE_PAYLOAD;
            }
            break;
            
        case MICOLINK_STATE_PAYLOAD:
            msg->payload[msg->payload_cnt++] = data;
            if (msg->payload_cnt >= msg->len) {
                msg->state = MICOLINK_STATE_CHECKSUM;
            }
            break;
            
        case MICOLINK_STATE_CHECKSUM:
            msg->checksum = data;
            msg->state = MICOLINK_STATE_IDLE;
            msg->payload_cnt = 0;
            
            if (micolinkCheckSum(msg)) {
                return true;
            }
            break;
            
        default:
            msg->state = MICOLINK_STATE_IDLE;
            msg->payload_cnt = 0;
            break;
    }
    
    return false;
}

static void opticalFlowMicrolinkInit(opticalFlowDev_t *dev)
{
    UNUSED(dev);
    
    microlinkData.flowX = 0;
    microlinkData.flowY = 0;
    microlinkData.quality = 0;
    microlinkData.distance = 0;
    microlinkData.strength = 0;
    microlinkData.newData = false;
    microlinkData.msg.state = MICOLINK_STATE_IDLE;
    microlinkData.msg.payload_cnt = 0;
    
    // Initialize serial port for MicroLink
    // Serial port configured as: 115200 baud, 8N1
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MICROLINK);
    
    if (portConfig) {
        microlinkData.serialPort = openSerialPort(portConfig->identifier, FUNCTION_MICROLINK, NULL, NULL, 115200, MODE_RXTX, 0);
    } else {
        microlinkData.serialPort = NULL;
    }
}

static void opticalFlowMicrolinkUpdate(opticalFlowDev_t *dev)
{
    UNUSED(dev);
    
    // Process incoming serial data
    if (microlinkData.serialPort) {
        while (serialRxBytesWaiting(microlinkData.serialPort) > 0) {
            uint8_t c = serialRead(microlinkData.serialPort);
            
            if (micolinkParseChar(&microlinkData.msg, c)) {
                micolinkProcessMessage(&microlinkData.msg);
            }
        }
    }
}

static bool opticalFlowMicrolinkRead(opticalFlowDev_t *dev, int16_t *flowX, int16_t *flowY, uint8_t *quality)
{
    UNUSED(dev);
    
    if (!microlinkData.newData) {
        return false;
    }
    
    *flowX = microlinkData.flowX;
    *flowY = microlinkData.flowY;
    *quality = microlinkData.quality;
    
    microlinkData.newData = false;
    
    return true;
}

bool opticalFlowMicrolinkDetect(opticalFlowDev_t *dev)
{
    dev->init = opticalFlowMicrolinkInit;
    dev->update = opticalFlowMicrolinkUpdate;
    dev->read = opticalFlowMicrolinkRead;
    
    return true;
}

// Get LIDAR distance (for rangefinder integration)
uint32_t opticalFlowMicrolinkGetDistance(void)
{
    return microlinkData.distance;
}

// Get LIDAR signal strength
uint8_t opticalFlowMicrolinkGetStrength(void)
{
    return microlinkData.strength;
}

// Send configuration to MicroLink sensor
bool opticalFlowMicrolinkSetConfig(const microlinkConfig_t *config)
{
    if (!microlinkData.serialPort || !config) {
        return false;
    }
    
    // Build configuration message based on recovered data format:
    // 0xEF·0x01·0x00·0x02·SEQ·PAYLOAD_LEN·0x12·0x02·0x00·PROT·0x00·ORI·FS_LSB·FS_MSB·...·CHECKSUM
    
    uint8_t configMsg[MICOLINK_MAX_LEN];
    uint8_t idx = 0;
    
    // Header
    configMsg[idx++] = MICOLINK_MSG_HEAD;  // 0xEF
    configMsg[idx++] = 0x01;               // dev_id
    configMsg[idx++] = 0x00;               // sys_id
    configMsg[idx++] = MICOLINK_MSG_ID_CONFIG;  // 0x02 (msg_id)
    configMsg[idx++] = 0x00;               // seq (sequence number, can be incremented if needed)
    
    // Payload length placeholder (will be filled later)
    uint8_t payloadLenIdx = idx;
    configMsg[idx++] = 0x00;  // Placeholder for payload length
    
    // Payload starts here
    uint8_t payloadStart = idx;
    
    // Command ID
    configMsg[idx++] = 0x12;  // Config command
    configMsg[idx++] = 0x02;  // Sub-command
    
    // Protocol type (byte 8 in the frame)
    configMsg[idx++] = 0x00;  // Reserved
    configMsg[idx++] = (uint8_t)config->protocol;  // Protocol: 0=Microlink, 1=MSP, 2=MAV_APM, 3=MAV_PX4
    
    // Orientation (bytes 10-11)
    configMsg[idx++] = 0x00;  // Reserved
    configMsg[idx++] = (uint8_t)config->orientation;  // Orientation: 0=0°, 1=90°, 2=180°, 3=270°
    
    // Flow scale (bytes 12-13, little-endian)
    configMsg[idx++] = (uint8_t)(config->flowScale & 0xFF);        // Flow scale LSB
    configMsg[idx++] = (uint8_t)((config->flowScale >> 8) & 0xFF); // Flow scale MSB
    
    // Fill remaining payload bytes with standard values from the recovered data
    configMsg[idx++] = 0x00;  // byte 14
    configMsg[idx++] = 0x04;  // byte 15
    configMsg[idx++] = 0x09;  // byte 16
    configMsg[idx++] = 0x00;  // byte 17
    configMsg[idx++] = 0x00;  // byte 18
    configMsg[idx++] = 0x01;  // byte 19
    configMsg[idx++] = 0x58;  // byte 20
    configMsg[idx++] = 0x00;  // byte 21
    configMsg[idx++] = 0x00;  // byte 22
    configMsg[idx++] = 0x00;  // byte 23
    configMsg[idx++] = 0x00;  // byte 24
    
    // Calculate payload length
    uint8_t payloadLen = idx - payloadStart;
    configMsg[payloadLenIdx] = payloadLen;
    
    // Calculate checksum (sum of all bytes except checksum itself)
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < idx; i++) {
        checksum += configMsg[i];
    }
    configMsg[idx++] = checksum;
    
    // Send the configuration message
    for (uint8_t i = 0; i < idx; i++) {
        serialWrite(microlinkData.serialPort, configMsg[i]);
    }
    
    return true;
}

#endif // USE_OPTICAL_FLOW
