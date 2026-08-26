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

#include "platform.h"

#ifdef USE_CRSF_SENSORS

#include <math.h>
#include <string.h>

#include "build/debug.h"
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/crsf_sensors.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "pg/crsf_sensors.h"
#include "rx/crsf_protocol.h"

#define CRSF_SENSORS_FRAME_BUFFER_SIZE CRSF_FRAME_SIZE_MAX

typedef struct crsfSensorsFrame_s {
    uint8_t data[CRSF_SENSORS_FRAME_BUFFER_SIZE];
    uint8_t length;
    bool valid;
} crsfSensorsFrame_t;

static serialPort_t *crsfSensorsPort;
static volatile uint8_t rxBuffer[CRSF_SENSORS_FRAME_BUFFER_SIZE];
static volatile uint8_t rxPosition;
static volatile uint8_t rxExpectedLength;
static volatile bool rxFrameReady;
static crsfSensorsFrame_t processFrame;

static crsfSensorsGpsData_t gpsData;
static crsfSensorsBatteryData_t batteryData;
static crsfSensorsBaroData_t baroData;
static bool useBaroAltitude;

static uint16_t be16Read(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t be24Read(const uint8_t *p)
{
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

static uint32_t be32Read(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static uint8_t crsfSensorsCrc8(const volatile uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static int32_t decodeBaroAltitudeCm(uint16_t packed)
{
    if (packed & 0x8000) {
        return (int32_t)(packed & 0x7FFF) * 100;
    }

    return ((int32_t)packed - 10000) * 10;
}

static int16_t decodeVerticalSpeedCmS(int8_t packed)
{
    if (packed == 0) {
        return 0;
    }

    const int sign = packed > 0 ? 1 : -1;
    const float magnitude = (expf(fabsf((float)packed) * 0.026f) - 1.0f) * 100.0f;
    return (int16_t)lrintf(magnitude * sign);
}

static void handleGpsFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    if (payloadLength != CRSF_FRAME_GPS_PAYLOAD_SIZE) {
        return;
    }

    gpsData.latitude = (int32_t)be32Read(&payload[0]);
    gpsData.longitude = (int32_t)be32Read(&payload[4]);

    const uint16_t groundspeedKmh100 = be16Read(&payload[8]);
    gpsData.groundspeedCmS = (uint16_t)((uint32_t)groundspeedKmh100 * 100U / 36U);
    gpsData.headingDeg10 = be16Read(&payload[10]) / 10U;
    gpsData.altitudeCm = ((int32_t)be16Read(&payload[12]) - 1000) * 100;
    gpsData.satellites = payload[14];
    gpsData.valid = true;
    gpsData.lastUpdateUs = currentTimeUs;
}

static void handleBatteryFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    if (payloadLength != CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE) {
        return;
    }

    const uint16_t rawVoltage = be16Read(&payload[0]);
    const uint16_t rawCurrent = be16Read(&payload[2]);

    batteryData.voltageMv = (uint32_t)rawVoltage * 100U;
    batteryData.currentMa = (uint32_t)rawCurrent * 100U;
    batteryData.capacityMah = be24Read(&payload[4]);
    batteryData.remainingPct = payload[7];
    batteryData.valid = true;
    batteryData.lastUpdateUs = currentTimeUs;
}

static void handleBaroFrame(const uint8_t *payload, uint8_t payloadLength, timeUs_t currentTimeUs)
{
    if (payloadLength < 3) {
        return;
    }

    baroData.altitudeCm = decodeBaroAltitudeCm(be16Read(&payload[0]));
    baroData.verticalSpeedCmS = decodeVerticalSpeedCmS((int8_t)payload[2]);
    baroData.valid = true;
    baroData.lastUpdateUs = currentTimeUs;
}

static void processReceivedFrame(timeUs_t currentTimeUs)
{
    if (!processFrame.valid || processFrame.length < 5) {
        return;
    }

    const uint8_t frameLength = processFrame.data[1];
    if (frameLength < 2 || (uint8_t)(frameLength + 2) != processFrame.length) {
        return;
    }

    const uint8_t type = processFrame.data[2];
    const uint8_t payloadLength = frameLength - CRSF_FRAME_LENGTH_TYPE_CRC;
    const uint8_t *payload = &processFrame.data[3];

    switch (type) {
    case CRSF_FRAMETYPE_GPS:
        handleGpsFrame(payload, payloadLength, currentTimeUs);
        break;
    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        handleBatteryFrame(payload, payloadLength, currentTimeUs);
        break;
    case CRSF_FRAMETYPE_ALTITUDE_SENSOR:
        handleBaroFrame(payload, payloadLength, currentTimeUs);
        break;
    default:
        break;
    }
}

static void crsfSensorsDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const uint8_t byte = (uint8_t)c;

    if (rxPosition == 0) {
        if (byte != CRSF_SYNC_BYTE && byte != CRSF_ADDRESS_CRSF_RECEIVER) {
            return;
        }
        rxBuffer[rxPosition++] = byte;
        rxExpectedLength = 0;
        return;
    }

    if (rxPosition == 1) {
        if (byte < 2 || byte > (CRSF_FRAME_SIZE_MAX - 2)) {
            rxPosition = 0;
            return;
        }
        rxBuffer[rxPosition++] = byte;
        rxExpectedLength = (uint8_t)(byte + 2);
        return;
    }

    if (rxPosition >= CRSF_SENSORS_FRAME_BUFFER_SIZE || (rxExpectedLength != 0 && rxPosition >= rxExpectedLength)) {
        rxPosition = 0;
        rxExpectedLength = 0;
        return;
    }

    rxBuffer[rxPosition++] = byte;

    if (rxExpectedLength != 0 && rxPosition == rxExpectedLength) {
        const uint8_t crc = crsfSensorsCrc8(&rxBuffer[2], (uint8_t)(rxExpectedLength - 3));
        if (crc == rxBuffer[rxExpectedLength - 1] && !rxFrameReady) {
            memcpy((void *)processFrame.data, (const void *)rxBuffer, rxExpectedLength);
            processFrame.length = rxExpectedLength;
            processFrame.valid = true;
            rxFrameReady = true;
        }
        rxPosition = 0;
        rxExpectedLength = 0;
    }
}

void crsfSensorsInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_CRSF_SENSORS);

    crsfSensorsPort = NULL;
    rxPosition = 0;
    rxExpectedLength = 0;
    rxFrameReady = false;
    processFrame.valid = false;
    memset(&gpsData, 0, sizeof(gpsData));
    memset(&batteryData, 0, sizeof(batteryData));
    memset(&baroData, 0, sizeof(baroData));
    useBaroAltitude = crsfSensorsConfig()->useBaroAltitude != 0;

    if (!portConfig) {
        return;
    }

    crsfSensorsPort = openSerialPort(portConfig->identifier,
        FUNCTION_CRSF_SENSORS,
        crsfSensorsDataReceive,
        NULL,
        CRSF_BAUDRATE,
        MODE_RX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED);
}

void crsfSensorsUpdate(timeUs_t currentTimeUs)
{
    const timeDelta_t timeoutUs = (timeDelta_t)crsfSensorsConfig()->sensorTimeoutMs * 1000;
    useBaroAltitude = crsfSensorsConfig()->useBaroAltitude != 0;

    if (rxFrameReady) {
        rxFrameReady = false;
        processReceivedFrame(currentTimeUs);
        processFrame.valid = false;
    }

    if (gpsData.valid && cmpTimeUs(currentTimeUs, gpsData.lastUpdateUs) > timeoutUs) {
        gpsData.valid = false;
    }
    if (batteryData.valid && cmpTimeUs(currentTimeUs, batteryData.lastUpdateUs) > timeoutUs) {
        batteryData.valid = false;
    }
    if (baroData.valid && cmpTimeUs(currentTimeUs, baroData.lastUpdateUs) > timeoutUs) {
        baroData.valid = false;
    }
}

bool crsfSensorsIsEnabled(void)
{
    return crsfSensorsPort != NULL;
}

bool crsfSensorsGetGpsData(crsfSensorsGpsData_t *data)
{
    if (!gpsData.valid) {
        return false;
    }

    if (data) {
        *data = gpsData;
    }
    return true;
}

bool crsfSensorsHasGpsData(void)
{
    return gpsData.valid;
}

bool crsfSensorsGetBatteryData(crsfSensorsBatteryData_t *data)
{
    if (!batteryData.valid) {
        return false;
    }

    if (data) {
        *data = batteryData;
    }
    return true;
}

bool crsfSensorsHasBatteryData(void)
{
    return batteryData.valid;
}

bool crsfSensorsGetBaroData(crsfSensorsBaroData_t *data)
{
    if (!baroData.valid) {
        return false;
    }

    if (data) {
        *data = baroData;
    }
    return true;
}

bool crsfSensorsHasBaroData(void)
{
    return baroData.valid;
}

void crsfSensorsSetBaroUse(bool enabled)
{
    useBaroAltitude = enabled;
}

bool crsfSensorsGetBaroUse(void)
{
    return useBaroAltitude;
}

#endif
