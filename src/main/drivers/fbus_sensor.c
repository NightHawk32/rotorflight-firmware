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

#include <math.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "pg/gps.h"
#include "pg/fbus_master.h"

#include "common/maths.h"
#include "config/feature.h"
#include "fc/runtime_config.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "drivers/fbus_sensor.h"
#include "sensors/sensors.h"

#ifdef USE_FBUS_MASTER
#include "drivers/fbus_xact.h"
#endif

#if defined(USE_FBUS_MASTER) || defined(USE_SPORT_MASTER)

static const char* const fbusSensorNames[] = {
    [FBUS_SENSOR_VARIO2]        = "VARIO2",
    [FBUS_SENSOR_FLVSS]         = "FLVSS",
    [FBUS_SENSOR_CURRENT]       = "CURRENT",
    [FBUS_SENSOR_GPS]           = "GPS",
    [FBUS_SENSOR_RPM]           = "RPM",
    [FBUS_SENSOR_SP2UART_A]     = "SP2UART_A",
    [FBUS_SENSOR_SP2UART_B]     = "SP2UART_B",
    [FBUS_SENSOR_FAS_150S]      = "FAS_150S",
    [FBUS_SENSOR_SBEC]          = "SBEC",
    [FBUS_SENSOR_AIR_SPEED]     = "AIR_SPEED",
    [FBUS_SENSOR_ESC]           = "ESC",
    [FBUS_SENSOR_PSD_30]        = "PSD_30",
    [FBUS_SENSOR_XACT_SERVO]    = "XACT_SERVO",
    [FBUS_SENSOR_SD1]           = "SD1",
    [FBUS_SENSOR_VS600]         = "VS600",
    [FBUS_SENSOR_GASSUIT]       = "GASSUIT",
    [FBUS_SENSOR_FSD]           = "FSD",
    [FBUS_SENSOR_GATEWAY]       = "GATEWAY",
    [FBUS_SENSOR_REDUNDANCY_BUS] = "REDUNDANCY_BUS",
    [FBUS_SENSOR_S6R]           = "S6R",
};

static fbusGpsData_t fbusGps;
static fbusServoData_t fbusServo;
static fbusCurrentData_t fbusCurrent;
static fbusEscData_t fbusEsc;

static fbusFlvssData_t fbusFlvss;

// GPS ADV Plus nav mode. Blocks are assembled in the FBUS receive interrupt and handed to
// fbusSensorUpdate() whole.
#define NAV_PART_LAT    BIT(0)
#define NAV_PART_LON    BIT(1)
#define NAV_PART_VEL    BIT(2)
#define NAV_PART_ACC    BIT(3)
#define NAV_PART_ALT    BIT(4)
#define NAV_PART_VZ     BIT(5)
#define NAV_PARTS_ALL   0x3F
#define NAV_TIMEOUT_US  1000000
#define NAV_MAG_TIMEOUT_US 500000

static fbusNavSolution_t navWork;
static uint8_t navParts;
static bool navOpen;
static fbusNavSolution_t navReady;
static volatile bool navReadyNew;
static volatile timeUs_t navLastUs;
static volatile int8_t navPhysId = -1;
static uint32_t navBlocks;
static uint32_t navDropped;

static int16_t magXY[2];
static bool magXYValid;
static int16_t magFRD[3];
static volatile timeUs_t magLastUs;

#ifdef USE_GPS
static timeUs_t legacyGpsConsumedUs;
#endif

#define FBUS_SENSOR_CACHE_SIZE 32
static fbusSensorData_t sensorCache[FBUS_SENSOR_CACHE_SIZE];
static uint8_t sensorCacheIndex = 0;
static fbusObservedSensor_t observedSensors[FBUS_MAX_OBSERVED_SENSORS];
static uint8_t observedSensorCount = 0;
static fbusSensorForwardBuffer_t forwardBuffers[FBUS_SENSOR_MAX_FORWARD_BUFFERS];

void fbusSensorInit(void)
{
    memset(&fbusGps, 0, sizeof(fbusGpsData_t));
    memset(&fbusServo, 0, sizeof(fbusServoData_t));
    memset(&fbusCurrent, 0, sizeof(fbusCurrentData_t));
    memset(&fbusEsc, 0, sizeof(fbusEscData_t));
    memset(&fbusFlvss, 0, sizeof(fbusFlvssData_t));
    memset(sensorCache, 0, sizeof(sensorCache));
    sensorCacheIndex = 0;
    memset(observedSensors, 0, sizeof(observedSensors));
    observedSensorCount = 0;
    
    fbusSensorInitForwarding();
}

int32_t fbusGpsConvertLatLon(uint32_t fbusData)
{
    bool isNegative = (fbusData & FBUS_GPS_LAT_LON_BIT30) != 0;
    uint32_t minutesTimes10000 = fbusData & FBUS_GPS_LAT_LON_DATA_MASK;
    
    float minutes = (float)minutesTimes10000 / 10000.0f;
    float degrees = minutes / 60.0f;
    
    int32_t result = (int32_t)(degrees * 1e7f);
    
    if (isNegative) {
        result = -result;
    }
    
    return result;
}

int32_t fbusGpsConvertAltitude(uint32_t fbusData)
{
    return (int32_t)fbusData;
}

uint16_t fbusGpsConvertSpeed(uint32_t fbusData)
{
    float knots = (float)fbusData / 1000.0f;
    float metersPerSec = knots * 0.514444f;
    uint16_t result = (uint16_t)(metersPerSec * 10.0f);
    
    return result;
}

uint16_t fbusGpsConvertCourse(uint32_t fbusData)
{
    uint16_t result = (uint16_t)(fbusData / 10);
    
    if (result >= 3600) {
        result = result % 3600;
    }
    
    return result;
}

bool fbusGpsConvertTime(uint32_t fbusData, fbusGpsTime_t *time)
{
    if (time) {
        memset(time, 0, sizeof(*time));
    }

    // Extract bytes: D3 is type, D4-D6 are data
    uint8_t type = (fbusData >> 24) & 0xFF;

    if (type == FBUS_GPS_TIME_TYPE_TIME && time) {
        time->seconds = (fbusData >> 16) & 0xFF;
        time->minutes = (fbusData >> 8) & 0xFF;
        time->hours = fbusData & 0xFF;
        return true;
    }

    return false;
}

bool fbusGpsConvertDate(uint32_t fbusData, fbusGpsDate_t *date)
{
    if (date) {
        memset(date, 0, sizeof(*date));
    }

    // Extract bytes: D3 is type, D4-D6 are data
    uint8_t type = (fbusData >> 24) & 0xFF;

    if (type == FBUS_GPS_TIME_TYPE_DATE && date) {
        date->day = (fbusData >> 16) & 0xFF;
        date->month = (fbusData >> 8) & 0xFF;
        date->year = 2000 + (fbusData & 0xFF);  // Assuming year is offset from 2000
        return true;
    }

    return false;
}

void fbusServoConvertData(uint32_t fbusData, uint16_t *current, uint16_t *voltage, uint16_t *temperature)
{
    *current = (fbusData & FBUS_SERVO_CURRENT_MASK);
    *voltage = ((fbusData & FBUS_SERVO_VOLTAGE_MASK) >> 8);
    *temperature = ((fbusData & FBUS_SERVO_TEMP_MASK) >> 16);
}

static uint16_t fbusFlvssConvertCellVoltage(uint16_t rawVoltage)
{
    return (rawVoltage + 2U) / 5U;
}

static void fbusFlvssUpdatePackVoltage(void)
{
    if (fbusFlvss.cellCount == 0 || fbusFlvss.cellCount > FBUS_FLVSS_MAX_CELLS) {
        fbusFlvss.voltageCentiVolts = 0;
        fbusFlvss.hasVoltage = false;
        return;
    }

    uint32_t totalVoltageCentiVolts = 0;

    for (uint8_t cellIndex = 0; cellIndex < fbusFlvss.cellCount; cellIndex++) {
        const uint16_t cellVoltage = fbusFlvss.cellVoltageCentiVolts[cellIndex];

        if (cellVoltage == 0) {
            fbusFlvss.voltageCentiVolts = 0;
            fbusFlvss.hasVoltage = false;
            return;
        }

        totalVoltageCentiVolts += cellVoltage;
    }

    fbusFlvss.voltageCentiVolts = totalVoltageCentiVolts;
    fbusFlvss.hasVoltage = true;
}

static fbusDetectedSensorType_e classifySensorTypeByAppId(uint16_t appId)
{
    // FrSky S.Port protocol: Application ID and physical ID are independent.
    // Classify by App ID signatures/ranges so sensors remain detectable even
    // when physical IDs are reconfigured.

    // GPS signature block: 0x0800, 0x0820, 0x0830, 0x0840, 0x0850 (+ low nibble group)
    if ((appId >= FBUS_GPS_LATITUDE_BASE && appId <= (FBUS_GPS_LATITUDE_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_ALTITUDE_BASE && appId <= (FBUS_GPS_ALTITUDE_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_SPEED_BASE && appId <= (FBUS_GPS_SPEED_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_COURSE_BASE && appId <= (FBUS_GPS_COURSE_BASE + 0x0F)) ||
        (appId >= FBUS_GPS_TIME_BASE && appId <= (FBUS_GPS_TIME_BASE + 0x0F))) {
        return FBUS_DETECTED_SENSOR_GPS;
    }

    // GPS ADV Plus nav mode
    switch (appId & 0xFFF0) {
    case FBUS_NAV_HDR_BASE:
    case FBUS_NAV_LAT_BASE:
    case FBUS_NAV_LON_BASE:
    case FBUS_NAV_VEL_BASE:
    case FBUS_NAV_ACC_BASE:
    case FBUS_NAV_ALT_BASE:
    case FBUS_NAV_VZ_BASE:
    case FBUS_NAV_MAG_XY_BASE:
    case FBUS_NAV_MAG_Z_BASE:
        return FBUS_DETECTED_SENSOR_GPS;
    default:
        break;
    }

    // XACT Servo signature block: 0x6800~0x680F
    if (appId >= FBUS_SERVO_DATA_BASE && appId <= (FBUS_SERVO_DATA_BASE + 0x0F)) {
        return FBUS_DETECTED_SENSOR_XACT_SERVO;
    }

    // ESC signature blocks from FrSky S.Port v3.5.8 table:
    // 0x0B50~0x0B5F (ESC_V/ESC_C), 0x0B60~0x0B6F (RPM/Consumption), 0x0B70~0x0B7F (Temp)
    if ((appId >= FBUS_ESC_POWER_BASE && appId <= (FBUS_ESC_POWER_BASE + 0x0F)) ||
        (appId >= FBUS_ESC_RPM_CONS_BASE && appId <= (FBUS_ESC_RPM_CONS_BASE + 0x0F)) ||
        (appId >= FBUS_ESC_TEMP_BASE && appId <= (FBUS_ESC_TEMP_BASE + 0x0F))) {
        return FBUS_DETECTED_SENSOR_ESC;
    }

    if (appId >= FBUS_FLVSS_VOLTAGE_BASE && appId <= (FBUS_FLVSS_VOLTAGE_BASE + 0x0F)) {
        return FBUS_DETECTED_SENSOR_FLVSS;
    }

    // FAS current/voltage sensor signatures seen in field captures and FrSky IDs:
    // current 0x0200~0x020F, voltage 0x0210~0x021F, high-precision current 0x0220~0x022F
    if ((appId >= 0x0200 && appId <= 0x020F) ||
        (appId >= 0x0210 && appId <= 0x021F) ||
        (appId >= 0x0220 && appId <= 0x022F)) {
        return FBUS_DETECTED_SENSOR_FAS_150S;
    }

    // FrSKY RPM S.PORT sensor https://www.frsky-rc.com/product/rpm/
    if ((appId >= FBUS_RPM_TEMP1_BASE && appId <= (FBUS_RPM_TEMP1_BASE + 0x0F)) ||
        (appId >= FBUS_RPM_TEMP2_BASE && appId <= (FBUS_RPM_TEMP2_BASE + 0x0F)) ||
        (appId >= FBUS_RPM_BASE && appId <= (FBUS_RPM_BASE + 0x0F))) {
        return FBUS_DETECTED_SENSOR_RPM;
    }

    return FBUS_DETECTED_SENSOR_UNKNOWN;
}

static fbusObservedSensor_t* getObservedSensor(uint8_t physicalId, fbusSensorSource_e source)
{
    for (uint8_t i = 0; i < observedSensorCount; i++) {
        if (observedSensors[i].physicalId == physicalId && observedSensors[i].source == source) {
            return &observedSensors[i];
        }
    }
    return NULL;
}

static fbusObservedSensor_t* getObservedSensorByPhysicalId(uint8_t physicalId)
{
    for (uint8_t i = 0; i < observedSensorCount; i++) {
        if (observedSensors[i].physicalId == physicalId) {
            return &observedSensors[i];
        }
    }
    return NULL;
}

static fbusObservedSensor_t* ensureObservedSensor(uint8_t physicalId, fbusSensorSource_e source)
{
    fbusObservedSensor_t *sensor = getObservedSensor(physicalId, source);
    if (!sensor && observedSensorCount < FBUS_MAX_OBSERVED_SENSORS) {
        sensor = &observedSensors[observedSensorCount++];
        sensor->physicalId = physicalId;
        sensor->source = source;
        sensor->appIdCount = 0;
        sensor->packetCount = 0;
        sensor->detectedType = FBUS_DETECTED_SENSOR_UNKNOWN;
    }

    return sensor;
}

static fbusObservedSensor_t* trackObservedSensor(uint8_t physicalId, uint16_t appId, timeUs_t currentTimeUs, fbusSensorSource_e source)
{
    fbusObservedSensor_t *sensor = ensureObservedSensor(physicalId, source);

    if (sensor) {
        sensor->lastSeenUs = currentTimeUs;
        sensor->packetCount++;

        bool appIdExists = false;
        for (uint8_t i = 0; i < sensor->appIdCount; i++) {
            if (sensor->appIds[i] == appId) {
                appIdExists = true;
                break;
            }
        }

        if (!appIdExists && sensor->appIdCount < 16) {
            sensor->appIds[sensor->appIdCount++] = appId;
        }

        if (sensor->detectedType == FBUS_DETECTED_SENSOR_UNKNOWN) {
            const fbusDetectedSensorType_e detected = classifySensorTypeByAppId(appId);
            if (detected != FBUS_DETECTED_SENSOR_UNKNOWN) {
                sensor->detectedType = detected;
            }
        }
    }

    return sensor;
}

void fbusSensorInitForwarding(void)
{
    memset(forwardBuffers, 0, sizeof(forwardBuffers));

    for (uint8_t i = 0; i < FBUS_SENSOR_MAX_FORWARD_BUFFERS; i++) {
#ifdef USE_FBUS_MASTER
        forwardBuffers[i].physicalId = fbusMasterConfig()->forwardedSensors[i];
#else
        forwardBuffers[i].physicalId = FBUS_INVALID_PHYSICAL_ID;
#endif
        forwardBuffers[i].writeIndex = 0;
        forwardBuffers[i].readIndex = 0;
        forwardBuffers[i].count = 0;
        forwardBuffers[i].startupFrameSent = false;
    }
}

bool fbusSensorIsForwarded(uint8_t physicalId)
{
    for (uint8_t i = 0; i < FBUS_SENSOR_MAX_FORWARD_BUFFERS; i++) {
        if (forwardBuffers[i].physicalId != FBUS_INVALID_PHYSICAL_ID && forwardBuffers[i].physicalId == physicalId) {
            return true;
        }
    }
    return false;
}

static fbusSensorForwardBuffer_t* getForwardBuffer(uint8_t physicalId)
{
    for (uint8_t i = 0; i < FBUS_SENSOR_MAX_FORWARD_BUFFERS; i++) {
        if (forwardBuffers[i].physicalId != FBUS_INVALID_PHYSICAL_ID && forwardBuffers[i].physicalId == physicalId) {
            return &forwardBuffers[i];
        }
    }
    return NULL;
}

static void addFrameToBuffer(uint8_t physicalId, uint16_t appId, uint32_t data)
{
    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (!buffer) {
        return;
    }
    
    // If buffer is full, discard oldest frame
    if (buffer->count >= FBUS_FORWARDED_FRAME_BUFFER_SIZE) {
        buffer->readIndex = (buffer->readIndex + 1) % FBUS_FORWARDED_FRAME_BUFFER_SIZE;
        buffer->count--;
    }
    
    buffer->frames[buffer->writeIndex].physicalId = physicalId;
    buffer->frames[buffer->writeIndex].appId = appId;
    buffer->frames[buffer->writeIndex].data = data;
    buffer->frames[buffer->writeIndex].valid = true;
    
    buffer->writeIndex = (buffer->writeIndex + 1) % FBUS_FORWARDED_FRAME_BUFFER_SIZE;
    buffer->count++;
}

bool fbusSensorPeekForwardedFrame(uint8_t physicalId, fbusSensorFrame_t *frame)
{
    if (!frame) {
        return false;
    }

    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (!buffer) {
        return false;
    }

    if (buffer->count == 0) {
        frame->physicalId = physicalId;
        frame->appId = 0;
        frame->data = 0;
        frame->valid = false;
        return true;
    }

    memcpy(frame, &buffer->frames[buffer->readIndex], sizeof(fbusSensorFrame_t));
    return true;
}

bool fbusSensorGetForwardedFrame(uint8_t physicalId, fbusSensorFrame_t *frame)
{
    if (!fbusSensorPeekForwardedFrame(physicalId, frame)) {
        return false;
    }

    if (!frame->valid) {
        return true;
    }

    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (!buffer) {
        return false;
    }

    buffer->readIndex = (buffer->readIndex + 1) % FBUS_FORWARDED_FRAME_BUFFER_SIZE;
    buffer->count--;

    return true;
}

bool fbusSensorNeedsStartupFrame(uint8_t physicalId)
{
    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (buffer) {
        return !buffer->startupFrameSent;
    }

    return false;
}

void fbusSensorMarkStartupFrameSent(uint8_t physicalId)
{
    fbusSensorForwardBuffer_t *buffer = getForwardBuffer(physicalId);
    if (buffer) {
        buffer->startupFrameSent = true;
    }
}

/*
 * GPS ADV Plus nav mode frames (receive interrupt). A block is HDR followed by the six data
 * frames of the same NAV-PVT; it is taken only when all six arrived after its header, so a lost
 * frame drops one 10 Hz solution instead of mixing two.
 */
static bool processNavFrame(uint8_t physicalId, uint16_t appId, uint32_t data, timeUs_t nowUs)
{
    uint8_t part = 0;

    switch (appId & 0xFFF0) {
    case FBUS_NAV_HDR_BASE:
        if (navOpen) {
            navDropped++;
        }
        memset(&navWork, 0, sizeof(navWork));
        navWork.fixType = (data >> 8) & 0x0F;
        navWork.fixOk = (data >> 12) & 1;
        navWork.numSat = (data >> 16) & 0x3F;
        navWork.ageMs = data >> 22;
        navWork.rxUs = nowUs;
        navParts = 0;
        navOpen = true;
        return true;
    case FBUS_NAV_LAT_BASE:
        navWork.lat = (int32_t)data;
        part = NAV_PART_LAT;
        break;
    case FBUS_NAV_LON_BASE:
        navWork.lon = (int32_t)data;
        part = NAV_PART_LON;
        break;
    case FBUS_NAV_VEL_BASE:
        navWork.velN = (int16_t)(data & 0xFFFF);
        navWork.velE = (int16_t)(data >> 16);
        part = NAV_PART_VEL;
        break;
    case FBUS_NAV_ACC_BASE:
        navWork.hAcc = data & 0xFFFF;
        navWork.sAcc = (data >> 16) & 0xFF;
        navWork.pdop10 = data >> 24;
        part = NAV_PART_ACC;
        break;
    case FBUS_NAV_ALT_BASE:
        navWork.altCm = (int32_t)data;
        part = NAV_PART_ALT;
        break;
    case FBUS_NAV_VZ_BASE:
        navWork.velD = (int16_t)(data & 0xFFFF);
        navWork.vAcc = data >> 16;
        part = NAV_PART_VZ;
        break;
    case FBUS_NAV_MAG_XY_BASE:
        magXY[0] = (int16_t)(data & 0xFFFF);
        magXY[1] = (int16_t)(data >> 16);
        magXYValid = true;
        return true;
    case FBUS_NAV_MAG_Z_BASE:
        if (magXYValid) {
            magFRD[0] = magXY[0];
            magFRD[1] = magXY[1];
            magFRD[2] = (int16_t)(data & 0xFFFF);
            magLastUs = nowUs;
            magXYValid = false;
        }
        return true;
    default:
        return false;
    }

    if (navOpen) {
        navParts |= part;
        if (navParts == NAV_PARTS_ALL) {
            navReady = navWork;
            navReadyNew = true;
            navOpen = false;
            navBlocks++;
            navLastUs = nowUs;
            navPhysId = physicalId;
        }
    }
    return true;
}

bool fbusSensorFindGps(uint8_t *physicalId, uint16_t *appId)
{
    for (uint8_t i = 0; i < observedSensorCount; i++) {
        const fbusObservedSensor_t *s = &observedSensors[i];
        if (s->source != FBUS_SENSOR_SOURCE_FBUS || s->detectedType != FBUS_DETECTED_SENSOR_GPS) {
            continue;
        }
        for (uint8_t k = 0; k < s->appIdCount; k++) {
            // a standard GPS App-ID: the sensor takes configuration frames for its own App-IDs
            if (s->appIds[k] >= FBUS_GPS_LATITUDE_BASE && s->appIds[k] <= FBUS_GPS_TIME_BASE + 0x0F) {
                *physicalId = s->physicalId;
                *appId = s->appIds[k];
                return true;
            }
        }
    }
    return false;
}

bool fbusSensorNavActive(void)
{
    const timeUs_t last = navLastUs;
    return last != 0 && cmpTimeUs(micros(), last) < NAV_TIMEOUT_US;
}

int fbusSensorNavPhysicalId(void)
{
    return fbusSensorNavActive() ? navPhysId : -1;
}

bool fbusSensorGetMag(int16_t mag[3])
{
    timeUs_t last;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        last = magLastUs;
        mag[0] = magFRD[0];
        mag[1] = magFRD[1];
        mag[2] = magFRD[2];
    }
    return last != 0 && cmpTimeUs(micros(), last) < NAV_MAG_TIMEOUT_US;
}

uint32_t fbusSensorNavBlocks(void)
{
    return navBlocks;
}

uint32_t fbusSensorNavDropped(void)
{
    return navDropped;
}

#ifdef USE_GPS
// A nav block into the GPS solution, as the u-blox driver fills it from NAV-PVT
static void fbusApplyNavSolution(const fbusNavSolution_t *s, timeUs_t currentTimeUs)
{
    gpsSol.llh.lat = s->lat;
    gpsSol.llh.lon = s->lon;
    gpsSol.llh.altCm = s->altCm;
    gpsSol.numSat = s->numSat;
    gpsSol.hdop = s->pdop10 * 10;
    gpsSol.velN = s->velN;
    gpsSol.velE = s->velE;
    gpsSol.velNEValid = s->fixOk;
    // 0 means "not reported" to the estimator: the sensor rounds down to whole cm
    gpsSol.hAcc = MAX(s->hAcc, 1);
    gpsSol.vAcc = MAX(s->vAcc, 1);
    gpsSol.sAcc = MAX(s->sAcc, 1);

    const float groundSpeed = sqrtf(sq((float)s->velN) + sq((float)s->velE));
    gpsSol.groundSpeed = lrintf(groundSpeed);
    gpsSol.speed3d = lrintf(sqrtf(sq(groundSpeed) + sq((float)s->velD)));
    int course = lrintf(atan2_approx(s->velE, s->velN) * (1800.0f / M_PIf));
    if (course < 0) {
        course += 3600;
    }
    gpsSol.groundCourse = course % 3600;

    // Time from the NAV-PVT reaching the sensor until now
    gpsSol.transportDelayMs = s->ageMs + cmpTimeUs(currentTimeUs, s->rxUs) / 1000;

    gpsSetFixState(s->fixOk && s->fixType == 3);
    gpsData.lastMessage = millis();
    GPS_update |= GPS_MSP_UPDATE;
}
#endif

bool fbusSensorProcessDataWithSource(uint8_t physicalId, uint16_t appId, uint32_t data, fbusSensorSource_e source)
{
    timeUs_t currentTimeUs = micros();
    fbusObservedSensor_t *observed = trackObservedSensor(physicalId, appId, currentTimeUs, source);
    const fbusDetectedSensorType_e detectedType = observed ? observed->detectedType : FBUS_DETECTED_SENSOR_UNKNOWN;
    
    // Store in cache
    sensorCache[sensorCacheIndex].physicalId = physicalId;
    sensorCache[sensorCacheIndex].appId = appId;
    sensorCache[sensorCacheIndex].data = data;
    sensorCache[sensorCacheIndex].valid = true;
    sensorCache[sensorCacheIndex].lastUpdateUs = currentTimeUs;
    sensorCacheIndex = (sensorCacheIndex + 1) % FBUS_SENSOR_CACHE_SIZE;

    if (fbusSensorIsForwarded(physicalId)) {
        addFrameToBuffer(physicalId, appId, data);
    }

    if (processNavFrame(physicalId, appId, data, currentTimeUs)) {
        return true;
    }

    if (detectedType == FBUS_DETECTED_SENSOR_GPS) {
        if (appId >= FBUS_GPS_LATITUDE_BASE && appId <= (FBUS_GPS_LATITUDE_BASE + 0x0F)) {
            if (data & FBUS_GPS_LAT_LON_BIT31) {
                fbusGps.longitude = fbusGpsConvertLatLon(data);
            } else {
                fbusGps.latitude = fbusGpsConvertLatLon(data);
            }
            fbusGps.hasPosition = true;
            fbusGps.lastUpdateUs = currentTimeUs;  

        } else if (appId >= FBUS_GPS_ALTITUDE_BASE && appId <= (FBUS_GPS_ALTITUDE_BASE + 0x0F)) {
            fbusGps.altitudeCm = fbusGpsConvertAltitude(data);
            fbusGps.hasAltitude = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_SPEED_BASE && appId <= (FBUS_GPS_SPEED_BASE + 0x0F)) {
            fbusGps.speedMilliKnots = data;
            fbusGps.hasSpeed = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_COURSE_BASE && appId <= (FBUS_GPS_COURSE_BASE + 0x0F)) {
            fbusGps.courseDeg = fbusGpsConvertCourse(data);
            fbusGps.hasCourse = true;
            fbusGps.lastUpdateUs = currentTimeUs;
            
        } else if (appId >= FBUS_GPS_TIME_BASE && appId <= (FBUS_GPS_TIME_BASE + 0x0F)) {
            uint8_t type = (data >> 24) & 0xFF;
            if (type == FBUS_GPS_TIME_TYPE_TIME) {
                fbusGps.hasTime = fbusGpsConvertTime(data, &fbusGps.time);
            } else if (type == FBUS_GPS_TIME_TYPE_DATE) {
                fbusGps.hasDate = fbusGpsConvertDate(data, &fbusGps.date);
            }
            fbusGps.lastUpdateUs = currentTimeUs;
        }
        
        return true;
    }
    
    if (detectedType == FBUS_DETECTED_SENSOR_XACT_SERVO) {
        if (appId >= FBUS_SERVO_DATA_BASE && appId <= (FBUS_SERVO_DATA_BASE + 0x0F)) {
#ifdef USE_FBUS_MASTER
            // Only the FBUS master link can send XACT read/write frames, so servo
            // programming tracking is scoped to sensors observed on that source.
            if (source == FBUS_SENSOR_SOURCE_FBUS) {
                fbusXactTrackServo(physicalId, appId, currentTimeUs);
            }
#endif
            fbusServoConvertData(data, &fbusServo.currentDeciAmps,
                                &fbusServo.voltageDeciVolts,
                                &fbusServo.temperatureDegC);
            fbusServo.hasData = true;
            fbusServo.lastUpdateUs = currentTimeUs;
        }
        
        return true;
    }

    if (detectedType == FBUS_DETECTED_SENSOR_FAS_150S) {
        if (appId >= FBUS_CURRENT_BASE && appId <= (FBUS_CURRENT_BASE + 0x0F)) {
            fbusCurrent.currentDeciAmps = data;
            fbusCurrent.hasCurrent = true;
            fbusCurrent.lastUpdateUs = currentTimeUs;
            return true;
        }

        if (appId >= FBUS_VOLTAGE_BASE && appId <= (FBUS_VOLTAGE_BASE + 0x0F)) {
            fbusCurrent.voltageCentiVolts = data;
            fbusCurrent.hasVoltage = true;
            fbusCurrent.lastUpdateUs = currentTimeUs;
            return true;
        }

        if (appId >= FBUS_HIGH_PREC_CURRENT_BASE && appId <= (FBUS_HIGH_PREC_CURRENT_BASE + 0x0F)) {
            fbusCurrent.currentMilliAmps = (uint16_t)(data & 0xFFFF);
            fbusCurrent.hasHighPrecisionCurrent = true;
            fbusCurrent.lastUpdateUs = currentTimeUs;
            return true;
        }

        return false;
    }

    if (detectedType == FBUS_DETECTED_SENSOR_FLVSS) {
        if (appId >= FBUS_FLVSS_VOLTAGE_BASE && appId <= (FBUS_FLVSS_VOLTAGE_BASE + 0x0F)) {
            const uint8_t cellBaseIndex = data & 0x0FU;
            const uint8_t totalCellCount = (data >> 4) & 0x0FU;
            const uint16_t firstCellRaw = (data >> 8) & 0x0FFFU;
            const uint16_t secondCellRaw = (data >> 20) & 0x0FFFU;

            if (cellBaseIndex >= FBUS_FLVSS_MAX_CELLS || (cellBaseIndex & 0x01U) || totalCellCount == 0 || totalCellCount > FBUS_FLVSS_MAX_CELLS) {
                return false;
            }

            if (fbusFlvss.cellCount != totalCellCount) {
                memset(fbusFlvss.cellVoltageCentiVolts, 0, sizeof(fbusFlvss.cellVoltageCentiVolts));
            }

            fbusFlvss.cellCount = totalCellCount;
            fbusFlvss.cellVoltageCentiVolts[cellBaseIndex] = fbusFlvssConvertCellVoltage(firstCellRaw);

            if ((cellBaseIndex + 1U) < totalCellCount) {
                fbusFlvss.cellVoltageCentiVolts[cellBaseIndex + 1U] = fbusFlvssConvertCellVoltage(secondCellRaw);
            }

            fbusFlvss.lastUpdateUs = currentTimeUs;
            fbusFlvssUpdatePackVoltage();
            return true;
        }

        return false;
    }

    if (detectedType == FBUS_DETECTED_SENSOR_ESC) {
        // ESC_POWER 0x0B50~0x0B5F:
        // bits 0..15 voltage (V/100), bits 16..31 current (A/100)
        if (appId >= FBUS_ESC_POWER_BASE && appId <= (FBUS_ESC_POWER_BASE + 0x0F)) {
            fbusEsc.voltageCentiVolts = (uint16_t)(data & 0xFFFFU);
            fbusEsc.currentCentiAmps = (uint16_t)((data >> 16) & 0xFFFFU);
            fbusEsc.hasPower = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }

        // ESC_R&C 0x0B60~0x0B6F:
        // bits 0..15 ERPM/100, bits 16..31 consumption (mAh)
        if (appId >= FBUS_ESC_RPM_CONS_BASE && appId <= (FBUS_ESC_RPM_CONS_BASE + 0x0F)) {
            fbusEsc.erpm = (uint32_t)(data & 0xFFFFU) * 100U;
            fbusEsc.consumptionMah = (uint16_t)((data >> 16) & 0xFFFFU);
            fbusEsc.hasRpm = true;
            fbusEsc.hasConsumption = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }

        // ESC_TEMP 0x0B70~0x0B7F:
        // bits 0..7 temperature (C)
        if (appId >= FBUS_ESC_TEMP_BASE && appId <= (FBUS_ESC_TEMP_BASE + 0x0F)) {
            fbusEsc.temperatureDegC = (uint8_t)(data & 0xFFU);
            fbusEsc.hasTemperature = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }

        return false;
    }

    if (detectedType == FBUS_DETECTED_SENSOR_RPM) {
        if (appId >= FBUS_RPM_TEMP1_BASE && appId <= (FBUS_RPM_TEMP1_BASE + 0x0F)) {
            fbusEsc.temperatureDegC = (uint8_t)(data & 0xFFU);
            fbusEsc.hasTemperature = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }
        if (appId >= FBUS_RPM_TEMP2_BASE && appId <= (FBUS_RPM_TEMP2_BASE + 0x0F)) {
            fbusEsc.temperature2DegC = (uint8_t)(data & 0xFFU);
            fbusEsc.hasTemperature = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }
        if (appId >= FBUS_RPM_BASE && appId <= (FBUS_RPM_BASE + 0x0F)) {
            // Sensor goes as high as 300k RPM, we need 19 bits for that.
            fbusEsc.erpm = (uint32_t)(data & 0x7FFFFU);
            fbusEsc.hasRpm = true;
            fbusEsc.lastUpdateUs = currentTimeUs;
            return true;
        }
        return false;
    }
    
    return false;
}

bool fbusSensorProcessData(uint8_t physicalId, uint16_t appId, uint32_t data)
{
    return fbusSensorProcessDataWithSource(physicalId, appId, data, FBUS_SENSOR_SOURCE_FBUS);
}

void fbusSensorObservePhysicalIdWithSource(uint8_t physicalId, fbusSensorSource_e source)
{
    fbusObservedSensor_t *sensor = ensureObservedSensor(physicalId, source);
    if (sensor) {
        sensor->lastSeenUs = micros();
    }
}

void fbusSensorObservePhysicalId(uint8_t physicalId)
{
    fbusSensorObservePhysicalIdWithSource(physicalId, FBUS_SENSOR_SOURCE_FBUS);
}

void fbusSensorUpdate(timeUs_t currentTimeUs)
{
    if (fbusGps.lastUpdateUs > 0 && (currentTimeUs - fbusGps.lastUpdateUs) > 1000000) {
        fbusGps.hasPosition = false;
        fbusGps.hasAltitude = false;
        fbusGps.hasSpeed = false;
        fbusGps.hasCourse = false;
        fbusGps.hasTime = false;
        fbusGps.hasDate = false;
    }
    
    if (fbusServo.lastUpdateUs > 0 && (currentTimeUs - fbusServo.lastUpdateUs) > 1000000) {
        fbusServo.hasData = false;
    }

    if (fbusCurrent.lastUpdateUs > 0 && (currentTimeUs - fbusCurrent.lastUpdateUs) > 1000000) {
        fbusCurrent.hasCurrent = false;
        fbusCurrent.hasVoltage = false;
        fbusCurrent.hasHighPrecisionCurrent = false;
    }

    if (fbusEsc.lastUpdateUs > 0 && (currentTimeUs - fbusEsc.lastUpdateUs) > 1000000) {
        fbusEsc.hasPower = false;
        fbusEsc.hasRpm = false;
        fbusEsc.hasConsumption = false;
        fbusEsc.hasTemperature = false;
    }

    if (fbusFlvss.lastUpdateUs > 0 && (currentTimeUs - fbusFlvss.lastUpdateUs) > 1000000) {
        memset(fbusFlvss.cellVoltageCentiVolts, 0, sizeof(fbusFlvss.cellVoltageCentiVolts));
        fbusFlvss.voltageCentiVolts = 0;
        fbusFlvss.cellCount = 0;
        fbusFlvss.hasVoltage = false;
    }

#ifdef USE_GPS
    if (gpsUsesFbusTransport() && fbusSensorNavActive()) {
        // GPS ADV Plus nav mode: one GPS message per complete 10 Hz block
        if (navReadyNew) {
            fbusNavSolution_t solution;
            ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                solution = navReady;
                navReadyNew = false;
            }
            fbusApplyNavSolution(&solution, currentTimeUs);
        }
    }
    else if (gpsUsesFbusTransport()) {
        // Standard GPS App-IDs: no velocity vector or accuracies
        gpsSol.velNEValid = false;
        gpsSol.hAcc = gpsSol.vAcc = gpsSol.sAcc = 0;
        gpsSol.transportDelayMs = 0;

        if (fbusGps.hasPosition) {
            // One GPS message per new frame, not per call
            if (fbusGps.lastUpdateUs == legacyGpsConsumedUs) {
                return;
            }
            legacyGpsConsumedUs = fbusGps.lastUpdateUs;
            gpsSol.llh.lat = fbusGps.latitude;
            gpsSol.llh.lon = fbusGps.longitude;

            if (fbusGps.hasAltitude) {
                gpsSol.llh.altCm = fbusGps.altitudeCm;
            }

            if (fbusGps.hasSpeed) {
                gpsSol.groundSpeed = fbusGpsConvertSpeed(fbusGps.speedMilliKnots);
                gpsSol.speed3d = gpsSol.groundSpeed;
            }

            if (fbusGps.hasCourse) {
                gpsSol.groundCourse = fbusGps.courseDeg;
            }

            gpsSol.numSat = 5;
            gpsSol.hdop = 100;
            gpsData.lastMessage = millis();
            gpsSetFixState(true);
            GPS_update |= GPS_MSP_UPDATE;
        } else {
            sensorsClear(SENSOR_GPS);
            gpsSetFixState(false);
            gpsSol.numSat = 0;
        }
    }
#endif
}

void fbusSensorGetGpsData(fbusGpsData_t *gpsData)
{
    if (gpsData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(gpsData, &fbusGps, sizeof(fbusGpsData_t));
        }
    }
}

bool fbusSensorHasGpsData(void)
{
    timeUs_t currentTimeUs = micros();
    
    if (fbusGps.hasPosition && fbusGps.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusGps.lastUpdateUs) < 1000000) {
            return true;
        }
    }
    
    return false;
}

void fbusSensorGetServoData(fbusServoData_t *servoData)
{
    if (servoData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(servoData, &fbusServo, sizeof(fbusServoData_t));
        }
    }
}

bool fbusSensorHasServoData(void)
{
    timeUs_t currentTimeUs = micros();
    
    if (fbusServo.hasData && fbusServo.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusServo.lastUpdateUs) < 1000000) {
            return true;
        }
    }
    
    return false;
}

void fbusSensorGetCurrentData(fbusCurrentData_t *currentData)
{
    if (currentData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(currentData, &fbusCurrent, sizeof(fbusCurrentData_t));
        }
    }
}

bool fbusSensorHasCurrentData(void)
{
    timeUs_t currentTimeUs = micros();

    if ((fbusCurrent.hasCurrent || fbusCurrent.hasVoltage || fbusCurrent.hasHighPrecisionCurrent)
        && fbusCurrent.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusCurrent.lastUpdateUs) < 1000000) {
            return true;
        }
    }

    return false;
}

bool fbusSensorGetBatteryVoltageCentiVolts(uint32_t *voltageCentiVolts)
{
    const timeUs_t currentTimeUs = micros();
    const bool hasCurrentVoltage = fbusCurrent.hasVoltage
        && fbusCurrent.lastUpdateUs > 0
        && (currentTimeUs - fbusCurrent.lastUpdateUs) < 1000000;
    const bool hasFlvssVoltage = fbusFlvss.hasVoltage
        && fbusFlvss.lastUpdateUs > 0
        && (currentTimeUs - fbusFlvss.lastUpdateUs) < 1000000;

    if (voltageCentiVolts) {
        *voltageCentiVolts = hasCurrentVoltage ? fbusCurrent.voltageCentiVolts
            : (hasFlvssVoltage ? fbusFlvss.voltageCentiVolts : 0);
    }

    return hasCurrentVoltage || hasFlvssVoltage;
}

void fbusSensorGetEscData(fbusEscData_t *escData)
{
    if (escData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(escData, &fbusEsc, sizeof(fbusEscData_t));
        }
    }
}

bool fbusSensorHasEscData(void)
{
    timeUs_t currentTimeUs = micros();

    if ((fbusEsc.hasPower || fbusEsc.hasRpm || fbusEsc.hasConsumption || fbusEsc.hasTemperature )
        && fbusEsc.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusEsc.lastUpdateUs) < 1000000) {
            return true;
        }
    }

    return false;
}

void fbusSensorGetFlvssData(fbusFlvssData_t *flvssData)
{
    if (flvssData) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            memcpy(flvssData, &fbusFlvss, sizeof(fbusFlvssData_t));
        }
    }
}

bool fbusSensorHasFlvssData(void)
{
    timeUs_t currentTimeUs = micros();

    if (fbusFlvss.hasVoltage && fbusFlvss.lastUpdateUs > 0) {
        if ((currentTimeUs - fbusFlvss.lastUpdateUs) < 1000000) {
            return true;
        }
    }

    return false;
}

uint8_t fbusSensorGetObservedCount(void)
{
    return observedSensorCount;
}

const fbusObservedSensor_t* fbusSensorGetObserved(uint8_t index)
{
    if (index < observedSensorCount) {
        return &observedSensors[index];
    }
    return NULL;
}

void fbusSensorClearObserved(void)
{
    memset(observedSensors, 0, sizeof(observedSensors));
    observedSensorCount = 0;
}

const char* fbusSensorGetName(uint8_t physicalId)
{
    fbusObservedSensor_t *observed = getObservedSensorByPhysicalId(physicalId);
    if (observed) {
        switch (observed->detectedType) {
            case FBUS_DETECTED_SENSOR_GPS:
                return "GPS";
            case FBUS_DETECTED_SENSOR_ESC:
                return "ESC";
            case FBUS_DETECTED_SENSOR_FAS_150S:
                return "FAS_150S";
            case FBUS_DETECTED_SENSOR_FLVSS:
                return "FLVSS";
            case FBUS_DETECTED_SENSOR_XACT_SERVO:
                return "XACT_SERVO";
            case FBUS_DETECTED_SENSOR_RPM:
                return "RPM";
            case FBUS_DETECTED_SENSOR_UNKNOWN:
            default:
                break;
        }
    }

    if (physicalId < sizeof(fbusSensorNames) / sizeof(fbusSensorNames[0]) && fbusSensorNames[physicalId]) {
        return fbusSensorNames[physicalId];
    }
    return "UNKNOWN";
}

const char* fbusSensorGetSourceName(fbusSensorSource_e source)
{
    switch (source) {
        case FBUS_SENSOR_SOURCE_SPORT:
            return "SPORT";
        case FBUS_SENSOR_SOURCE_FBUS:
            return "FBUS";
        default:
            return "UNKNOWN";
    }
}

#endif // USE_FBUS_MASTER || USE_SPORT_MASTER
