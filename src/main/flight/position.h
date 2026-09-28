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

#include <stdbool.h>
#include <stdint.h>

void positionInit(void);
void positionUpdate(void);

float getAltitude(void);
float getVario(void);

int getEstimatedAltitudeCm(void);
int getEstimatedVarioCms(void);

// False when the Z estimate is stale; getAltitude() then returns 0.
bool isAltitudeValid(void);

// Fused Kalman altitude (m, arm-point frame), vario (m/s) and 1-sigma
// altitude uncertainty (m).  Returns false when the estimate is stale.
bool getAltitudeEstimate(float *altitudeM, float *varioMs, float *stdDevM);

// Baro downwash model state (for diagnostics)
float getBaroBias(void);
float getBaroDisturbance(void);

// AGL altitude from rangefinder (meters)
#ifdef USE_RANGEFINDER
float getAGLAltitude(void);
float getAGLVario(void);
bool  isAGLAltitudeValid(void);
float getAGLReliability(void);
#endif

// Snapshot of the estimator for MSP (configurator live view)
#define POS_STATUS_ALT_VALID        (1 << 0)    // general altitude output valid
#define POS_STATUS_KF_VALID         (1 << 1)    // Z Kalman filter valid
#define POS_STATUS_AGL_VALID        (1 << 2)
#define POS_STATUS_XY_VALID         (1 << 3)
#define POS_STATUS_ANCHOR_FRESH     (1 << 4)    // baro bias observable
#define POS_STATUS_INVERTED         (1 << 5)    // inverted-thrust baro bias active
#define POS_STATUS_BARO_GATED       (1 << 6)    // last baro sample rejected
#define POS_STATUS_HAVE_BARO        (1 << 7)
#define POS_STATUS_HAVE_GPS_ALT     (1 << 8)
#define POS_STATUS_XY_GPS_FRESH     (1 << 9)
#define POS_STATUS_XY_FLOW_FRESH    (1 << 10)
#define POS_STATUS_XY_CLAMPED       (1 << 11)
#define POS_STATUS_XY_GPS_ORIGIN    (1 << 12)
#define POS_STATUS_RANGEFINDER      (1 << 13)   // rangefinder detected
#define POS_STATUS_FLOW             (1 << 14)   // optical flow detected
#define POS_STATUS_FLOW_HEALTHY     (1 << 15)

typedef struct positionStatus_s {
    uint16_t    flags;              // POS_STATUS_*
    float       altitudeCm;         // general output (getAltitude())
    float       kfAltCm;
    float       kfVarioCms;
    float       kfSigmaCm;
    float       baroBiasCm;
    float       disturbance;
    float       baroMeasCm;         // measurements as last fused, KF frame
    float       gpsMeasCm;
    float       rfMeasCm;
    float       aglAltCm;
    float       aglVarioCms;
    float       aglReliability;     // 0..1
    int32_t     rangefinderRawCm;
    int16_t     flowX;              // raw, cm/s @ 1m
    int16_t     flowY;
    uint8_t     flowQuality;
    uint8_t     flowStatus;         // see DEBUG_OPTICAL_FLOW[7]
    float       posEastCm;
    float       posNorthCm;
    float       velEastCms;
    float       velNorthCms;
    float       posSigmaCm;
    float       flowVelEastCms;
    float       flowVelNorthCms;
} positionStatus_t;

void positionGetStatus(positionStatus_t *status);

// Optical-flow dead-reckoning position (cm) and velocity (cm/s)
#ifdef USE_OPTICAL_FLOW
float getPositionXCm(void);
float getPositionYCm(void);
float getVelocityXCms(void);
float getVelocityYCms(void);
bool  isPositionXYValid(void);
#endif
