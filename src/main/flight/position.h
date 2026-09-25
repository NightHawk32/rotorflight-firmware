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

// Optical-flow dead-reckoning position (cm) and velocity (cm/s)
#ifdef USE_OPTICAL_FLOW
float getPositionXCm(void);
float getPositionYCm(void);
float getVelocityXCms(void);
float getVelocityYCms(void);
bool  isPositionXYValid(void);
#endif
