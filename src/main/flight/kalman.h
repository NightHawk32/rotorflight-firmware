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
 *
 * Ported from Betaflight's flight/position_filter.c (GPLv3).
 */

#pragma once

#include <stdbool.h>

// 2-state Kalman filter for one axis: [position, velocity]
// Driven by acceleration control input, corrected by position or velocity measurements.
typedef struct positionKalman_s {
    float x[2];      // state: [0]=position (cm), [1]=velocity (cm/s)
    float P[2][2];   // error covariance
    float Q_accel;   // process noise: accelerometer variance (cm/s^2)^2
} positionKalman_t;

void kalmanInit(positionKalman_t *kf, float initialPos, float initialVel, float initialPosVar, float initialVelVar, float qAccel);
void kalmanPredict(positionKalman_t *kf, float dt, float accel);
void kalmanUpdatePosition(positionKalman_t *kf, float measuredPos, float R);
void kalmanUpdateVelocity(positionKalman_t *kf, float measuredVel, float R);

static inline float kalmanGetPosition(const positionKalman_t *kf) { return kf->x[0]; }
static inline float kalmanGetVelocity(const positionKalman_t *kf) { return kf->x[1]; }
static inline float kalmanGetPositionVariance(const positionKalman_t *kf) { return kf->P[0][0]; }
static inline float kalmanGetVelocityVariance(const positionKalman_t *kf) { return kf->P[1][1]; }


// 4-state vertical Kalman filter:
//   [altitude, vertical velocity, baro bias, terrain offset]
//
// The barometer on a helicopter sits in the rotor downwash, so its reading is
// offset by a pressure error that depends on rotor thrust (collective,
// headspeed) and flips character when the thrust direction reverses
// (inverted flight).  Modelling that error as its own random-walk state lets
// the non-drifting sources (GPS altitude, GPS Doppler velocity, rangefinder)
// estimate it continuously, instead of the baro dragging the altitude around.
//
// The rangefinder measures height above the ground, not altitude in the arm
// frame.  The terrain offset state is the ground height under the model
// relative to the arm point (after the one-time alignment), so a hill or a
// table edge moves the terrain state instead of the altitude.  It is a
// random walk whose process noise the caller scales with horizontal speed:
// hovering over one spot the ground cannot change, so the rangefinder is a
// firm altitude anchor there.
//
// Measurement models (H):
//   baro             [1, 0, 1, 0]   altitude + bias
//   GPS altitude     [1, 0, 0, 0]
//   GPS velocity     [0, 1, 0, 0]
//   rangefinder      [1, 0, 0, -1]  altitude - terrain
#define ALT_KF_STATES   4

typedef struct altitudeKalman_s {
    float x[ALT_KF_STATES];                 // [0]=altitude (cm), [1]=vertical velocity (cm/s),
                                            // [2]=baro bias (cm), [3]=terrain offset (cm)
    float P[ALT_KF_STATES][ALT_KF_STATES];  // error covariance
    float Q_accel;   // process noise: accelerometer variance (cm/s^2)^2
    float Q_bias;    // process noise: baro bias random walk (cm^2/s)
} altitudeKalman_t;

void altKalmanInit(altitudeKalman_t *kf, float initialPosVar, float initialVelVar, float initialBiasVar,
                   float initialTerrainVar, float qAccel, float qBias);
// biasNoiseScale multiplies Q_bias*dt; terrainNoise is the variance (cm^2)
// added to the terrain state in this step.
void altKalmanPredict(altitudeKalman_t *kf, float dt, float accel, float biasNoiseScale, float terrainNoise);
// Scalar update with a measurement row H. With gateSigma > 0 an innovation
// larger than gateSigma standard deviations is rejected; returns false then.
// innovation (optional) receives measurement - predicted measurement.
bool altKalmanUpdate(altitudeKalman_t *kf, const float H[ALT_KF_STATES], float measurement, float R,
                     float gateSigma, float *innovation);
// Reset one state to a value and variance, removing its cross-covariances.
void altKalmanResetState(altitudeKalman_t *kf, int state, float value, float variance);
// Drop the cross-covariances of one state and cap its variance, so that a
// measurement can no longer move it much (used to freeze the bias when no
// anchor sensor makes it observable).
void altKalmanDecoupleState(altitudeKalman_t *kf, int state, float maxVariance);

static inline float altKalmanGetAltitude(const altitudeKalman_t *kf) { return kf->x[0]; }
static inline float altKalmanGetVelocity(const altitudeKalman_t *kf) { return kf->x[1]; }
static inline float altKalmanGetBias(const altitudeKalman_t *kf) { return kf->x[2]; }
static inline float altKalmanGetTerrain(const altitudeKalman_t *kf) { return kf->x[3]; }
static inline float altKalmanGetAltitudeVariance(const altitudeKalman_t *kf) { return kf->P[0][0]; }
static inline float altKalmanGetBiasVariance(const altitudeKalman_t *kf) { return kf->P[2][2]; }
static inline float altKalmanGetTerrainVariance(const altitudeKalman_t *kf) { return kf->P[3][3]; }
