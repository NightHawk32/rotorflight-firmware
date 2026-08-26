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
#include <stdbool.h>

enum {
    ALT_SOURCE_DEFAULT = 0,
    ALT_SOURCE_BARO_ONLY,
    ALT_SOURCE_GPS_ONLY,
    ALT_SOURCE_LIDAR_ONLY,
};

enum {
    XY_SOURCE_AUTO = 0,     // fuse GPS and optical flow, whichever is healthy
    XY_SOURCE_GPS_ONLY,
    XY_SOURCE_FLOW_ONLY,
};

typedef struct positionConfig_s {
    uint8_t alt_source;
    uint8_t baro_alt_lpf;
    uint8_t baro_offset_lpf;
    uint8_t gps_alt_lpf;
    uint8_t gps_offset_lpf;
    uint8_t gps_min_sats;
    uint8_t vario_lpf;

    // State estimator (per-axis position/velocity Kalman filters)
    uint8_t  xy_source;         // XY_SOURCE_*: which sensors feed the horizontal estimate
    uint16_t est_q_accel_xy;    // process noise: horizontal accel variance (cm/s^2)^2
    uint16_t est_q_accel_z;     // process noise: vertical accel variance (cm/s^2)^2
    uint16_t est_r_baro_alt;    // measurement noise: baro altitude (cm^2)
    uint16_t est_r_rangefinder_alt; // measurement noise: rangefinder altitude (cm^2)
    uint16_t est_r_gps_pos;     // measurement noise: GPS position (cm^2) at HDOP 1.0
    uint16_t est_r_gps_vel;     // measurement noise: GPS velocity ((cm/s)^2) at HDOP 1.0
    uint16_t est_r_flow_vel;    // measurement noise: optical flow velocity ((cm/s)^2) at max quality
} positionConfig_t;

PG_DECLARE(positionConfig_t, positionConfig);

