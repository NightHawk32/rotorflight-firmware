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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/position.h"

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 1);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .alt_source = ALT_SOURCE_DEFAULT,
    .baro_alt_lpf = 100,
    .baro_offset_lpf = 5,
    .gps_alt_lpf = 25,
    .gps_offset_lpf = 5,
    .gps_min_sats = 12,
    .vario_lpf = 50,
    // Estimator defaults ported from Betaflight position_estimator.c,
    // tuned for a 100Hz fusion rate (ESTIMATOR_PERIOD_US in flight/position.c).
    .xy_source = XY_SOURCE_AUTO,
    .est_q_accel_xy = 50000,
    .est_q_accel_z = 20000,
    .est_r_baro_alt = 1500,
    .est_r_rangefinder_alt = 100,
    .est_r_gps_pos = 500,
    .est_r_gps_vel = 100,
    .est_r_flow_vel = 400,
);
