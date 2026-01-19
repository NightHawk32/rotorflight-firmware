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

#include "types.h"
#include "platform.h"

#ifdef USE_OPTICAL_FLOW

#include "pg/pg_ids.h"
#include "pg/optical_flow.h"

PG_REGISTER_WITH_RESET_TEMPLATE(opticalFlowConfig_t, opticalFlowConfig, PG_OPTICAL_FLOW_CONFIG, 0);

PG_RESET_TEMPLATE(opticalFlowConfig_t, opticalFlowConfig,
    .optical_flow_hardware = OPTICAL_FLOW_MICROLINK,
);

#endif