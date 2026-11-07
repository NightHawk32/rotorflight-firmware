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

/*
 * Compass of a GPS ADV Plus in nav mode, received over FBUS (or S.Port) as NAV_MAG_XY/NAV_MAG_Z
 * at 20 Hz. The sensor sends its field in 0.1 uT after its own calibration and mounting
 * rotation, in its board frame x forward, y right, z down. The IMU works in x forward, y left,
 * z up, so y and z change sign here. align_mag and mag_calibration apply on top as for any
 * compass.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_MAG) && (defined(USE_FBUS_MASTER) || defined(USE_SPORT_MASTER))

#include "drivers/fbus_sensor.h"

#include "compass_fbus.h"

static bool fbusMagInit(magDev_t *mag)
{
    UNUSED(mag);
    return true;
}

static bool fbusMagRead(magDev_t *mag, int16_t *magData)
{
    UNUSED(mag);
    int16_t frd[3];
    fbusSensorGetMag(frd);

    magData[0] = frd[0];
    magData[1] = -frd[1];
    magData[2] = -frd[2];
    return true;
}

bool fbusMagIsFresh(void)
{
    int16_t frd[3];
    return fbusSensorGetMag(frd);
}

bool fbusMagDetect(magDev_t *mag)
{
    // The data arrives later, once the FBUS master has found the sensor; until then and
    // whenever it stops, compassIsHealthy() reports the compass as not healthy.
    mag->init = fbusMagInit;
    mag->read = fbusMagRead;
    return true;
}

#endif
