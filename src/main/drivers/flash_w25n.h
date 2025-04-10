/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: jflyper
 */

#pragma once

#define JEDEC_ID_WINBOND_W25N01GV    0xEFAA21
#define JEDEC_ID_WINBOND_W25N01KV    0xEFAE21
#define JEDEC_ID_WINBOND_W25N02KV    0xEFAA22

bool w25n_detect(flashDevice_t *fdevice, uint32_t jedecID);
