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
 */

#pragma once

#include <stdbool.h>

// Drives the external TX/RX tri-state buffers used on the shared, invert-capable
// RX serial pin found on boards like the XLPower F4 Mini. Needed on boards where
// the MCU has no built-in UART inverter and direction arbitration for half-duplex
// protocols (FPort/FBUS) is done externally instead of via USART half-duplex mode.
//
// Pin assignment is done at runtime via the `resource RX_UART_TX_EN`/
// `resource RX_UART_RX_EN` unified-target config commands (see pg/rx.h), not via
// compile-time target.h macros. This lets USE_RX_UART_PINIO be compiled into a
// shared unified-target binary (e.g. STM32_UNIFIED) without affecting any other
// board's config: rxUartPinioIsConfigured() returns false unless a board's own
// .config explicitly assigns both pins.
#ifdef USE_RX_UART_PINIO

void rxUartPinioInit(void);
bool rxUartPinioIsConfigured(void);
void rxUartPinioSetDirection(bool txEnable, bool rxEnable);

#else

static inline void rxUartPinioInit(void) {}
static inline bool rxUartPinioIsConfigured(void) { return false; }
static inline void rxUartPinioSetDirection(bool txEnable, bool rxEnable) { (void)txEnable; (void)rxEnable; }

#endif
