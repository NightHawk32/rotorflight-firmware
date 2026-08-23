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

#include <stdbool.h>

#include "platform.h"

#ifdef USE_RX_UART_PINIO

#include "drivers/io.h"
#include "drivers/resource.h"

#include "pg/rx.h"

#include "rx_uart_pinio.h"

// Pin assignment is fully runtime-configurable via the `resource RX_UART_TX_EN`
// and `resource RX_UART_RX_EN` unified-target config commands (see pg/rx.h and
// the resourceTable entries in cli.c). This keeps the feature inert (IO_NONE)
// on every existing target/config that doesn't explicitly assign these pins.
static IO_t txEnPin = IO_NONE;
static IO_t rxEnPin = IO_NONE;

void rxUartPinioInit(void)
{
    txEnPin = IOGetByTag(rxConfig()->rxUartTxEnIoTag);
    rxEnPin = IOGetByTag(rxConfig()->rxUartRxEnIoTag);

    if (txEnPin) {
        IOInit(txEnPin, OWNER_RX_UART_TX_EN, 0);
        IOConfigGPIO(txEnPin, IOCFG_OUT_PP);
    }
    if (rxEnPin) {
        IOInit(rxEnPin, OWNER_RX_UART_RX_EN, 0);
        IOConfigGPIO(rxEnPin, IOCFG_OUT_PP);
    }

    // Safe default until a protocol driver claims a direction: both buffers off.
    rxUartPinioSetDirection(false, false);
}

bool rxUartPinioIsConfigured(void)
{
    return txEnPin != IO_NONE && rxEnPin != IO_NONE;
}

void rxUartPinioSetDirection(bool txEnable, bool rxEnable)
{
    if (txEnPin) {
        IOWrite(txEnPin, txEnable);
    }
    if (rxEnPin) {
        IOWrite(rxEnPin, rxEnable);
    }
}

#endif
