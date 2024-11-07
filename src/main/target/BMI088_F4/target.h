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
#define TARGET_BOARD_IDENTIFIER "BMI8"
#define USBD_PRODUCT_STRING     "BMI088_F4"
#define USE_TARGET_CONFIG
/*--------------LED----------------*/
#define LED0_PIN                PB5
#define LED1_PIN                PB6
/*---------------------------------*/

/*------------BEEPER---------------*/
#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED
/*---------------------------------*/

/*------------SENSORS--------------*/
// MPU interrupt
#define GYRO_1_EXTI_PIN         PA15
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_SPI_GYRO
#define USE_GYRO
#define BMI088_CS_A_PIN         PB10
#define BMI088_CS_G_PIN         PA8
#define USE_ACCGYRO_SPI_BMI088
//#define GYRO_1_ALIGN            CW180_DEG

#define GYRO_1_SPI_INSTANCE     SPI1

#define GYRO_1_CS_PIN           PA8

#define USE_ACC


/*---------------------------------*/

/*------------FLASH----------------*/
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
/*---------------------------------*/


/*-----------USB-UARTs-------------*/
#define USE_VCP
//#define USB_DETECT_PIN          NONE
//#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#define USE_UART3
#define UART3_RX_PIN            PB11
//#define UART3_TX_PIN            PB10
#define INVERTER_PIN_UART3      PC15
#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#define INVERTER_PIN_UART3      PC15
#define SERIAL_PORT_COUNT       6
/*---------------------------------*/

/*-------------SPIs----------------*/
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA8
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
/*---------------------------------*/

/*-------------ADCs----------------*/
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define VBAT_ADC_PIN                    PC2
#define CURRENT_METER_ADC_PIN           PC1
/*---------------------------------*/

/*-------------ESCs----------------*/
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB0  // (HARDARE=0)
/*---------------------------------*/

/*--------DEFAULT VALUES-----------*/
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
/*---------------------------------*/

/*--------------TIMERS-------------*/
#if defined(FF_FORTINIF4_REV03)
#define USABLE_TIMER_CHANNEL_COUNT  7
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
#else
#define USABLE_TIMER_CHANNEL_COUNT  6
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) )
#endif
/*---------------------------------*/
