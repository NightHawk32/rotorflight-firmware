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

//#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "B743"
#define USBD_PRODUCT_STRING  "BMI088-H7"

#define LED0_PIN                PC4
#define LED1_PIN                PC5

#define USE_BEEPER
#define BEEPER_PIN              PB2
#define BEEPER_INVERTED
#define BEEPER_PWM_HZ           2500 

// *************** SPI1 & SPI4, Gyro & ACC *******************

#define USE_SPI
#define USE_GYRO
#define USE_ACC
#define USE_ACCGYRO_SPI_BMI088

#define BMI088_CS_A_PIN         PE4
#define BMI088_CS_G_PIN         PE3
#define BMI088_INT_G_PIN        PE5
#define BMI088_INT_A_PIN        PC13

#define BMI320_CS_PIN           PC0
#define BMI320_INT_PIN          PA4

#define USE_SPI_DEVICE_1
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_EXTI_PIN         BMI088_INT_G_PIN
#define GYRO_1_CS_PIN           BMI088_CS_G_PIN
#define GYRO_1_ACC_CS_PIN       BMI088_CS_A_PIN

//#define GYRO_2_SPI_INSTANCE     SPI1
//#define GYRO_2_CS_PIN           BMI320_CS_PIN
//#define GYRO_2_EXTI_PIN         BMI320_INT_PIN

#define ENSURE_MPU_DATA_READY_IS_LOW
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

// *************** SPI ***********************
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

/*------------FLASH----------------*/
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

// *************** I2C /Baro/Mag *********************

#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1            (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7


/*#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1            (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define MAG_I2C_INSTANCE        (I2CDEV_1)
#define USE_MAG
#define USE_MAG_BMP388

#define BARO_I2C_INSTANCE       (I2CDEV_1)
#define USE_BARO
#define DEFAULT_BARO_BMP851
#define USE_BARO_BMP581*/

// *************** UART *****************************

#define USE_VCP
//#define USB_DETECT_PIN          PE2
//#define USE_USB_DETECT

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART3
#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7  //PPM

#define USE_UART7
#define UART7_TX_PIN            PE8
#define UART7_RX_PIN            PE7

     
#define USE_SOFTSERIAL1
#define SOFTSERIAL1_TX_PIN      PC6 // TX6 Pad

#define SERIAL_PORT_COUNT       9

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

// *************** ADC *****************************
#define USE_DMA
#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9

#define USE_ADC
#define USE_ADC_INTERNAL   // ADC3

#define VBAT_ADC_PIN            PC1  //ADC123 VBAT1 
//#define CURRENT_METER_ADC_PIN   PC2  //ADC123 CURR1

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define VBAT_SCALE_DEFAULT            110
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT   250

// *************** PINIO ***************************

/*#define USE_PINIO
#define PINIO1_PIN              PD10  // Vsw power switch
#define PINIO2_PIN              PD11  // Camera switch
#define USE_PINIOBOX*/

// *************** Others ***************************

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY )

//#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

#define USE_ESCSERIAL

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS    (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(5)|TIM_N(8)|TIM_N(15)|TIM_N(16)|TIM_N(17))
