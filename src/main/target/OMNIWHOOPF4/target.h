/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#define TARGET_BOARD_IDENTIFIER "OWF4"

#define USBD_PRODUCT_STRING "OmniWhoop F4"

#define LED0                    PA4

#define BEEPER                  PC13

#define INVERTER_PIN_UART2      PC14

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA15
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define MPU6500_CS_PIN          SPI2_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI2

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       ALIGN_DEFAULT

#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      ALIGN_DEFAULT

#define USE_EXTI
#define MPU_INT_EXTI            PA8
#define USE_MPU_DATA_READY_SIGNAL

#define BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI2
#define BMP280_CS_PIN           PA9

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PA10
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0

#define USE_ADC
#define VBAT_ADC_PIN            PA1

#define SPEKTRUM_BIND_PIN       UART1_RX_PIN

#define USE_FLASH_M25P16
#define M25P16_CS_PIN           SPI1_NSS_PIN
#define M25P16_SPI_INSTANCE     SPI1
#define USE_FLASHFS
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) )
