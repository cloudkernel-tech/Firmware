/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * PX4FMUv2 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

// flags for boards
#define KERLOUD_MINI_V1    //kercloud mini v1 board 2020 May


/* Run time Hardware detection */
#define BOARD_HAS_SIMPLE_HW_VERSIONING 1
#define HW_VER_PA8             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN8)
#define HW_VER_PB4             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4)
#define HW_VER_PB12            (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN12)
#define HW_VER_PA8_INIT        (GPIO_VDD_5V_PERIPH_EN)
#define HW_VER_PB4_INIT        (GPIO_SPI1_EXTI_DRDY_PB4)
#define HW_VER_PB12_INIT       (GPIO_CAN2_RX | GPIO_PULLUP) /* Assume V2 needing pull up */
#define HW_VER_FMUV2_STATE     0x8 /* PB12:PU:1 PB12:PD:0 PB4:PU:0 PB4PD:0 */
#define HW_VER_FMUV3_STATE     0xE /* PB12:PU:1 PB12:PD:1 PB4:PU:1 PB4PD:0 */
#define HW_VER_FMUV2MINI_STATE 0xA /* PB12:PU:1 PB12:PD:0 PB4:PU:1 PB4PD:0 */
#define HW_VER_FMUV2X_STATE    0xB /* PB12:PU:1 PB12:PD:0 PB4:PU:1 PB4PD:1 */
#define HW_VER_TYPE_INIT {'V','2',0, 0}
#define BOARD_NUM_SPI_CFG_HW_VERSIONS 3

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* PX4IO connection configuration */
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE	"/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO	GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO	GPIO_USART6_RX
#define PX4IO_SERIAL_BASE	STM32_USART6_BASE	/* hardwired on the board */
#define PX4IO_SERIAL_VECTOR	STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP	DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP	DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG	STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN	RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK	STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE	1500000			/* 1.5Mbps -> max rate for IO */


/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define BOARD_OVERLOAD_LED LED_AMBER

#define GPIO_SPI1_EXTI_DRDY_PB4          (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN4)

#define BOARD_SPI_BUS_MAX_BUS_ITEMS 3

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_I2C_BUS_ONBOARD	2
#define PX4_I2C_BUS_LED		PX4_I2C_BUS_ONBOARD

/*----------------------------------------------------------*/
/*           FMUv3 Cube SPI chip selects and DRDY           */
/*----------------------------------------------------------*/
/* Due to inconsistent use of chip select and dry signal on
 * different board that use this build. We are defining the GPIO
 * inclusive of the SPI port and GPIO to help identify pins the
 * are part of the sensor Net's controlled by different power
 * domains.
 *
 *  --------------- SPI1 -------------------- SPI4 --------------     Incompatibilities    ---------
 *  FMUv3 Cube:                                                       FmuV2            PixhawkMini
 *   Power Domain:  VDD_3V3_SENSORS_EN         NA			      V3V:SPI V5:SPI4   V3V:SPI1 No SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0                                      EXTERN_DRDY          SPI1:GYRO_DRDY          NC
 *   PB1            MAG_DRDY                  nEXTERN_CS           -SPI1:MAG_DRDY          NC
 *   PB4                                          NC               SPI1:ACCEL_DRDY         NC
 *   PC1            SPI_INT_MAG_!CS                                -ADC1_IN11              NC
 *   PC2            nMPU_CS                                        @MPU6000             @MPU9250
 *   PC13                                     nGYRO_EXT_CS         SPI1:nGYRO_CS           NC
 *   PC14                                     nBARO_EXT_CS         GPIO_EXT_1            -20608_DRDY
 *   PC15                                     nACCEL_MAG_EXT_CS    SPI1:nACCEL_MAG_CS     20608_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000              @MPU9250
 *   PE2                                      SPI_EXT_SCK                                  NC
 *   PE4                                      MPU_EXT_CS           SPI4:nSPI_EXT_NSS       NC
 *   PE5                                      SPI_EXT_MISO                                 NC
 *   PE6                                      SPI_EXT_MOSI                                 NC
 *
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 */

/* FMUv3 Cube SPI1 chip selects */
/*      Was a spare ACD IN10 on V2 */
#define GPIO_SPI1_CS_PC1                 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN1)
#define GPIO_SPI4_CS_PB1                 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_SPI4_CS_PC13                GPIO_SPI1_CS_PC13
#define GPIO_SPI4_CS_PC14                GPIO_SPI4_GPIO_PC14
#define GPIO_SPI4_CS_PC15                GPIO_SPI1_CS_PC15

/* FMUv3 Cube chip selects Assignments */
/*                                                        Cube 2.0   V2.1    */
#define GPIO_SPI1_CS_MPU                 GPIO_SPI1_CS_PC2  /* MPU600    MPU9250 */
#define GPIO_SPI1_CS_BARO                GPIO_SPI1_CS_PD7  /* MS5611    MS5611  */
#define GPIO_SPI1_CS_HMC                 GPIO_SPI1_CS_PC1  /* HMC5983   Removed */

/* N.B. bus moves from SPI1 to SPI4 */
#define GPIO_SPI4_GYRO_EXT_CS            GPIO_SPI4_CS_PC13
#define GPIO_SPI4_BARO_EXT_CS            GPIO_SPI4_CS_PC14
#define GPIO_SPI4_ACCEL_MAG_EXT_CS       GPIO_SPI4_CS_PC15

/* No move */
#define GPIO_SPI4_MPU_EXT_CS             GPIO_SPI4_NSS_PE4

/* FMUv3 DRDY Assignments */
#define GPIO_SPI4_EXTI_DRDY_PB0          GPIO_SPI1_EXTI_DRDY_PB0
#define GPIO_SPI4_EXTI_EXTERN_DRDY       GPIO_SPI4_EXTI_DRDY_PB0
#define GPIO_SPI4_EXTERN_CS              GPIO_SPI4_CS_PB1
/* PB1 is an External CS on V3 */

#define PX4_SPIDEV_HMC            5

/*----------------------------------------------------------*/
/*       End FMUv3 Cube SPI chip selects and DRDY           */
/*----------------------------------------------------------*/
/*----------------------------------------------------------*/
/* Due to inconsistent use of chip select and dry signal on
 * different board that use this build. We are defining the GPIO
 * inclusive of the SPI port and GPIO to help identify pins the
 * are part of the sensor Net's controlled by different power
 * domains.
 *
 *  --------------- SPI1 -------------------- SPI4 --------------      Incompatibilities    ---------
 *  FMUv2 Pixhawk Mini                                                FmuV2               FmuV3 Cube
 *   Power Domain:  VDD_3V3_SENSORS_EN        NA                  V3V:SPI V5:SPI4        V3V:SPI1&SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0            NC                                             SPI1:GYRO_DRDY      SPI4:EXTERN_DRDY
 *   PB1            NC                                             -SPI1:MAG_DRDY      +SPI4:nEXTERN_CS
 *   PB4            NC                                             SPI1:ACCEL_DRDY     NC
 *   PC1            Spare ADC ( NC )                                                   +SPI1:SPI_INT_MAG_!CS
 *   PC2            nMPU_CS                                        @MPU6000            @MPU6000|MPU9250
 *   PC13           NC                                             SPI1:nGYRO_CS       SPI4:nGYRO_EXT_CS
 *   PC14           20608_DRDY                                     +GPIO_EXT_1         nBARO_EXT_CS
 *   PC15           20608_CS                                       nACCEL_MAG_CS       SPI4:nACCEL_MAG_EXT_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000            @MPU6000|MPU9250
 *   PE2                                      NC                   SPI_EXT_SCK         SPI_EXT_SCK
 *   PE4                                      NC                   SPI4:nSPI_EXT_NSS   SPI4:nMPU_EXT_CS
 *   PE5                                      NC                   SPI_EXT_MISO        SPI_EXT_MISO
 *   PE6                                      NC                   SPI_EXT_MOSI        SPI_EXT_MOSI
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 */

/*----------------------------------------------------------*/
/*           FMUv2 PixhawkMini SPI chip selects and DRDY    */
/*----------------------------------------------------------*/

/* FMUv2 PixhawkMini SPI1 chip selects */

/* FMUv3 Cube chip selects Assignments */

#define GPIO_SPI1_CS_MPU                 GPIO_SPI1_CS_PC2  /* MPU9250  */
#define GPIO_SPI1_CS_BARO                GPIO_SPI1_CS_PD7  /* MS5611   */
#define GPIO_SPI1_CS_20608               GPIO_SPI1_CS_PC15 /* ICM20608 */

/* FMUv3 DRDY Assignments */

/* Pixhawk mini has reused the PC14 GPIO_SPI_CS_EXT1 signal that was associated
 * with SPI4.
 */
#define GPIO_SPI1_EXTI_20608_DRDY_PC14   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN14)

#define PX4_SPIDEV_ICM_20608         6   /* ICM_20608 on PC15 */

#define PX4_SPIDEV_ICM_20602                7
#define PX4_SPIDEV_ICM_20689                8

/* Kerloud mini assignments*/
#ifdef KERLOUD_MINI_V1
    #define GPIO_SPI1_CS_20602               GPIO_SPI1_CS_PC15 /* ICM20602 */
    #define GPIO_SPI1_CS_20689               GPIO_SPI1_CS_PC2  /* ICM20689 */
#endif

#define BOARD_OVERRIDE_I2C_BUS_EXTERNAL

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	2
#define ADC_BATTERY_CURRENT_CHANNEL	3
#define ADC_5V_RAIL_SENSE		4
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV   (10.177939394f)
#define BOARD_BATTERY1_A_PER_V (15.391030303f)

/* Power supply control and monitoring GPIOs */
#define GPIO_VDD_5V_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_VDD_BRICK_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_VDD_SERVO_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN7)
#define GPIO_VDD_USB_VALID		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)
#define GPIO_VDD_5V_HIPOWER_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
#define GPIO_VDD_5V_PERIPH_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)

/* Tone alarm output */
#define TONE_ALARM_TIMER	2	/* timer 2 */
#define TONE_ALARM_CHANNEL	1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS	6
#define DIRECT_INPUT_TIMER_CHANNELS  6

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER		4
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_TIM4_CH2IN_2

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (!px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
#define BOARD_ADC_SERVO_VALID   (!px4_arch_gpioread(GPIO_VDD_SERVO_VALID))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_VDD_USB_VALID))
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_OC))

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4, 5};

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

/****************************************************************************************************
 * Name: board_spi_reset board_peripheral_reset
 *
 * Description:
 *   Called to reset SPI and the perferal bus
 *
 ****************************************************************************************************/

extern void board_peripheral_reset(int ms);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
