/*
 * mapping.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#ifndef MAPPING_H
#define MAPPING_H

#include "gpio_reg.h"
#include "mode.h"

/* --------------------------------------------- Hardware configuration (to be set) --------------------------------------------- */

// PCB internal configuration.
#ifdef HW1_0
#define USE_SX1232_DIOX 					// To be defined if SX1232_DIOx pin is connected to MCU, otherwise DIO3 available.
#define USE_MAX11136_EOC 					// To be defined is EOC pin connected to MCU, otherwise DIO1 available.
#endif

// Wind vane type.
#define WIND_VANE_ULTIMETER					// Phase shift technique.
//#define WIND_VANE_ARGENT_DATA_SYSTEMS		// Analog technique.

// Digital I/O (DIOx pins mapping).
#ifdef HW1_0
#if (defined CM_RTC || defined ATM)
#define GPIO_WIND_SPEED						GPIO_DIO0
#ifdef WIND_VANE_ULTIMETER
#define GPIO_WIND_DIRECTION					GPIO_DIO2
#endif
#define GPIO_RAIN_GAUGE						GPIO_DIO3
#endif
#endif
#ifdef HW2_0
#if (defined CM_RTC || defined ATM)
#define GPIO_WIND_SPEED						GPIO_DIO0
#ifdef WIND_VANE_ULTIMETER
#define GPIO_WIND_DIRECTION					GPIO_DIO1
#endif
#define GPIO_RAIN_GAUGE						GPIO_DIO2
#endif
#endif

// Analog inputs (MAX11136 channels mapping).
#ifdef HW1_0
#if (defined CM_RTC || defined ATM)
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define MAX11136_CHANNEL_WIND_DIRECTION		MAX11136_CHANNEL_AIN0
#endif
#endif
#define MAX11136_CHANNEL_SOLAR_PANEL		MAX11136_CHANNEL_AIN4
#define MAX11136_CHANNEL_SUPERCAP			MAX11136_CHANNEL_AIN5
#define MAX11136_CHANNEL_LDR				MAX11136_CHANNEL_AIN6
#define MAX11136_CHANNEL_BANDGAP			MAX11136_CHANNEL_AIN7
#define MAX11136_BANDGAP_VOLTAGE_MV			2048
#endif
#ifdef HW2_0
#if (defined CM_RTC || defined ATM)
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define MAX11136_CHANNEL_WIND_DIRECTION		MAX11136_CHANNEL_AIN0
#endif
#endif
#define MAX11136_CHANNEL_LDR				MAX11136_CHANNEL_AIN1
#define MAX11136_CHANNEL_BANDGAP			MAX11136_CHANNEL_AIN5
#define MAX11136_BANDGAP_VOLTAGE_MV			2048
#define MAX11136_CHANNEL_SOLAR_PANEL		MAX11136_CHANNEL_AIN6
#define MAX11136_CHANNEL_SUPERCAP			MAX11136_CHANNEL_AIN7


#endif

/* ------------------------------------------------------------------------------------------------------------------------------ */

// DIO0 / MAX5495_CS.
#ifdef HW1_0
#ifdef IM_HWT
#define GPIO_MAX5495_CS				(GPIO) {GPIOA, 0, 1, 0}
#else
#define GPIO_DIO0					(GPIO) {GPIOA, 0, 1, 0}
#endif
#endif
#ifdef HW2_0
#define GPIO_DIO0					(GPIO) {GPIOA, 0, 10, 0}
#endif

// Debug LED / HWT_Out.
#ifdef HW1_0
#ifdef IM_HWT
#define GPIO_HWT_OUT				(GPIO) {GPIOA, 0, 2, 0}
#else
#define GPIO_LED					(GPIO) {GPIOA, 0, 2, 0}
#endif
#endif
#ifdef HW2_0
#define GPIO_LED					(GPIO) {GPIOA, 0, 12, 0}
#endif

// RF switch control.
#ifdef HW1_0
#define GPIO_RF_CHANNEL_A			(GPIO) {GPIOA, 0, 3, 0}
#define GPIO_RF_CHANNEL_B			(GPIO) {GPIOA, 0, 4, 0}
#endif
#ifdef HW2_0
#define GPIO_RF_TX_ENABLE			(GPIO) {GPIOA, 0, 1, 0}
#define GPIO_RF_RX_ENABLE			(GPIO) {GPIOA, 0, 0, 0}
#endif

// SPI1.
#define GPIO_SPI1_SCK				(GPIO) {GPIOA, 0, 5, 0}
#define GPIO_SPI1_MISO				(GPIO) {GPIOA, 0, 6, 0}
#define GPIO_SPI1_MOSI				(GPIO) {GPIOA, 0, 7, 0}

// DIO3 / SX1232_DIOx.
#ifdef HW1_0
#ifdef USE_SX1232_DIOX
#define GPIO_SX1232_DIOX			(GPIO) {GPIOA, 0, 8, 0}
#else
#define GPIO_DIO3					(GPIO) {GPIOA, 0, 8, 0}
#endif
#endif
#ifdef HW2_0
#define GPIO_SX1232_DIO2			(GPIO) {GPIOB, 1, 0, 0}
#define GPIO_DIO3					(GPIO) {GPIOB, 1, 3, 0}
#endif

// USART1 / USART2.
#ifdef HW1_0
#define GPIO_USART2_TX				(GPIO) {GPIOA, 0, 9, 4}
#define GPIO_USART2_RX				(GPIO) {GPIOA, 0, 10, 4}
#endif
#ifdef HW2_0
#define GPIO_USART1_TX				(GPIO) {GPIOB, 1, 6, 0}
#define GPIO_USART1_RX				(GPIO) {GPIOB, 1, 7, 0}
#endif

// Debug power enable.
#ifdef HW1_0
#define GPIO_DEBUG_POWER_ENABLE		(GPIO) {GPIOA, 0, 11, 0}
#endif

// GPS power enable.
#ifdef HW1_0
#define GPIO_GPS_POWER_ENABLE		(GPIO) {GPIOA, 0, 12, 0}
#endif
#ifdef HW2_0
#define GPIO_GPS_POWER_ENABLE		(GPIO) {GPIOB, 1, 2, 0}
#endif

// LPUART1.
#ifdef HW1_0
#define GPIO_LPUART1_RX				(GPIO) {GPIOA, 0, 13, 6}
#define GPIO_LPUART1_TX				(GPIO) {GPIOA, 0, 14, 6}
#endif
#ifdef HW2_0
#define GPIO_LPUART1_RX				(GPIO) {GPIOB, 1, 10, 4}
#define GPIO_LPUART1_TX				(GPIO) {GPIOB, 1, 11, 4}
#endif

// DIO2.
#define GPIO_DIO2					(GPIO) {GPIOA, 0, 15, 0}

// DIO4
#ifdef HW2_0
#define GPIO_DIO4					(GPIO) {GPIOB, 1, 4, 0}
#endif

// SX1232_CS.
#ifdef HW1_0
#define GPIO_SX1232_CS				(GPIO) {GPIOB, 1, 0, 0}
#endif
#ifdef HW2_0
#define GPIO_SX1232_CS				(GPIO) {GPIOA, 0, 4, 0}
#endif

// RF power enable.
#ifdef HW1_0
#define GPIO_RF_POWER_ENABLE		(GPIO) {GPIOB, 1, 1, 0}
#endif
#ifdef HW2_0
#define GPIO_RF_POWER_ENABLE		(GPIO) {GPIOA, 0, 2, 0}
#endif

// SX1232_DIO0.
#ifdef HW1_0
#define GPIO_SX1232_DIO0			(GPIO) {GPIOB, 1, 2, 0}
#endif
#ifdef HW2_0
#define GPIO_SX1232_DIO0			(GPIO) {GPIOB, 1, 1, 0}
#endif

// DIO1 / MAX11136_EOC.
#ifdef HW1_0
#ifdef USE_MAX11136_EOC
#define GPIO_MAX11136_EOC			(GPIO) {GPIOB, 1, 3, 0}
#else
#define GPIO_DIO1					(GPIO) {GPIOB, 1, 3, 0}
#endif
#endif
#ifdef HW2_0
#define GPIO_MAX11136_EOC			(GPIO) {GPIOA, 0, 8, 0}
#define GPIO_DIO1					(GPIO) {GPIOA, 0, 11, 0}
#endif

// ADC power enable.
#ifdef HW2_0
#define GPIO_ADC_POWER_ENABLE		(GPIO) {GPIOA, 0, 9, 0}
#endif

// SPI2.
#ifdef HW2_0
#define GPIO_SPI2_SCK				(GPIO) {GPIOB, 1, 13, 0}
#define GPIO_SPI2_MISO				(GPIO) {GPIOB, 1, 14, 0}
#define GPIO_SPI2_MOSI				(GPIO) {GPIOB, 1, 15, 0}
#endif

// MAX11136_CS.
#ifdef HW1_0
#define GPIO_MAX11136_CS			(GPIO) {GPIOB, 1, 4, 0}
#endif
#ifdef HW2_0
#define GPIO_MAX11136_CS			(GPIO) {GPIOB, 1, 12, 0}
#endif

// Sensors power enable.
#define GPIO_SENSORS_POWER_ENABLE	(GPIO) {GPIOB, 1, 5, 0}

// I2C1.
#ifdef HW1_0
#define GPIO_I2C1_SCL				(GPIO) {GPIOB, 1, 6, 1}
#define GPIO_I2C1_SDA				(GPIO) {GPIOB, 1, 7, 1}
#endif
#ifdef HW2_0
#define GPIO_I2C1_SCL				(GPIO) {GPIOB, 1, 8, 4}
#define GPIO_I2C1_SDA				(GPIO) {GPIOB, 1, 9, 4}
#endif

// 16MHz TCXO power enable.
#ifdef HW1_0
#define GPIO_TCXO16_POWER_ENABLE	(GPIO) {GPIOB, 1, 8, 0}
#endif
#ifdef HW2_0
#define GPIO_TCXO16_POWER_ENABLE	(GPIO) {GPIOC, 2, 13, 0}
#endif

// 32MHz TCXO power enable.
#ifdef HW2_0
#define GPIO_TCXO32_POWER_ENABLE	(GPIO) {GPIOA, 0, 3, 0}
#endif

// 32.768kHz quartz / Main power disable.
#ifdef HW1_0
#ifdef IM_HWT
#define GPIO_MAIN_POWER_DISABLE		(GPIO) {GPIOC, 2, 14, 0}
#endif
#endif

// 32.768kHz quartz / HWT_Reset.
#ifdef HW1_0
#ifdef IM_HWT
#define GPIO_HWT_RESET				(GPIO) {GPIOC, 2, 15, 0}
#endif
#endif

/*** Errors management ***/

#if (defined WIND_VANE_ULTIMETER && defined WIND_VANE_ARGENT_DATA_SYSTEMS)
#error "Only one wind vane type must be selected"
#endif

#endif /* MAPPING_H */
