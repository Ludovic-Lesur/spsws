/*
 * mapping.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludovic
 */

#ifndef MAPPING_H
#define MAPPING_H

/*** Hardware revision selection ***/

#define HW1_0

/*** Hardware configuration ***/

#ifdef HW1_0
// To be defined is EOC pin connected to MCU.
#define USE_MAX11136_EOC
// To be defined if SX1232_DIOx pin is connected to MCU.
//#define USE_SX1232_DIOX
#endif // HW1_0.

/*** MCU mapping ***/

#ifdef HW1_0

// DIO0 / MAX5495_CS.
#ifdef IM_HWT
#define GPIO_MAX5495_CS				(GPIO) {GPIOA, 1, 0}
#else
#define GPIO_DIO0					(GPIO) {GPIOA, 1, 0}
#endif

// Debug LED / HWT_Out.
#ifdef IM_HWT
#define GPIO_HWT_OUT				(GPIO) {GPIOA, 2, 0}
#else
#define GPIO_LED					(GPIO) {GPIOA, 2, 0}
#endif

// RF switch control.
#define GPIO_RF_CHANNEL_A			(GPIO) {GPIOA, 3, 0}
#define GPIO_RF_CHANNEL_B			(GPIO) {GPIOA, 4, 0}

// SPI1.
#define GPIO_SPI1_SCK				(GPIO) {GPIOA, 5, 0}
#define GPIO_SPI1_MISO				(GPIO) {GPIOA, 6, 0}
#define GPIO_SPI1_MOSI				(GPIO) {GPIOA, 7, 0}

// DIO3 / SX1232_DIOx.
#ifdef USE_SX1232_DIOX
#define GPIO_SX1232_DIOX			(GPIO) {GPIOA, 8, 0}
#else
#define GPIO_DIO3					(GPIO) {GPIOA, 8, 0}
#endif

// USART2.
#define GPIO_USART2_TX				(GPIO) {GPIOA, 9, 4}
#define GPIO_USART2_RX				(GPIO) {GPIOA, 10, 4}

// Debug power enable.
#define GPIO_DEBUG_POWER_ENABLE		(GPIO) {GPIOA, 11, 0}

// GPS power enable.
#define GPIO_GPS_POWER_ENABLE		(GPIO) {GPIOA, 12, 0}

// LPUART1.
#define GPIO_LPUART1_RX				(GPIO) {GPIOA, 13, 6}
#define GPIO_LPUART1_TX				(GPIO) {GPIOA, 14, 6}

// DIO2.
#define GPIO_DIO2					(GPIO) {GPIOA, 15, 0}

// SX1232_CS.
#define GPIO_SX1232_CS				(GPIO) {GPIOB, 0, 0}

// RF power enable.
#define GPIO_RF_POWER_ENABLE		(GPIO) {GPIOB, 1, 0}

// SX1232_DIO0.
#define GPIO_SX1232_DIO0			(GPIO) {GPIOB, 2, 0}

// DIO1 / MAX11136_EOC.
#ifdef USE_MAX11136_EOC
#define GPIO_MAX11136_EOC			(GPIO) {GPIOB, 3, 0}
#else
#define GPIO_DIO1					(GPIO) {GPIOB, 3, 0}
#endif

// MX11136_CS.
#define GPIO_MAX11136_CS			(GPIO) {GPIOB, 4, 0}

// Sensors power enable.
#define GPIO_SENSORS_POWER_ENABLE	(GPIO) {GPIOB, 5, 0}

// I2C1.
#define GPIO_I2C1_SCL				(GPIO) {GPIOB, 6, 1}
#define GPIO_I2C1_SDA				(GPIO) {GPIOB, 7, 1}

// TCXO power enable.
#define GPIO_TCXO_POWER_ENABLE		(GPIO) {GPIOB, 8, 0}

// 32.768kHz quartz / Main power disable.
#ifdef IM_RTC
#define GPIO_MAIN_POWER_DISABLE		(GPIO) {GPIOC, 14, 0}
#endif

// 32.768kHz quartz / HWT_Reset.
#ifdef IM_RTC
#define GPIO_HWT_RESET				(GPIO) {GPIOC, 15, 0}
#endif

#endif // HW1_0.

#endif /* MAPPING_H */
