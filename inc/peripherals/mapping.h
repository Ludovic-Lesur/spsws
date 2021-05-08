/*
 * mapping.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#ifndef MAPPING_H
#define MAPPING_H

#include "gpio.h"
#include "gpio_reg.h"
#include "mode.h"

// DIO0.
#ifdef HW1_0
static const GPIO GPIO_DIO0 = 					(GPIO) {GPIOA, 0, 1, 0};
#else
static const GPIO GPIO_DIO0 = 					(GPIO) {GPIOA, 0, 10, 0};
#endif

// Debug LED.
#ifdef HW1_0
static const GPIO GPIO_LED = 					(GPIO) {GPIOA, 0, 2, 0};
#else
static const GPIO GPIO_LED = 					(GPIO) {GPIOA, 0, 12, 0};
#endif

// RF switch control.
#ifdef HW1_0
static const GPIO GPIO_RF_CHANNEL_A = 			(GPIO) {GPIOA, 0, 3, 0};
static const GPIO GPIO_RF_CHANNEL_B = 			(GPIO) {GPIOA, 0, 4, 0};
#endif
#ifdef HW2_0
static const GPIO GPIO_RF_TX_ENABLE = 			(GPIO) {GPIOA, 0, 1, 0};
static const GPIO GPIO_RF_RX_ENABLE = 			(GPIO) {GPIOA, 0, 0, 0};
#endif

// SPI1.
static const GPIO GPIO_SPI1_SCK = 				(GPIO) {GPIOA, 0, 5, 0};
static const GPIO GPIO_SPI1_MISO = 				(GPIO) {GPIOA, 0, 6, 0};
static const GPIO GPIO_SPI1_MOSI = 				(GPIO) {GPIOA, 0, 7, 0};

// SX1232_DIO2.
#ifdef HW1_0
static const GPIO GPIO_SX1232_DIO2 = 			(GPIO) {GPIOA, 0, 8, 0};
#else
static const GPIO GPIO_SX1232_DIO2 =			(GPIO) {GPIOB, 1, 0, 0};
#endif

// USART1 / USART2.
#ifdef HW1_0
static const GPIO GPIO_USART2_TX =				(GPIO) {GPIOA, 0, 9, 4};
static const GPIO GPIO_USART2_RX =				(GPIO) {GPIOA, 0, 10, 4};
#else
static const GPIO GPIO_USART1_TX =	 			(GPIO) {GPIOB, 1, 6, 0};
static const GPIO GPIO_USART1_RX =				(GPIO) {GPIOB, 1, 7, 0};
#endif

// GPS power enable.
#ifdef HW1_0
static const GPIO GPIO_GPS_POWER_ENABLE =		(GPIO) {GPIOA, 0, 12, 0};
#else
static const GPIO GPIO_GPS_POWER_ENABLE =		(GPIO) {GPIOB, 1, 2, 0};
#endif

// LPUART1.
#ifdef HW1_0
static const GPIO GPIO_LPUART1_RX =				(GPIO) {GPIOA, 0, 13, 6};
static const GPIO GPIO_LPUART1_TX =				(GPIO) {GPIOA, 0, 14, 6};
#else
static const GPIO GPIO_LPUART1_RX =				(GPIO) {GPIOB, 1, 10, 4};
static const GPIO GPIO_LPUART1_TX =				(GPIO) {GPIOB, 1, 11, 4};
#endif

// Programming pins.
#ifdef HW2_0
static const GPIO GPIO_SWDIO =					(GPIO) {GPIOA, 0, 13, 0};
static const GPIO GPIO_SWCLK =					(GPIO) {GPIOA, 0, 14, 0};
#endif

// DIO2.
static const GPIO GPIO_DIO2 =					(GPIO) {GPIOA, 0, 15, 0};

// DIO1, DIO3 and DIO4
#ifdef HW2_0
static const GPIO GPIO_DIO1 =					(GPIO) {GPIOA, 0, 11, 0};
static const GPIO GPIO_DIO3 = 					(GPIO) {GPIOB, 1, 3, 0};
static const GPIO GPIO_DIO4	=					(GPIO) {GPIOB, 1, 4, 0};
#endif

// SX1232_CS.
#ifdef HW1_0
static const GPIO GPIO_SX1232_CS =				(GPIO) {GPIOB, 1, 0, 0};
#else
static const GPIO GPIO_SX1232_CS =				(GPIO) {GPIOA, 0, 4, 0};
#endif

// RF power enable.
#ifdef HW1_0
static const GPIO GPIO_RF_POWER_ENABLE =		(GPIO) {GPIOB, 1, 1, 0};
#else
static const GPIO GPIO_RF_POWER_ENABLE =		(GPIO) {GPIOA, 0, 2, 0};
#endif

// SX1232_DIO0.
#ifdef HW1_0
static const GPIO GPIO_SX1232_DIO0 =			(GPIO) {GPIOB, 1, 2, 0};
#else
static const GPIO GPIO_SX1232_DIO0 =			(GPIO) {GPIOB, 1, 1, 0};
#endif

// DIO1 / MAX11136_EOC.
#ifdef HW1_0
static const GPIO GPIO_MAX11136_EOC =			(GPIO) {GPIOB, 1, 3, 0};
#else
static const GPIO GPIO_MAX11136_EOC	=			(GPIO) {GPIOA, 0, 8, 0};
#endif

// ADC power enable.
#ifdef HW2_0
static const GPIO GPIO_ADC_POWER_ENABLE =		(GPIO) {GPIOA, 0, 9, 0};
#endif

// SPI2.
#ifdef HW2_0
static const GPIO GPIO_SPI2_SCK =				(GPIO) {GPIOB, 1, 13, 0};
static const GPIO GPIO_SPI2_MISO =				(GPIO) {GPIOB, 1, 14, 0};
static const GPIO GPIO_SPI2_MOSI =				(GPIO) {GPIOB, 1, 15, 0};
#endif

// MAX11136_CS.
#ifdef HW1_0
static const GPIO GPIO_MAX11136_CS =			(GPIO) {GPIOB, 1, 4, 0};
#else
static const GPIO GPIO_MAX11136_CS =			(GPIO) {GPIOB, 1, 12, 0};
#endif

// Sensors power enable.
static const GPIO GPIO_SENSORS_POWER_ENABLE =	(GPIO) {GPIOB, 1, 5, 0};

// I2C1.
#ifdef HW1_0
static const GPIO GPIO_I2C1_SCL	=				(GPIO) {GPIOB, 1, 6, 1};
static const GPIO GPIO_I2C1_SDA	=				(GPIO) {GPIOB, 1, 7, 1};
#else
static const GPIO GPIO_I2C1_SCL =				(GPIO) {GPIOB, 1, 8, 4};
static const GPIO GPIO_I2C1_SDA =				(GPIO) {GPIOB, 1, 9, 4};
#endif

// 16MHz TCXO power enable.
#ifdef HW1_0
static const GPIO GPIO_TCXO16_POWER_ENABLE =	(GPIO) {GPIOB, 1, 8, 0};
#else
static const GPIO GPIO_TCXO16_POWER_ENABLE =	(GPIO) {GPIOC, 2, 13, 0};
#endif

// 32MHz TCXO power enable.
#ifdef HW1_0
static const GPIO GPIO_TCXO32_POWER_ENABLE =	(GPIO) {GPIOA, 0, 11, 0};
#else
static const GPIO GPIO_TCXO32_POWER_ENABLE =	(GPIO) {GPIOA, 0, 3, 0};
#endif

#endif /* MAPPING_H */
