/*
 * mapping.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#ifndef __MAPPING_H__
#define __MAPPING_H__

#include "gpio.h"

/*** MAPPING macros ***/

#ifdef HW1_0
#define GPIO_SX1232_DIO2_LOW(void)		{ GPIOA -> ODR &= 0xFFFFFEFF; }
#define GPIO_SX1232_DIO2_HIGH(void)		{ GPIOA -> ODR |= 0x00000100; }
#else
#define GPIO_SX1232_DIO2_LOW(void)		{ GPIOB -> ODR &= 0xFFFFFFFE; }
#define GPIO_SX1232_DIO2_HIGH(void)		{ GPIOB -> ODR |= 0x00000001; }
#endif
#ifdef HW1_0
#define GPIO_SX1232_CS_LOW(void)		{ GPIOB -> ODR &= 0xFFFFFFFE; }
#define GPIO_SX1232_CS_HIGH(void)		{ GPIOB -> ODR |= 0x00000001; }
#else
#define GPIO_SX1232_CS_LOW(void)		{ GPIOA -> ODR &= 0xFFFFFFEF; }
#define GPIO_SX1232_CS_HIGH(void)		{ GPIOA -> ODR |= 0x00000010; }
#endif

/*** MAPPING global variables ***/

// DIO0.
extern const GPIO_pin_t GPIO_DIO0;
// Debug LED.
extern const GPIO_pin_t GPIO_LED;
// RF switch control.
#ifdef HW1_0
extern const GPIO_pin_t GPIO_RF_CHANNEL_A;
extern const GPIO_pin_t GPIO_RF_CHANNEL_B;
#endif
#ifdef HW2_0
extern const GPIO_pin_t GPIO_RF_TX_ENABLE;
extern const GPIO_pin_t GPIO_RF_RX_ENABLE;
#endif
// SPI1.
extern const GPIO_pin_t GPIO_SPI1_SCK;
extern const GPIO_pin_t GPIO_SPI1_MISO;
extern const GPIO_pin_t GPIO_SPI1_MOSI;
// SX1232_DIO2.
extern const GPIO_pin_t GPIO_SX1232_DIO2;
// USART1 / USART2.
#ifdef HW1_0
extern const GPIO_pin_t GPIO_USART2_TX;
extern const GPIO_pin_t GPIO_USART2_RX;
#else
extern const GPIO_pin_t GPIO_USART1_TX;
extern const GPIO_pin_t GPIO_USART1_RX;
#endif
// GPS power enable.
extern const GPIO_pin_t GPIO_GPS_POWER_ENABLE;
// LPUART1.
extern const GPIO_pin_t GPIO_LPUART1_RX;
extern const GPIO_pin_t GPIO_LPUART1_TX;
// Programming pins.
#ifdef HW2_0
extern const GPIO_pin_t GPIO_SWDIO;
extern const GPIO_pin_t GPIO_SWCLK;
#endif
// DIO2.
extern const GPIO_pin_t GPIO_DIO2;
// DIO1, DIO3 and DIO4
#ifdef HW2_0
extern const GPIO_pin_t GPIO_DIO1;
extern const GPIO_pin_t GPIO_DIO3;
extern const GPIO_pin_t GPIO_DIO4;
#endif
// SX1232_CS.
extern const GPIO_pin_t GPIO_SX1232_CS;
// RF power enable.
extern const GPIO_pin_t GPIO_RF_POWER_ENABLE;
// SX1232_DIO0.
extern const GPIO_pin_t GPIO_SX1232_DIO0;
// DIO1 / MAX11136_EOC.
extern const GPIO_pin_t GPIO_MAX11136_EOC;
// ADC power enable.
#ifdef HW2_0
extern const GPIO_pin_t GPIO_ADC_POWER_ENABLE;
#endif
// SPI2.
#ifdef HW2_0
extern const GPIO_pin_t GPIO_SPI2_SCK;
extern const GPIO_pin_t GPIO_SPI2_MISO;
extern const GPIO_pin_t GPIO_SPI2_MOSI;
#endif
// MAX11136_CS.
extern const GPIO_pin_t GPIO_MAX11136_CS;
// Sensors power enable.
extern const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE;
// I2C1.
extern const GPIO_pin_t GPIO_I2C1_SCL;
extern const GPIO_pin_t GPIO_I2C1_SDA;
// 16MHz TCXO power enable.
extern const GPIO_pin_t GPIO_TCXO16_POWER_ENABLE;
// 32MHz TCXO power enable.
extern const GPIO_pin_t GPIO_TCXO32_POWER_ENABLE;

#endif /* __MAPPING_H__ */
