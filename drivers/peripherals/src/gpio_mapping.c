/*
 * gpio_mapping.c
 *
 *  Created on: 23 apr. 2024
 *      Author: Ludo
 */

#include "gpio_mapping.h"

#include "gpio.h"
#include "gpio_registers.h"
#include "i2c.h"
#include "lpuart.h"
#include "spi.h"
#include "usart.h"

/*** GPIO MAPPING local global variables ***/

// I2C1.
#ifdef HW1_0
static const GPIO_pin_t GPIO_I2C1_SCL = { GPIOB, 1, 6, 1 };
static const GPIO_pin_t GPIO_I2C1_SDA = { GPIOB, 1, 7, 1 };
#else
static const GPIO_pin_t GPIO_I2C1_SCL = { GPIOB, 1, 8, 4 };
static const GPIO_pin_t GPIO_I2C1_SDA = { GPIOB, 1, 9, 4 };
#endif
// SPI1.
static const GPIO_pin_t GPIO_SPI1_SCK = { GPIOA, 0, 5, 0 };
static const GPIO_pin_t GPIO_SPI1_MISO = { GPIOA, 0, 6, 0 };
static const GPIO_pin_t GPIO_SPI1_MOSI = { GPIOA, 0, 7, 0 };
// LPUART1.
#ifdef HW1_0
static const GPIO_pin_t GPIO_LPUART1_RX = { GPIOA, 0, 13, 6 };
static const GPIO_pin_t GPIO_LPUART1_TX = { GPIOA, 0, 14, 6 };
#else
static const GPIO_pin_t GPIO_LPUART1_RX = { GPIOB, 1, 10, 4 };
static const GPIO_pin_t GPIO_LPUART1_TX = { GPIOB, 1, 11, 4 };
#endif
// SPI2.
#ifdef HW2_0
static const GPIO_pin_t GPIO_SPI2_SCK = { GPIOB, 1, 13, 0 };
static const GPIO_pin_t GPIO_SPI2_MISO = { GPIOB, 1, 14, 0 };
static const GPIO_pin_t GPIO_SPI2_MOSI = { GPIOB, 1, 15, 0 };
#endif
// USART.
#ifdef HW1_0
static const GPIO_pin_t GPIO_USART2_TX = { GPIOA, 0, 9, 4 };
static const GPIO_pin_t GPIO_USART2_RX = { GPIOA, 0, 10, 4 };
#else
static const GPIO_pin_t GPIO_USART1_TX = { GPIOB, 1, 6, 0 };
static const GPIO_pin_t GPIO_USART1_RX = { GPIOB, 1, 7, 0 };
#endif

/*** GPIO MAPPING global variables ***/

// MCU TCXO.
#ifdef HW1_0
const GPIO_pin_t GPIO_TCXO16_POWER_ENABLE = { GPIOB, 1, 8, 0 };
#else
const GPIO_pin_t GPIO_TCXO16_POWER_ENABLE = { GPIOC, 2, 13, 0 };
#endif
// DIOx.
#ifdef HW1_0
const GPIO_pin_t GPIO_DIO0 = { GPIOA, 0, 1, 0 };
#else
const GPIO_pin_t GPIO_DIO0 = { GPIOA, 0, 10, 0 };
#endif
const GPIO_pin_t GPIO_DIO2 = { GPIOA, 0, 15, 0 };
#ifdef HW2_0
const GPIO_pin_t GPIO_DIO1 = { GPIOA, 0, 11, 0 };
const GPIO_pin_t GPIO_DIO3 = { GPIOB, 1, 3, 0 };
const GPIO_pin_t GPIO_DIO4 = { GPIOB, 1, 4, 0 };
#endif
// Radio power control.
#ifdef HW1_0
const GPIO_pin_t GPIO_RF_POWER_ENABLE = { GPIOB, 1, 1, 0 };
#else
const GPIO_pin_t GPIO_RF_POWER_ENABLE = { GPIOA, 0, 2, 0 };
#endif
#ifdef HW1_0
const GPIO_pin_t GPIO_TCXO32_POWER_ENABLE = { GPIOA, 0, 11, 0 };
#else
const GPIO_pin_t GPIO_TCXO32_POWER_ENABLE = { GPIOA, 0, 3, 0 };
#endif
// Radio switch control.
#ifdef HW1_0
const GPIO_pin_t GPIO_RF_CHANNEL_A = { GPIOA, 0, 3, 0 };
const GPIO_pin_t GPIO_RF_CHANNEL_B = { GPIOA, 0, 4, 0 };
#endif
#ifdef HW2_0
const GPIO_pin_t GPIO_RF_TX_ENABLE = { GPIOA, 0, 1, 0 };
const GPIO_pin_t GPIO_RF_RX_ENABLE = { GPIOA, 0, 0, 0 };
#endif
// SX1232.
#ifdef HW1_0
const GPIO_pin_t GPIO_SX1232_CS = { GPIOB, 1, 0, 0 };
const GPIO_pin_t GPIO_SX1232_DIO0 = { GPIOB, 1, 2, 0 };
const GPIO_pin_t GPIO_SX1232_DIO2 = { GPIOA, 0, 8, 0 };
#else
const GPIO_pin_t GPIO_SX1232_CS = { GPIOA, 0, 4, 0 };
const GPIO_pin_t GPIO_SX1232_DIO0 = { GPIOB, 1, 1, 0 };
const GPIO_pin_t GPIO_SX1232_DIO2 = { GPIOB, 1, 0, 0 };
#endif
const SPI_gpio_t GPIO_SX1232_SPI = { &GPIO_SPI1_SCK, &GPIO_SPI1_MOSI, &GPIO_SPI1_MISO };
// GPS.
#ifdef HW1_0
const GPIO_pin_t GPIO_GPS_POWER_ENABLE = { GPIOA, 0, 12, 0 };
#else
const GPIO_pin_t GPIO_GPS_POWER_ENABLE = { GPIOB, 1, 2, 0 };
#endif
const LPUART_gpio_t GPIO_GPS_LPUART = { &GPIO_LPUART1_TX, &GPIO_LPUART1_RX };
// ADC power control.
#ifdef HW2_0
const GPIO_pin_t GPIO_ADC_POWER_ENABLE = { GPIOA, 0, 9, 0 };
#endif
// MAX11136.
#ifdef HW1_0
const GPIO_pin_t GPIO_MAX11136_CS = { GPIOB, 1, 4, 0 };
const GPIO_pin_t GPIO_MAX11136_EOC = { GPIOB, 1, 3, 0 };
const SPI_gpio_t GPIO_MAX11136_SPI = { &GPIO_SPI1_SCK, &GPIO_SPI1_MOSI, &GPIO_SPI1_MISO };
#else
const GPIO_pin_t GPIO_MAX11136_CS = { GPIOB, 1, 12, 0 };
const GPIO_pin_t GPIO_MAX11136_EOC = { GPIOA, 0, 8, 0 };
const SPI_gpio_t GPIO_MAX11136_SPI = { &GPIO_SPI2_SCK, &GPIO_SPI2_MOSI, &GPIO_SPI2_MISO };
#endif
// Sensors.
const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE = { GPIOB, 1, 5, 0 };
const I2C_gpio_t GPIO_SENSORS_I2C = { &GPIO_I2C1_SCL, &GPIO_I2C1_SDA };
// AT interface.
#ifdef HW1_0
const USART_gpio_t GPIO_AT_USART = { &GPIO_USART2_TX, &GPIO_USART2_RX };
#endif
#ifdef HW2_0
const USART_gpio_t GPIO_AT_USART = { &GPIO_USART1_TX, &GPIO_USART1_RX };
#endif
// Debug LED.
#ifdef HW1_0
const GPIO_pin_t GPIO_LED = { GPIOA, 0, 2, 0 };
#else
const GPIO_pin_t GPIO_LED = { GPIOA, 0, 12, 0 };
#endif
