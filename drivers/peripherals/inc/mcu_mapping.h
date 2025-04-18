/*
 * mcu_mapping.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#ifndef __MCU_MAPPING_H__
#define __MCU_MAPPING_H__

#include "gpio.h"
#include "i2c.h"
#include "lpuart.h"
#include "spi.h"
#include "usart.h"

/*** MCU MAPPING macros ***/

#define I2C_INSTANCE_SENSORS            I2C_INSTANCE_I2C1

#define SPI_INSTANCE_RADIO              SPI_INSTANCE_SPI1
#ifdef HW1_0
#define SPI_INSTANCE_ADC                SPI_INSTANCE_SPI1
#endif
#ifdef HW2_0
#define SPI_INSTANCE_ADC                SPI_INSTANCE_SPI2
#endif

#define TIM_INSTANCE_MCU_API            TIM_INSTANCE_TIM2
#define TIM_INSTANCE_RF_API             TIM_INSTANCE_TIM22

#ifdef HW1_0
#define USART_INSTANCE_AT               USART_INSTANCE_USART2
#endif
#ifdef HW2_0
#define USART_INSTANCE_AT               USART_INSTANCE_USART1
#endif

#ifdef HW1_0
#define GPIO_SX1232_DIO2_LOW(void)      { GPIOA->ODR &= 0xFFFFFEFF; }
#define GPIO_SX1232_DIO2_HIGH(void)     { GPIOA->ODR |= 0x00000100; }
#else
#define GPIO_SX1232_DIO2_LOW(void)      { GPIOB->ODR &= 0xFFFFFFFE; }
#define GPIO_SX1232_DIO2_HIGH(void)     { GPIOB->ODR |= 0x00000001; }
#endif
#ifdef HW1_0
#define GPIO_SX1232_CS_LOW(void)        { GPIOB->ODR &= 0xFFFFFFFE; }
#define GPIO_SX1232_CS_HIGH(void)       { GPIOB->ODR |= 0x00000001; }
#else
#define GPIO_SX1232_CS_LOW(void)        { GPIOA->ODR &= 0xFFFFFFEF; }
#define GPIO_SX1232_CS_HIGH(void)       { GPIOA->ODR |= 0x00000010; }
#endif

/*** MCU MAPPING global variables ***/

// MCU TCXO.
extern const GPIO_pin_t GPIO_TCXO16_POWER_ENABLE;
// DIOx.
extern const GPIO_pin_t GPIO_DIO0;
extern const GPIO_pin_t GPIO_DIO2;
#ifdef HW2_0
extern const GPIO_pin_t GPIO_DIO1;
extern const GPIO_pin_t GPIO_DIO3;
extern const GPIO_pin_t GPIO_DIO4;
#endif
// Radio power control.
extern const GPIO_pin_t GPIO_RF_POWER_ENABLE;
extern const GPIO_pin_t GPIO_TCXO32_POWER_ENABLE;
// Radio switch control.
#ifdef HW1_0
extern const GPIO_pin_t GPIO_RF_CHANNEL_A;
extern const GPIO_pin_t GPIO_RF_CHANNEL_B;
#endif
#ifdef HW2_0
extern const GPIO_pin_t GPIO_RF_TX_ENABLE;
extern const GPIO_pin_t GPIO_RF_RX_ENABLE;
#endif
// SX1232.
extern const SPI_gpio_t SPI_GPIO_SX1232;
extern const GPIO_pin_t GPIO_SX1232_CS;
extern const GPIO_pin_t GPIO_SX1232_DIO0;
extern const GPIO_pin_t GPIO_SX1232_DIO2;
// GPS.
extern const GPIO_pin_t GPIO_GPS_POWER_ENABLE;
extern const LPUART_gpio_t LPUART_GPIO_GPS;
// ADC power control.
#ifdef HW2_0
extern const GPIO_pin_t GPIO_ADC_POWER_ENABLE;
#endif
// MAX11136.
extern const SPI_gpio_t SPI_GPIO_MAX11136;
extern const GPIO_pin_t GPIO_MAX11136_CS;
extern const GPIO_pin_t GPIO_MAX11136_EOC;
// Sensors.
extern const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE;
extern const I2C_gpio_t I2C_GPIO_SENSORS;
// AT interface.
extern const USART_gpio_t USART_GPIO_AT;
// Debug LED.
extern const GPIO_pin_t GPIO_LED;

#endif /* __MCU_MAPPING_H__ */
