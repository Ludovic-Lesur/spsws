/*
 * error_base.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_BASE_H__
#define __ERROR_BASE_H__

// Peripherals.
#include "adc.h"
#include "aes.h"
#include "flash.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "nvm.h"
#include "rcc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
// Utils.
#include "math.h"
#include "parser.h"
#include "string.h"
// Components.
#include "dps310.h"
#include "max11136.h"
#include "neom8n.h"
#include "power.h"
#include "rain.h"
#include "sht3x.h"
#include "si1133.h"
#include "sky13317.h"
#include "sx1232.h"
#include "wind.h"
// Sigfox.
#include "sigfox_error.h"

/*** ERROR structures ***/

/*!******************************************************************
 * \enum ERROR_base_t
 * \brief Board error bases.
 *******************************************************************/
typedef enum {
	SUCCESS = 0,
	// Peripherals.
	ERROR_BASE_ADC1 = 0x0100,
	ERROR_BASE_AES = (ERROR_BASE_ADC1 + ADC_ERROR_BASE_LAST),
	ERROR_BASE_FLASH = (ERROR_BASE_AES + AES_ERROR_BASE_LAST),
	ERROR_BASE_I2C1 = (ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
	ERROR_BASE_IWDG = (ERROR_BASE_I2C1 + I2C_ERROR_BASE_LAST),
	ERROR_BASE_LPTIM1 = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
	ERROR_BASE_LPUART1 = (ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST),
	ERROR_BASE_NVM = (ERROR_BASE_LPUART1 + LPUART_ERROR_BASE_LAST),
	ERROR_BASE_RCC = (ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
	ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
	ERROR_BASE_SPI1 = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	ERROR_BASE_TIM2 = (ERROR_BASE_SPI1 + SPI_ERROR_BASE_LAST),
	ERROR_BASE_TIM21 = (ERROR_BASE_TIM2 + TIM_ERROR_BASE_LAST),
	ERROR_BASE_TIM22 = (ERROR_BASE_TIM21 + TIM_ERROR_BASE_LAST),
#ifdef HW1_0
	ERROR_BASE_USART2 = (ERROR_BASE_TIM22 + TIM_ERROR_BASE_LAST),
#endif
#ifdef HW2_0
	ERROR_BASE_USART1 = (ERROR_BASE_TIM22 + TIM_ERROR_BASE_LAST),
#endif
	// Utils.
#ifdef HW1_0
	ERROR_BASE_MATH = (ERROR_BASE_USART2 + USART_ERROR_BASE_LAST),
#endif
#ifdef HW2_0
	ERROR_BASE_MATH = (ERROR_BASE_USART1 + USART_ERROR_BASE_LAST),
#endif
	ERROR_BASE_PARSER = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
	ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
	// Components.
	ERROR_BASE_DPS310 = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	ERROR_BASE_MAX11136 = (ERROR_BASE_DPS310 + DPS310_ERROR_BASE_LAST),
	ERROR_BASE_NEOM8N = (ERROR_BASE_MAX11136 + MAX11136_ERROR_BASE_LAST),
	ERROR_BASE_POWER = (ERROR_BASE_NEOM8N + NEOM8N_ERROR_BASE_LAST),
	ERROR_BASE_RAIN = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
	ERROR_BASE_SHT3X = (ERROR_BASE_RAIN + RAIN_ERROR_BASE_LAST),
	ERROR_BASE_SI1133 = (ERROR_BASE_SHT3X + SHT3X_ERROR_BASE_LAST),
	ERROR_BASE_SKY13317 = (ERROR_BASE_SI1133 + SI1133_ERROR_BASE_LAST),
	ERROR_BASE_SX1232 = (ERROR_BASE_SKY13317 + SKY13317_ERROR_BASE_LAST),
	ERROR_BASE_WIND = (ERROR_BASE_SX1232 + SX1232_ERROR_BASE_LAST),
	// Sigfox.
	ERROR_BASE_SIGFOX_EP_LIB = (ERROR_BASE_WIND + WIND_ERROR_BASE_LAST),
	ERROR_BASE_SIGFOX_EP_ADDON_RFP = (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_LAST * 0x0100)),
	// Last base value.
	ERROR_BASE_LAST = (ERROR_BASE_SIGFOX_EP_ADDON_RFP + 0x0100)
} ERROR_base_t;

#endif /* __ERROR_BASE_H__ */
