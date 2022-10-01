/*
 * error.h
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#ifndef __ERROR_H__
#define __ERROR_H__

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
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
#include "wind.h"

/*** ERROR macros ***/

#define ERROR_STACK_DEPTH	6

/*** ERROR structures ***/

typedef enum {
	SUCCESS = 0,
	ERROR_BUSY,
	ERROR_SIGFOX_RC,
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
	ERROR_BASE_SPI2 = (ERROR_BASE_SPI1 + SPI_ERROR_BASE_LAST),
	ERROR_BASE_TIM21 = (ERROR_BASE_SPI2 + SPI_ERROR_BASE_LAST),
	ERROR_BASE_USART = (ERROR_BASE_TIM21 + TIM_ERROR_BASE_LAST),
	// Utils.
	ERROR_BASE_MATH = (ERROR_BASE_USART + USART_ERROR_BASE_LAST),
	ERROR_BASE_PARSER = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
	ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
	// Components.
	ERROR_BASE_DPS310 = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	ERROR_BASE_MAX11136 = (ERROR_BASE_DPS310 + DPS310_ERROR_BASE_LAST),
	ERROR_BASE_NEOM8N = (ERROR_BASE_MAX11136 + MAX11136_ERROR_BASE_LAST),
	ERROR_BASE_SHT3X_INT = (ERROR_BASE_NEOM8N + NEOM8N_ERROR_BASE_LAST),
	ERROR_BASE_SHT3X_EXT = (ERROR_BASE_SHT3X_INT + SHT3X_ERROR_BASE_LAST),
	ERROR_BASE_SI1133 = (ERROR_BASE_SHT3X_EXT + SHT3X_ERROR_BASE_LAST),
	ERROR_BASE_SX1232 = (ERROR_BASE_SI1133 + SI1133_ERROR_BASE_LAST),
	ERROR_BASE_WIND = (ERROR_BASE_SX1232 + SX1232_ERROR_BASE_LAST),
	// Libraries.
	ERROR_BASE_SIGFOX = (ERROR_BASE_WIND + WIND_ERROR_BASE_LAST),
	// Last index.
	ERROR_BASE_LAST
} ERROR_t;

/*** ERROR functions ***/

void ERROR_stack_init(void);
void ERROR_stack_add(ERROR_t status);
void ERROR_stack_read(ERROR_t* error_stack);
unsigned char ERROR_stack_is_empty(void);

#define ERROR_status_check(status, success, error_base) { \
	if (status != success) { \
		ERROR_stack_add(error_base + status); \
	} \
}

#define ERROR_status_check_print(status, success, error_base) { \
	if (status != success) { \
		ERROR_stack_add(error_base + status); \
		AT_print_status(error_base + status); \
		goto errors; \
	} \
}

#endif /* __ERROR_H__ */
