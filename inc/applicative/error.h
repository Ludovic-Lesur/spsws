/*
 * error.h
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#ifndef ERROR_H
#define ERROR_H

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
#include "spi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
// Components.
#include "dps310.h"
#include "max11136.h"
#include "neom8n.h"
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
#include "wind.h"
// Utils.
#include "math.h"
#include "parser.h"
#include "string.h"

/*** ERROR macros ***/

#define ERROR_STACK_DEPTH	6

/*** ERROR structures ***/

typedef enum {
	SUCCESS = 0,
	ERROR_BUSY,
	ERROR_SIGFOX_RC,
	// Peripherals.
	ERROR_BASE_ADC = 0x0100,
	ERROR_BASE_AES = (ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	ERROR_BASE_FLASH = (ERROR_BASE_AES + AES_ERROR_BASE_LAST),
	ERROR_BASE_I2C = (ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
	ERROR_BASE_IWDG = (ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
	ERROR_BASE_LPUART = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	ERROR_BASE_NVM = (ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
	ERROR_BASE_RCC = (ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
	ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
	ERROR_BASE_SPI = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	ERROR_BASE_TIM = (ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	ERROR_BASE_USART = (ERROR_BASE_TIM + TIM_ERROR_BASE_LAST),
	// Utils.
	ERROR_BASE_MATH = (ERROR_BASE_USART + USART_ERROR_BASE_LAST),
	ERROR_BASE_PARSER = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
	ERROR_BASE_STRING = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
	// Components.
	ERROR_BASE_DPS310 = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
	ERROR_BASE_MAX11136 = (ERROR_BASE_DPS310 + DPS310_ERROR_BASE_LAST),
	ERROR_BASE_NEOM8N = (ERROR_BASE_MAX11136 + MAX11136_ERROR_BASE_LAST),
	ERROR_BASE_SHT3X = (ERROR_BASE_NEOM8N + NEOM8N_ERROR_BASE_LAST),
	ERROR_BASE_SI1133 = (ERROR_BASE_SHT3X + SHT3X_ERROR_BASE_LAST),
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

#ifdef ATM
#define ERROR_status_check(status, success, error_base) { \
	if (status != success) { \
		ERROR_stack_add(error_base + status); \
		AT_print_status(error_base + status); \
		goto errors; \
	} \
}
#else
#define ERROR_status_check(status, success, error_base) { \
	if (status != success) { \
		ERROR_stack_add(error_base + status); \
		goto errors; \
	} \
}
#endif

#endif /* ERROR_H */
