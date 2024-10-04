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
#include "dma.h"
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
// Components.
#include "dps310.h"
#include "max111xx.h"
#include "neom8x.h"
#include "sen15901.h"
#include "sht3x.h"
#include "si1133.h"
#include "sx1232.h"
// Middleware.
#include "analog.h"
#include "gps.h"
#include "math.h"
#include "parser.h"
#include "power.h"
#include "rfe.h"
#include "string.h"
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
    ERROR_BASE_ADC = 0x0100,
    ERROR_BASE_AES = (ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
    ERROR_BASE_DMA = (ERROR_BASE_AES + AES_ERROR_BASE_LAST),
    ERROR_BASE_FLASH = (ERROR_BASE_DMA + DMA_ERROR_BASE_LAST),
    ERROR_BASE_I2C_SENSORS = (ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST),
    ERROR_BASE_IWDG = (ERROR_BASE_I2C_SENSORS + I2C_ERROR_BASE_LAST),
    ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
    ERROR_BASE_LPUART = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    ERROR_BASE_NVM = (ERROR_BASE_LPUART + LPUART_ERROR_BASE_LAST),
    ERROR_BASE_RCC = (ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
    ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    ERROR_BASE_SPI_RADIO = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
    ERROR_BASE_SPI_ADC = (ERROR_BASE_SPI_RADIO + SPI_ERROR_BASE_LAST),
    ERROR_BASE_TIM_MCU_API = (ERROR_BASE_SPI_ADC + SPI_ERROR_BASE_LAST),
    ERROR_BASE_TIM_CALIBRATION = (ERROR_BASE_TIM_MCU_API + TIM_ERROR_BASE_LAST),
    ERROR_BASE_TIM_MODULATION = (ERROR_BASE_TIM_CALIBRATION + TIM_ERROR_BASE_LAST),
    ERROR_BASE_USART_AT = (ERROR_BASE_TIM_MODULATION + TIM_ERROR_BASE_LAST),
    // Components.
    ERROR_BASE_DPS310 = (ERROR_BASE_USART_AT + USART_ERROR_BASE_LAST),
    ERROR_BASE_MAX11136 = (ERROR_BASE_DPS310 + DPS310_ERROR_BASE_LAST),
    ERROR_BASE_NEOM8N = (ERROR_BASE_MAX11136 + MAX111XX_ERROR_BASE_LAST),
    ERROR_BASE_SEN15901 = (ERROR_BASE_NEOM8N + NEOM8X_ERROR_BASE_LAST),
    ERROR_BASE_SHT30_INTERNAL = (ERROR_BASE_SEN15901 + SEN15901_ERROR_BASE_LAST),
    ERROR_BASE_SHT30_EXTERNAL = (ERROR_BASE_SHT30_INTERNAL + SHT3X_ERROR_BASE_LAST),
    ERROR_BASE_SI1133 = (ERROR_BASE_SHT30_EXTERNAL + SHT3X_ERROR_BASE_LAST),
    ERROR_BASE_SX1232 = (ERROR_BASE_SI1133 + SI1133_ERROR_BASE_LAST),
    // Middleware.
    ERROR_BASE_ANALOG = (ERROR_BASE_SX1232 + SX1232_ERROR_BASE_LAST),
    ERROR_BASE_GPS = (ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_LAST),
    ERROR_BASE_MATH = (ERROR_BASE_GPS + GPS_ERROR_BASE_LAST),
    ERROR_BASE_PARSER = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
    ERROR_BASE_POWER = (ERROR_BASE_PARSER + PARSER_ERROR_BASE_LAST),
    ERROR_BASE_STRING = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
    // Sigfox.
    ERROR_BASE_RFE = (ERROR_BASE_STRING + STRING_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_LIB = (ERROR_BASE_RFE + RFE_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_ADDON_RFP = (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_LAST * 0x0100)),
    // Last base value.
    ERROR_BASE_LAST = (ERROR_BASE_SIGFOX_EP_ADDON_RFP + 0x0100)
} ERROR_base_t;

#endif /* __ERROR_BASE_H__ */
