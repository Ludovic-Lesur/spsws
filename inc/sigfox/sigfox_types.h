/*!
 \if SIGFOX PATTERN
 ----------------------------------------------

   _____   _   _____   _____   _____  __    __      
  /  ___/ | | /  ___| |  ___| /  _  \ \ \  / /      
  | |___  | | | |     | |__   | | | |  \ \/ /       
  \___  \ | | | |  _  |  __|  | | | |   }  {        
   ___| | | | | |_| | | |     | |_| |  / /\ \
  /_____/ |_| \_____/ |_|     \_____/ /_/  \_\

  ----------------------------------------------

    !!!!  DO NOT MODIFY THIS FILE !!!!

  ----------------------------------------------
 \endif
  ----------------------------------------------*/
/*!
 * \file sigfox_types.h
 * \brief Sigfox types definition
 * \author $(SIGFOX_LIB_AUTHOR)
 * \version $(SIGFOX_LIB_VERSION)
 * \date $(SIGFOX_LIB_DATE)
 * \copyright Copyright (c) 2011-2015 SIGFOX, All Rights Reserved. This is unpublished proprietary source code of SIGFOX.
 *
 */

#ifndef SIGFOX_TYPES_H
#define SIGFOX_TYPES_H

#include "adc.h"
#include "aes.h"
#include "lptim.h"
#include "nvm.h"
#include "rtc.h"
#include "spi.h"
#include "sx1232.h"

/****************************************************/
/*!
 * \defgroup SIGFOX_TYPES Custom types used in library
 *           TAKE CARE that these definitions have to be adapted to your compiler / target
 *  @{
 */
/* Unsigned Type*/
typedef unsigned char       sfx_u8;
typedef unsigned short      sfx_u16;
typedef unsigned long       sfx_u32;
typedef unsigned char       sfx_bool;
/* Signed Type */
typedef signed char         sfx_s8;
typedef signed short        sfx_s16;
typedef signed long         sfx_s32;
/* Custom Types */
typedef unsigned short      sfx_error_t;

typedef enum {
	SIGFOX_API_SUCCESS = 0,
	SIGFOX_API_ERROR_MALLOC,
	SIGFOX_API_ERROR_DELAY_TYPE,
	SIGFOX_API_ERROR_KEY_TYPE,
	SIGFOX_API_ERROR_MODULATION_TYPE,
	SIGFOX_API_ERROR_RF_MODE,
	SIGFOX_API_ERROR_BASE_ADC = 0x0100,
	SIGFOX_API_ERROR_BASE_LPTIM = (SIGFOX_API_ERROR_BASE_ADC + ADC_ERROR_BASE_LAST),
	SIGFOX_API_ERROR_BASE_NVM = (SIGFOX_API_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	SIGFOX_API_ERROR_BASE_AES = (SIGFOX_API_ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
	SIGFOX_API_ERROR_BASE_RTC = (SIGFOX_API_ERROR_BASE_AES + AES_ERROR_BASE_LAST),
	SIGFOX_API_ERROR_BASE_SPI = (SIGFOX_API_ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
	SIGFOX_API_ERROR_BASE_SX1232 = (SIGFOX_API_ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	SIGFOX_API_ERROR_BASE_LAST = (SIGFOX_API_ERROR_BASE_SX1232 + SX1232_ERROR_BASE_LAST)
} SIGFOX_API_status_t;

#define SFX_NULL            (void*)0
#define SFX_TRUE            (sfx_bool)(1)
#define SFX_FALSE           (sfx_bool)(0)
#define SFX_U8_1            (sfx_u8)(1)
#define SFX_U8_0            (sfx_u8)(0)

#define SFX_U8_MIN          0U
#define SFX_U8_MAX          0xFFU
#define SFX_U16_MIN         0U
#define SFX_U16_MAX         0xFFFFU
#define SFX_U32_MIN         0UL
#define SFX_U32_MAX         0xFFFFFFFFUL

#define SFX_S8_MIN          0x80
#define SFX_S8_MAX          0x7F
#define SFX_S16_MIN         0x8000
#define SFX_S16_MAX         0x7FFF
#define SFX_S32_MIN         0x80000000L
#define SFX_S32_MAX         0x7FFFFFFFL

/*
 ** @}
 ***************************************************
 */

#endif /* SIGFOX_TYPES_H */
