/*
 * rtc_reg.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#ifndef __RTC_REG_H__
#define __RTC_REG_H__

#include "types.h"

/*** RTC registers ***/

typedef struct {
	volatile uint32_t TR;			// RTC time register.
	volatile uint32_t DR;			// RTC date register.
	volatile uint32_t CR;			// RTC control register.
	volatile uint32_t ISR;			// RTC interrupt and status register.
	volatile uint32_t PRER;			// RTC prescaler register.
	volatile uint32_t WUTR;			// RTC write unlock register.
	volatile uint32_t RESERVED0;	// Reserved 0x18.
	volatile uint32_t ALRMAR;		// RTC alarm A register.
	volatile uint32_t ALRMBR;		// RTC alarm B register.
	volatile uint32_t WPR;			// RTC write protect register.
	volatile uint32_t SSR;			// RTC sub-seconds register.
	volatile uint32_t SHIFTR;		// RTC shift control register.
	volatile uint32_t TSTR;			// RTC timestamp time register.
	volatile uint32_t TSDR;			// RTC timestamp date register.
	volatile uint32_t TSSSR;		// RTC timestamp sub-seconds register.
	volatile uint32_t CALR;			// RTC calibration register.
	volatile uint32_t TAMPCR;		// RTC tamper configuration register.
	volatile uint32_t ALRMASSR;		// RTC alarm A sub-seconds register.
	volatile uint32_t ALRMBSSR;		// RTC alarm B sub-seconds register.
	volatile uint32_t OR;			// RTC option register.
	volatile uint32_t BKP0R;		// RTC back-up register 0.
	volatile uint32_t BKP1R;		// RTC back-up register 1.
	volatile uint32_t BKP2R;		// RTC back-up register 2.
	volatile uint32_t BKP3R;		// RTC back-up register 3.
	volatile uint32_t BKP4R;		// RTC back-up register 4.
} RTC_base_address_t;

/*** RTC base address ***/

#define RTC		((RTC_base_address_t*) ((uint32_t) 0x40002800))

#endif /* __RTC_REG_H__ */
