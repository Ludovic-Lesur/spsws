/*
 * rtc_reg.h
 *
 *  Created on: 25 nov. 2018
 *      Author: Ludo
 */

#ifndef RTC_REG_H
#define RTC_REG_H

/*** RTC registers ***/

typedef struct {
	volatile unsigned int TR;			// RTC time register.
	volatile unsigned int DR;			// RTC date register.
	volatile unsigned int CR;			// RTC control register.
	volatile unsigned int ISR;			// RTC interrupt and status register.
	volatile unsigned int PRER;			// RTC prescaler register.
	volatile unsigned int WUTR;			// RTC write unlock register.
	unsigned int RESERVED0;				// Reserved 0x18.
	volatile unsigned int ALRMAR;		// RTC alarm A register.
	volatile unsigned int ALRMBR;		// RTC alarm B register.
	volatile unsigned int WPR;			// RTC write protect register.
	volatile unsigned int SSR;			// RTC sub-seconds register.
	volatile unsigned int SHIFTR;		// RTC shift control register.
	volatile unsigned int TSTR;			// RTC timestamp time register.
	volatile unsigned int TSDR;			// RTC timestamp date register.
	volatile unsigned int TSSSR;		// RTC timestamp sub-seconds register.
	volatile unsigned int CALR;			// RTC calibration register.
	volatile unsigned int TAMPCR;		// RTC tamper configuration register.
	volatile unsigned int ALRMASSR;		// RTC alarm A sub-seconds register.
	volatile unsigned int ALRMBSSR;		// RTC alarm B sub-seconds register.
	volatile unsigned int OR;			// RTC option register.
	volatile unsigned int BKP0R;		// RTC back-up register 0.
	volatile unsigned int BKP1R;		// RTC back-up register 1.
	volatile unsigned int BKP2R;		// RTC back-up register 2.
	volatile unsigned int BKP3R;		// RTC back-up register 3.
	volatile unsigned int BKP4R;		// RTC back-up register 4.
} RTC_BaseAddress;

/*** RTC base address ***/

#define RTC		((RTC_BaseAddress*) ((unsigned int) 0x40002800))

#endif /* RTC_REG_H */
