/*
 * adc_reg.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludovic
 */

#ifndef REGISTERS_ADC_REG_H
#define REGISTERS_ADC_REG_H

/*** ADC registers ***/

typedef struct {
	volatile unsigned int ISR;    	// ADC interrupt and status register.
	volatile unsigned int IER;    	// ADC interrupt enable register.
	volatile unsigned int CR;    	// ADC control register.
	volatile unsigned int CFGR1;    // ADC configuration register 1.
	volatile unsigned int CFGR2;    // ADC configuration register 2
	volatile unsigned int SMPR;    	// ADC sampling time register.
	unsigned int RESERVED0;			// Reserved 0x18.
	unsigned int RESERVED1;			// Reserved 0x1C.
	volatile unsigned int TR;    	// ADC watchdog threshold register.
	unsigned int RESERVED2;			// Reserved 0x24.
	volatile unsigned int CHSELR;   // ADC channel selection register.
	unsigned int RESERVED3[5];		// Reserved 0x2C.
	volatile unsigned int DR;    	// ADC data register.
	unsigned int RESERVED4[28];		// Reserved 0x44.
	volatile unsigned int CALFACT;  // ADC calibration factor register.
	unsigned int RESERVED5[148];	// Reserved 0xB8.
	volatile unsigned int CCR;    	// ADC common configuration register.
} ADC_BaseAddress;

/*** ADC base address ***/

#define ADC1	((ADC_BaseAddress*) ((unsigned int) 0x40012400))

/*** Temperature sensor calibration values address */

#define TS_VCC_CALIB_MV			3000
#define TS_CAL1_ADDR			((unsigned short*) ((unsigned int) 0x1FF8007A))
#define TS_CAL1					((int) (*TS_CAL1_ADDR)) // Raw ADC output value on 12 bits.
#define TS_CAL1_TEMP			((int) 30)

#define TS_CAL2_ADDR			((unsigned short*) ((unsigned int) 0x1FF8007E))
#define TS_CAL2					((int) (*TS_CAL2_ADDR)) // Raw ADC output value on 12 bits.
#define TS_CAL2_TEMP			((int) 130)

/* Internal voltage reference calibration value address */

#define VREFINT_VCC_CALIB_MV	3000
#define VREFINT_CAL_ADDR		((unsigned short*) ((unsigned int) 0x1FF80078))
#define VREFINT_CAL				((unsigned int) (*VREFINT_CAL_ADDR)) // Raw ADC output value on 12 bits.

#endif /* REGISTERS_ADC_REG_H */
