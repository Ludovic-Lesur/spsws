/*
 * adc_reg.h
 *
 *  Created on: 5 may 2018
 *      Author: Ludo
 */

#ifndef __ADC_REG_H__
#define __ADC_REG_H__

#include "types.h"

/*** ADC registers ***/

typedef struct {
	volatile uint32_t ISR;    			// ADC interrupt and status register.
	volatile uint32_t IER;    			// ADC interrupt enable register.
	volatile uint32_t CR;    			// ADC control register.
	volatile uint32_t CFGR1;    		// ADC configuration register 1.
	volatile uint32_t CFGR2;    		// ADC configuration register 2
	volatile uint32_t SMPR;    			// ADC sampling time register.
	volatile uint32_t RESERVED0;		// Reserved 0x18.
	volatile uint32_t RESERVED1;		// Reserved 0x1C.
	volatile uint32_t TR;    			// ADC watchdog threshold register.
	volatile uint32_t RESERVED2;		// Reserved 0x24.
	volatile uint32_t CHSELR;   		// ADC channel selection register.
	volatile uint32_t RESERVED3[5];		// Reserved 0x2C.
	volatile uint32_t DR;    			// ADC data register.
	volatile uint32_t RESERVED4[28];	// Reserved 0x44.
	volatile uint32_t CALFACT;			// ADC calibration factor register.
	volatile uint32_t RESERVED5[148];	// Reserved 0xB8.
	volatile uint32_t CCR;				// ADC common configuration register.
} ADC_registers_t;

/*** ADC base address ***/

#define ADC1	((ADC_registers_t*) ((uint32_t) 0x40012400))

/*** Temperature sensor calibration values address */

#define TS_VCC_CALIB_MV			3000
#define TS_CAL1_ADDR			((uint16_t*) ((uint32_t) 0x1FF8007A))
#define TS_CAL1					((int32_t) (*TS_CAL1_ADDR)) // Raw ADC output value on 12 bits.
#define TS_CAL1_TEMP			((int32_t) 30)

#define TS_CAL2_ADDR			((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TS_CAL2					((int32_t) (*TS_CAL2_ADDR)) // Raw ADC output value on 12 bits.
#define TS_CAL2_TEMP			((int32_t) 130)

/* Internal voltage reference calibration value address */

#define VREFINT_VCC_CALIB_MV	3000
#define VREFINT_CAL_ADDR		((uint16_t*) ((uint32_t) 0x1FF80078))
#define VREFINT_CAL				((uint32_t) (*VREFINT_CAL_ADDR)) // Raw ADC output value on 12 bits.

#endif /* __ADC_REG_H__ */
