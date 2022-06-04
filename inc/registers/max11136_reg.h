/*
 * max11136_reg.h
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#ifndef __MAX11136_REG_H__
#define __MAX11136_REG_H__

/*** MAX11136 register map ***/

#define MAX11136_REG_ADC_MODE_CONTROL	0b00000
#define MAX11136_REG_ADC_CONFIG			0b10000
#define MAX11136_REG_UNIPOLAR			0b10001
#define MAX11136_REG_BIPOLAR			0b10010
#define MAX11136_REG_RANGE				0b10011
#define MAX11136_REG_CUSTOM_SCAN0		0b10100
#define MAX11136_REG_CUSTOM_SCAN1		0b10101
#define MAX11136_REG_SAMPLE_SET			0b10110

#endif /* __MAX11136_REG_H__ */
