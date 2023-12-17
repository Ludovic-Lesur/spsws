/*
 * dma_reg.h
 *
 *  Created on: 08 may 2018
 *      Author: Ludo
 */

#ifndef __DMA_REG_H__
#define __DMA_REG_H__

#include "types.h"

/*** DMA REG macros ***/

// Peripheral base address.
#define DMA1	((DMA_registers_t*) ((uint32_t) 0x40020000))

/*** DMA REG structures ***/

/*!******************************************************************
 * \enum DMA_registers_t
 * \brief DMA registers map.
 *******************************************************************/
typedef struct {
	volatile uint32_t ISR;			// DMA interrupt status register.
	volatile uint32_t IFCR;			// DMA interrupt flag clear register.
	volatile uint32_t CCR1;			// DMA channel 1 configuration register.
	volatile uint32_t CNDTR1;		// DMA channel 1 number of data register.
	volatile uint32_t CPAR1;		// DMA channel 1 peripheral address register.
	volatile uint32_t CMAR1;    	// DMA channel 1 memory address register.
	volatile uint32_t RESERVED0;	// Reserved 0x18.
	volatile uint32_t CCR2;			// DMA channel 2 configuration register.
	volatile uint32_t CNDTR2;		// DMA channel 2 number of data register.
	volatile uint32_t CPAR2;		// DMA channel 2 peripheral address register.
	volatile uint32_t CMAR2;		// DMA channel 2 memory address register.
	volatile uint32_t RESERVED1;	// Reserved 0x2C.
	volatile uint32_t CCR3;			// DMA channel 3 configuration register.
	volatile uint32_t CNDTR3;		// DMA channel 3 number of data register.
	volatile uint32_t CPAR3;		// DMA channel 3 peripheral address register.
	volatile uint32_t CMAR3;		// DMA channel 3 memory address register.
	volatile uint32_t RESERVED2;	// Reserved 0x40.
	volatile uint32_t CCR4;			// DMA channel 4 configuration register.
	volatile uint32_t CNDTR4;		// DMA channel 4 number of data register.
	volatile uint32_t CPAR4;		// DMA channel 4 peripheral address register.
	volatile uint32_t CMAR4;		// DMA channel 4 memory address register.
	volatile uint32_t RESERVED3;	// Reserved 0x54.
	volatile uint32_t CCR5;			// DMA channel 5 configuration register.
	volatile uint32_t CNDTR5;		// DMA channel 5 number of data register.
	volatile uint32_t CPAR5;		// DMA channel 5 peripheral address register.
	volatile uint32_t CMAR5;		// DMA channel 5 memory address register.
	volatile uint32_t RESERVED4;	// Reserved 0x68.
	volatile uint32_t CCR6;			// DMA channel 6 configuration register.
	volatile uint32_t CNDTR6;		// DMA channel 6 number of data register.
	volatile uint32_t CPAR6;		// DMA channel 6 peripheral address register.
	volatile uint32_t CMAR6;		// DMA channel 6 memory address register.
	volatile uint32_t RESERVED5;	// Reserved 0x7C.
	volatile uint32_t CCR7;			// DMA channel 7 configuration register.
	volatile uint32_t CNDTR7;		// DMA channel 7 number of data register.
	volatile uint32_t CPAR7;		// DMA channel 7 peripheral address register.
	volatile uint32_t CMAR7;		// DMA channel 7 memory address register.
	volatile uint32_t RESERVED6[6];	// Reserved 0x90-0xA7.
	volatile uint32_t CSELR;    	// DMA channel selection register.
} DMA_registers_t;

#endif /* __DMA_REG_H__ */
