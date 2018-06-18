/*
 * dma_reg.h
 *
 *  Created on: 8 may 2018
 *      Author: Ludovic
 */

#ifndef REGISTERS_DMA_REG_H
#define REGISTERS_DMA_REG_H

/*** DMA registers ***/

typedef struct {
	volatile unsigned int ISR;    	// DMA interrupt status register.
	volatile unsigned int IFCR;    	// DMA interrupt flag clear register.
	volatile unsigned int CCR1;    	// DMA channel 1 configuration register.
	volatile unsigned int CNDTR1;   // DMA channel 1 number of data register.
	volatile unsigned int CPAR1;    // DMA channel 1 peripheral address register.
	volatile unsigned int CMAR1;    // DMA channel 1 memory address register.
	unsigned int RESERVED0;    		// Reserved 0x18.
	volatile unsigned int CCR2;    	// DMA channel 2 configuration register.
	volatile unsigned int CNDTR2;   // DMA channel 2 number of data register.
	volatile unsigned int CPAR2;    // DMA channel 2 peripheral address register.
	volatile unsigned int CMAR2;    // DMA channel 2 memory address register.
	unsigned int RESERVED1;    		// Reserved 0x2C.
	volatile unsigned int CCR3;    	// DMA channel 3 configuration register.
	volatile unsigned int CNDTR3;   // DMA channel 3 number of data register.
	volatile unsigned int CPAR3;    // DMA channel 3 peripheral address register.
	volatile unsigned int CMAR3;    // DMA channel 3 memory address register.
	unsigned int RESERVED2;    		// Reserved 0x40.
	volatile unsigned int CCR4;    	// DMA channel 4 configuration register.
	volatile unsigned int CNDTR4;   // DMA channel 4 number of data register.
	volatile unsigned int CPAR4;    // DMA channel 4 peripheral address register.
	volatile unsigned int CMAR4;    // DMA channel 4 memory address register.
	unsigned int RESERVED3;    		// Reserved 0x54.
	volatile unsigned int CCR5;    	// DMA channel 5 configuration register.
	volatile unsigned int CNDTR5;   // DMA channel 5 number of data register.
	volatile unsigned int CPAR5;    // DMA channel 5 peripheral address register.
	volatile unsigned int CMAR5;    // DMA channel 5 memory address register.
	unsigned int RESERVED4;    		// Reserved 0x68.
	volatile unsigned int CCR6;    	// DMA channel 6 configuration register.
	volatile unsigned int CNDTR6;   // DMA channel 6 number of data register.
	volatile unsigned int CPAR6;    // DMA channel 6 peripheral address register.
	volatile unsigned int CMAR6;    // DMA channel 6 memory address register.
	unsigned int RESERVED5;    		// Reserved 0x7C.
	volatile unsigned int CCR7;    	// DMA channel 7 configuration register.
	volatile unsigned int CNDTR7;   // DMA channel 7 number of data register.
	volatile unsigned int CPAR7;    // DMA channel 7 peripheral address register.
	volatile unsigned int CMAR7;    // DMA channel 7 memory address register.
	unsigned int RESERVED6[6];    	// Reserved 0x90-0xA7.
	volatile unsigned int CSELR;    // DMA channel selection register.
} DMA_BaseAddress;

/*** DMA base address ***/

#define DMA1	((DMA_BaseAddress*) ((unsigned int) 0x40020000))

#endif /* REGISTERS_DMA_REG_H */
