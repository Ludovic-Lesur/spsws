/*
 * dma.c
 *
 *  Created on: 08 may 2018
 *      Author: Ludo
 */

#include "dma.h"

#include "dma_reg.h"
#include "lpuart_reg.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "types.h"

/*** DMA local global variables ***/

static DMA_transfer_complete_irq_cb_t dma1_ch6_tc_irq_callback = NULL;

/*** DMA local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_Channel4_5_6_7_IRQHandler(void) {
	// Transfer complete interrupt (TCIF6='1').
	if (((DMA1 -> ISR) & (0b1 << 21)) != 0) {
		// Call transfer complete callback.
		if ((((DMA1 -> CCR6) & (0b1 << 1)) != 0) && (dma1_ch6_tc_irq_callback != NULL)) {
			dma1_ch6_tc_irq_callback();
		}
		// Clear flag.
		DMA1 -> IFCR |= (0b1 << 21); // CTCIF6='1'.
	}
}

/*** DMA functions ***/

/*******************************************************************/
void DMA1_CH6_init(DMA_transfer_complete_irq_cb_t irq_callback) {
	// Enable peripheral clock.
	RCC -> AHBENR |= (0b1 << 0); // DMAEN='1'.
	// Memory and peripheral data size are 8 bits (MSIZE='00' and PSIZE='00').
	// Disable memory to memory mode (MEM2MEM='0').
	// Peripheral increment mode disabled (PINC='0').
	// Circular mode disabled (CIRC='0').
	// Read from peripheral (DIR='0').
	// Very high priority (PL='11').
	// Memory increment mode enabled (MINC='1').
	// Enable transfer complete interrupt (TCIE='1').
	DMA1 -> CCR6 |= (0b11 << 12) | (0b1 << 7) | (0b1 << 1);
	// Configure peripheral address.
	DMA1 -> CPAR6 = (uint32_t) &(LPUART1 -> RDR); // Peripheral address = LPUART RX register.
	// Configure channel 3 for LPUART1 RX (request number 5).
	DMA1 -> CSELR |= (0b0101 << 20); // DMA channel mapped on LPUART1_RX (C6S='0101').
	// Register callback.
	dma1_ch6_tc_irq_callback = irq_callback;
}

/*******************************************************************/
void DMA1_CH6_de_init(void) {
	// Disable channel.
	DMA1 -> CCR6 &= ~(0b1 << 0); // EN='0'.
	// Disable peripheral clock.
	RCC -> AHBENR &= ~(0b1 << 0); // DMAEN='0'.
}

/*******************************************************************/
void DMA1_CH6_start(void) {
	// Clear all flags.
	DMA1 -> IFCR |= 0x00F00000;
	NVIC_enable_interrupt(NVIC_INTERRUPT_DMA1_CH_4_7, NVIC_PRIORITY_DMA1_CH_4_7);
	// Start transfer.
	DMA1 -> CCR6 |= (0b1 << 0); // EN='1'.
}

/*******************************************************************/
void DMA1_CH6_stop(void) {
	// Stop transfer.
	DMA1 -> CCR6 &= ~(0b1 << 0); // EN='0'.
	NVIC_disable_interrupt(NVIC_INTERRUPT_DMA1_CH_4_7);
}

/*******************************************************************/
void DMA1_CH6_set_destination_address(uint32_t destination_buffer_addr, uint16_t destination_buffer_size) {
	// Set address and buffer size.
	DMA1 -> CMAR6 = destination_buffer_addr;
	DMA1 -> CNDTR6 = destination_buffer_size;
}
