/*
 * usart.c
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#include "usart.h"

#include "gpio_reg.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "usart_reg.h"

/*** USART local macros ***/

// Baud rate.
#define USART_BAUD_RATE 		9600
// TX buffer size.
#define USART_TX_BUFFER_SIZE	128

/*** USART local structures ***/

typedef struct {
	unsigned char tx_buf[USART_TX_BUFFER_SIZE]; 	// Transmit buffer.
	unsigned int tx_buf_read_idx; 					// Reading index in TX buffer.
	unsigned int tx_buf_write_idx; 					// Writing index in TX buffer.
} USART_Context;

/*** USART local global variables ***/

static USART_Context usart_ctx;

/*** USART functions ***/

/* CONFIGURE USART PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART_Init(void) {

	/* Init context */
	unsigned int idx = 0;
	for (idx=0 ; idx<USART_TX_BUFFER_SIZE ; idx++) usart_ctx.tx_buf[idx] = 0;
	usart_ctx.tx_buf_write_idx = 0;
	usart_ctx.tx_buf_read_idx = 0;

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 17); // USART2EN='1'.

	/* Configure TX and RX GPIOs */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= ~(0b1111 << 18); // Reset bits 18-21.
	GPIOA -> MODER |= (0b1010 << 18); // Configure PA9 and PA10 as alternate function.
	GPIOA -> AFRH &= 0xFFFFFF00; // Reset bits 8-15.
	GPIOA -> AFRH |= 0x00000044; // Link PA9 and PA10 to AF4.

	/* Configure peripheral */
	USART2 -> CR1 = 0; // Disable peripheral before configuration (UE='0'), 1 stop bit and 8 data bits (M='00').
	USART2 -> CR2 = 0; // 1 stop bit (STOP='00').
	USART2 -> CR3 = 0;
	USART2 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='1').
	USART2 -> BRR = ((SYSCLK_KHZ*1000)/(USART_BAUD_RATE)); // BRR = (fCK)/(baud rate). See p.730 of RM0377 datasheet.
	USART2 -> CR1 &= ~(0b11 << 2); // Disable transmitter (TE='0') and receiver (RE='0') by default.

	/* Enable transmitter */
	USART2 -> CR1 |= (0b1 << 3); // TE='1'.
	USART2 -> CR1 |= (0b1 << 7); // TXEIE='1'.
	NVIC_EnableInterrupt(IT_USART2);

	/* Enable peripheral */
	USART2 -> CR1 |= (0b1 << 0);
}

/* SEND A BYTE THROUGH USART.
 * @param byte_to_send:	Byte to send.
 * @return:				None.
 */
void USART_SendByte(unsigned char byte_to_send) {

	/* Fill TX buffer with new byte */
	usart_ctx.tx_buf[usart_ctx.tx_buf_write_idx] = byte_to_send;
	usart_ctx.tx_buf_write_idx++;
	if (usart_ctx.tx_buf_write_idx == USART_TX_BUFFER_SIZE) {
		usart_ctx.tx_buf_write_idx = 0;
	}

	/* Enable TXE interrupt */
	USART2 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
}

/* SEND A BYTE ARRAY THROUGH USART.
 * @param string_to_send:	Byte array to send.
 * @return:					None.
 */
void USART_SendString(char* string_to_send) {
	/* Fill TX buffer with new bytes */
	while (*string_to_send) {
		USART_SendByte(*(string_to_send++));
	}
}

/* USART2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void USART2_IRQHandler(void) {

	/* TXE interrupt */
	if (((USART2 -> ISR) & (0b1 << 7)) != 0) {
		if ((usart_ctx.tx_buf_read_idx) != (usart_ctx.tx_buf_write_idx)) {
			USART2 -> TDR = (usart_ctx.tx_buf)[usart_ctx.tx_buf_read_idx]; // Fill transmit data register with new byte.
			usart_ctx.tx_buf_read_idx++; // Increment TX read index.
			if (usart_ctx.tx_buf_read_idx == USART_TX_BUFFER_SIZE) {
				usart_ctx.tx_buf_read_idx = 0; // Manage roll-over.
			}
		}
		else {
			// No more bytes, disable TXE interrupt.
			USART2 -> CR1 &= ~(0b1 << 7); // TXEIE = '0'.
		}
	}

	/* Overrun error interrupt */
	if (((USART2 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART2 -> ICR |= (0b1 << 3);
	}
}
