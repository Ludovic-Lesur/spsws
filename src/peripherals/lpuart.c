/*
 * lpuart.c
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#include "gpio_reg.h"
#include "gps.h"
#include "lpuart.h"
#include "lpuart_reg.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"

/*** USART local macros ***/

//#define USE_TXE	// If defined, TX bytes are bufferised and transmitted under interrupt.

// Baud rate.
#define LPUART_BAUD_RATE 		9600
// Buffer sizes.
#ifdef USE_TXE
#define LPUART_TX_BUFFER_SIZE	512
#endif
#define LPUART_RX_BUFFER_SIZE	128

/*** LPUART local structures ***/

#ifdef USE_TXE
typedef struct {
	unsigned char tx_buf[LPUART_TX_BUFFER_SIZE]; 	// Transmit buffer.
	unsigned int tx_buf_read_idx; 					// Reading index in TX buffer.
	unsigned int tx_buf_write_idx; 					// Writing index in TX buffer.
} LPUART_Context;


/*** LPUART local global variables ***/

static LPUART_Context lpuart_ctx;
#endif

/*** LPUART functions ***/

/* CONFIGURE LOW POWER UART.
 * @param:	None.
 * @return:	None.
 */
void LPUART_Init(void) {

#ifdef USE_TXE
	/* Init context */
	unsigned int idx = 0;
	for (idx=0 ; idx<LPUART_TX_BUFFER_SIZE ; idx++) lpuart_ctx.tx_buf[idx] = 0;
	lpuart_ctx.tx_buf_write_idx = 0;
	lpuart_ctx.tx_buf_read_idx = 0;
#endif

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 18); // LPUARTEN='1'.

	/* Configure TX and RX GPIOs */
	RCC -> IOPENR |= (0b1 << 0); // Enable GPIOA clock.
	GPIOA -> MODER &= ~(0b1111 << 4); // Reset bits 4-7.
	GPIOA -> MODER |= (0b1010 << 4); // Configure PA2 and PA3 as alternate function.
	GPIOA -> AFRL &= 0xFFFF00FF; // Reset bits 8-15.
	GPIOA -> AFRL |= 0x00006600; // Link PA2 and PA3 to AF6.

	/* Configure peripheral */
	LPUART1 -> CR1 = 0; // Disable peripheral before configuration (UE='0'), 1 stop bit and 8 data bits (M = '00').
	LPUART1 -> CR2 = 0; // 1 stop bit (STOP='00').
	LPUART1 -> CR3 = 0;
	LPUART1 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='0').
	LPUART1 -> BRR = ((RCC_GetSysclkKhz()*1000)/(LPUART_BAUD_RATE))*256; // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
	LPUART1 -> CR1 &= ~(0b11 << 2); // Disable transmitter (TE='0') and receiver (RE='0') by default.

#ifdef USE_DMA
	/* Configure DMA and CM interrupt */
	LPUART1 -> CR2 &= (0xFF << 24); // Reset bits 24-31.
	LPUART1 -> CR2 |= (NMEA_LF << 24); // LF character used to trigger CM interrupt.
	LPUART1 -> CR3 |= (0b1 << 6); // Enable DMA transfer for reception (DMAR='1'). Transfer is performed after each RXNE event (see p.738 of RM0377 datasheet).
#endif

	/* Enable peripheral */
	LPUART1 -> CR1 |= (0b1 << 0);
}

/* DISABLE LOW POWER UART.
 * @param:	None.
 * @return:	None.
 */
void LPUART_Off(void) {

	/* Disable interrupts */
	NVIC_DisableInterrupt(IT_LPUART1);

	/* Switch peripheral off */
	LPUART1 -> CR1 &= ~(0b11 << 2); // Disable transmitter (TE='0') and receiver (RE='0').
	LPUART1 -> CR1 &= ~(0b1 << 0); // Disable peripheral.

	/* Disable peripheral clock */
	RCC -> APB1ENR &= ~(0b1 << 18); // (LPUARTEN='0').

	/* Put GPIOs in reset state */
	GPIOA -> MODER &= ~(0b1111 << 4); // Configure PA2 and PA3 as input.
}

/* ENABLE THE LPUART TRANSMITTER.
 * @param:	None.
 * @return:	None.
 */
void LPUART_EnableTx(void) {
		LPUART1 -> CR1 |= (0b1 << 3); // Enable transmitter (TE='1').
#ifdef USE_TXE
		LPUART1 -> CR1 |= (0b1 << 7); // Enable TXE interrupt (TXEIE='1').
		NVIC_EnableInterrupt(IT_LPUART1);
#endif
}

/* ENABLE THE LPUART RECEIVER.
 * @param:	None.
 * @return:	None.
 */
void LPUART_EnableRx(void) {
	LPUART1 -> CR1 |= (0b1 << 2); // Enable receiver (RE='1').
#ifdef USE_DMA
	LPUART1 -> CR1 |= (0b1 << 14); // Enable CM interrupt (CMIE='1').
#else
	LPUART1 -> CR1 |= (0b1 << 5); // Enable RXNE interrupt (RXNEIE='1').
#endif
	NVIC_EnableInterrupt(IT_LPUART1);
}

/* SEND A BYTE THROUGH LOW POWER UART.
 * @param byte_to_send:	Byte to send.
 * @return:				None.
 */
void LPUART_SendByte(unsigned char byte_to_send) {

#ifdef USE_TXE
	/* Fill TX buffer with new byte */
	lpuart_ctx.tx_buf[lpuart_ctx.tx_buf_write_idx] = byte_to_send;
	lpuart_ctx.tx_buf_write_idx++;
	if (lpuart_ctx.tx_buf_write_idx == LPUART_TX_BUFFER_SIZE) {
		lpuart_ctx.tx_buf_write_idx = 0;
	}

	/* Enable TXE interrupt */
	LPUART1 -> CR1 |= (0b1 << 7); // (TXEIE = '1').

#else

	/* Fill transmit data register with new byte */
	LPUART1 -> TDR = byte_to_send;

	/* Wait for transfer to complete */
	while (((LPUART1 -> ISR) & (0b1 << 6)) == 0); // (TC='1').

#endif
}

/* SEND A BYTE ARRAY THROUGH LOW POWER UART.
 * @param string_to_send:	Byte array to send.
 * @return:					None.
 */
void LPUART_SendString(char* string_to_send) {

	/* Fill TX buffer with new bytes */
	while (*string_to_send) {
		LPUART_SendByte(*(string_to_send++));
	}
}

/* LPUART1 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void LPUART1_IRQHandler(void) {

#ifdef USE_TXE
	/* TXE interrupt */
	if (((LPUART1 -> ISR) & (0b1 << 7)) != 0) {
		if ((lpuart_ctx.tx_buf_read_idx) != (lpuart_ctx.tx_buf_write_idx)) {
			LPUART1 -> TDR = (lpuart_ctx.tx_buf)[lpuart_ctx.tx_buf_read_idx]; // Fill transmit data register with new byte.
			lpuart_ctx.tx_buf_read_idx++; // Increment TX read index.
			if (lpuart_ctx.tx_buf_read_idx == LPUART_TX_BUFFER_SIZE) {
				lpuart_ctx.tx_buf_read_idx = 0; // Manage roll-over.
			}
		}
		else {
			LPUART1 -> CR1 &= ~(0b1 << 7); // No more bytes, disable TXE interrupt (TXEIE = '0').
		}
	}
#endif

#ifdef USE_DMA
	/* CM interrupt */
	if (((LPUART1 -> ISR) & (0b1 << 17)) != 0) {
		GPS_SwitchDmaBuffer(); // Tell GPS module to switch DMA buffer address and start decoding the current one.
		LPUART1 -> ICR |= (0b1 << 17); // Clear CM flag.
	}
#else
	/* RXNE interrupt */
	if (((LPUART1 -> ISR) & (0b1 << 5)) != 0) {
		GPS_FillNmeaRxBuffer(LPUART1 -> RDR); // Read receive data register and transmit incoming byte to GPS NMEA RX buffer.
	}
#endif
}
