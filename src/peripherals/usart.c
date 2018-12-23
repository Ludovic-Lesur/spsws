/*
 * usart.c
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#include "usart.h"

#include "at.h"
#include "gpio.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim.h"
#include "usart_reg.h"

/*** USART local macros ***/

// If defined, use TXE interrupt for sending data.
//#define USE_TXE_INTERRUPT
// Baud rate.
#define USART_BAUD_RATE 		9600
// TX buffer size.
#define USART_TX_BUFFER_SIZE	512

/*** USART local structures ***/

#ifdef USE_TXE_INTERRUPT
typedef struct {
	unsigned char tx_buf[USART_TX_BUFFER_SIZE]; 	// Transmit buffer.
	unsigned int tx_buf_read_idx; 					// Reading index in TX buffer.
	unsigned int tx_buf_write_idx; 					// Writing index in TX buffer.
} USART_Context;
#endif

/*** USART local global variables ***/

#ifdef USE_TXE_INTERRUPT
static USART_Context usart_ctx;
#endif

/*** USART local functions ***/

/* USART2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void USART2_IRQHandler(void) {

#ifdef USE_TXE_INTERRUPT
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
#endif

	/* RXNE interrupt */
	if (((USART2 -> ISR) & (0b1 << 5)) != 0) {
		// Transmit incoming byte to AT command manager.
		AT_FillRxBuffer(USART2 -> RDR);
	}

	/* Overrun error interrupt */
	if (((USART2 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART2 -> ICR |= (0b1 << 3);
	}
}

/* FILL USART2 TX BUFFER WITH A NEW BYTE.
 * @param tx_byte:	Byte to append.
 * @return:			None.
 */
void USART2_FillTxBuffer(unsigned char tx_byte) {
#ifdef USE_TXE_INTERRUPT
	// Fill buffer.
	usart_ctx.tx_buf[usart_ctx.tx_buf_write_idx] = tx_byte;
	// Manage index roll-over.
	usart_ctx.tx_buf_write_idx++;
	if (usart_ctx.tx_buf_write_idx == USART_TX_BUFFER_SIZE) {
		usart_ctx.tx_buf_write_idx = 0;
	}
#else
	// Fill transmit register.
	USART2 -> TDR = tx_byte;
	// Wait for transmission to complete.
	while (((USART2 -> ISR) & (0b1 << 7)) == 0); // Wait for TXE='1'.
#endif
}

/* CONVERTS A 4-BIT WORD TO THE ASCII CODE OF THE CORRESPONDING HEXADECIMAL CHARACTER.
 * @param n:	The word to converts.
 * @return:		The results of conversion.
 */
char USART_HexaToAscii(unsigned char hexa_value) {
	char hexa_ascii = 0;
	if (hexa_value <= 15) {
		hexa_ascii = (hexa_value <= 9 ? (char) (hexa_value + '0') : (char) (hexa_value + ('A' - 10)));
	}
	return hexa_ascii;
}

/* COMPUTE A POWER A 10.
 * @param power:	The desired power.
 * @return result:	Result of computation.
 */
unsigned int USART_Pow10(unsigned char power) {
	unsigned int result = 0;
	unsigned int pow10_buf[10] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
	if (power <= 9) {
		result = pow10_buf[power];
	}
	return result;
}

/*** USART functions ***/

/* CONFIGURE USART2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART2_Init(void) {

#ifdef USE_TXE_INTERRUPT
	/* Init context */
	unsigned int idx = 0;
	for (idx=0 ; idx<USART_TX_BUFFER_SIZE ; idx++) usart_ctx.tx_buf[idx] = 0;
	usart_ctx.tx_buf_write_idx = 0;
	usart_ctx.tx_buf_read_idx = 0;
#endif

	/* Enable peripheral clock */
	RCC -> APB1ENR |= (0b1 << 17); // USART2EN='1'.

	/* Configure TX and RX GPIOs (first as high impedance) */
	GPIO_Configure(GPIO_USART2_TX, Input, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Configure(GPIO_USART2_RX, Input, PushPull, LowSpeed, NoPullUpNoPullDown);

	/* Configure peripheral */
	USART2 -> CR1 = 0; // Disable peripheral before configuration (UE='0'), 1 stop bit and 8 data bits (M='00').
	USART2 -> CR2 = 0; // 1 stop bit (STOP='00').
	USART2 -> CR3 = 0;
	USART2 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='1').
	USART2 -> BRR = ((SYSCLK_KHZ*1000) / (USART_BAUD_RATE)); // BRR = (fCK)/(baud rate). See p.730 of RM0377 datasheet.
	USART2 -> CR1 &= ~(0b11 << 2); // Disable transmitter (TE='0') and receiver (RE='0') by default.

	/* Enable transmitter and receiver */
	USART2 -> CR1 |= (0b11 << 2); // TE='1' and RE='1'.
	USART2 -> CR1 |= (0b1 << 5); // RXNEIE='1'.

	/* Disable peripheral by default */
	RCC -> APB1ENR &= ~(0b1 << 17); // USART2EN='0'.
}

/* ENABLE USART2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART2_Enable(void) {

	/* Enable USART2 peripheral */
	RCC -> APB1ENR |= (0b1 << 17); // USART2EN='1'.
	USART2 -> CR1 |= (0b1 << 0);
	NVIC_EnableInterrupt(IT_USART2);
}

/* DISABLE USART2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART2_Disable(void) {

	/* Disable USART2 peripheral */
	NVIC_DisableInterrupt(IT_USART2);
	USART2 -> CR1 &= ~(0b1 << 0);
	RCC -> APB1ENR |= (0b1 << 17); // USART2EN='1'.
}

/* SWITCH USART2 SLAVE ON.
 * @param:	None.
 * @return:	None.
 */
void USART2_PowerOn(void) {

	/* Switch MCP2221A on */
	GPIO_Write(GPIO_DEBUG_POWER_ENABLE, 1);
	TIM22_WaitMilliseconds(100);

	/* Enable LPUART alternate function */
	GPIO_Configure(GPIO_USART2_TX, AlternateFunction, PushPull, HighSpeed, NoPullUpNoPullDown);
	GPIO_Configure(GPIO_USART2_RX, AlternateFunction, PushPull, HighSpeed, NoPullUpNoPullDown);
}

/* SWITCH USART2 SLAVE OFF.
 * @param:	None.
 * @return:	None.
 */
void USART2_PowerOff(void) {

	/* Disable LPUART alternate function */
	GPIO_Configure(GPIO_USART2_TX, Input, PushPull, LowSpeed, NoPullUpNoPullDown);
	GPIO_Configure(GPIO_USART2_RX, Input, PushPull, LowSpeed, NoPullUpNoPullDown);

	/* Switch MCP2221A off */
	GPIO_Write(GPIO_DEBUG_POWER_ENABLE, 0);
}

/* SEND A BYTE THROUGH USART2.
 * @param byte_to_send:	The byte to send.
 * @param format:		Display format (see ByteDisplayFormat enumeration in usart.h).
 * @param print_prexix	Print '0b' or '0x' prefix is non zero.
 * @return: 			None.
 */
void USART2_SendValue(unsigned int tx_value, USART_Format format, unsigned char print_prefix) {
	// Disable interrupt.
	NVIC_DisableInterrupt(IT_USART2);
	// Common variables.
	unsigned char first_non_zero_found = 0;
	unsigned int idx;
	unsigned char current_value = 0;
	unsigned int current_power = 0;
	unsigned int previous_decade = 0;
	// Fill TX buffer according to format.
	switch (format) {
	case USART_FORMAT_BINARY:
		if (print_prefix != 0) {
			// Print "0b" prefix.
			USART2_FillTxBuffer('0');
			USART2_FillTxBuffer('b');
		}
		// Maximum 32 bits.
		for (idx=31 ; idx>=0 ; idx--) {
			if (tx_value & (0b1 << idx)) {
				USART2_FillTxBuffer('1'); // = '1'.
				first_non_zero_found = 1;
			}
			else {
				if ((first_non_zero_found != 0) || (idx == 0)) {
					USART2_FillTxBuffer('0'); // = '0'.
				}
			}
			if (idx == 0) {
				break;
			}
		}
		break;
	case USART_FORMAT_HEXADECIMAL:
		if (print_prefix != 0) {
			// Print "0x" prefix.
			USART2_FillTxBuffer('0');
			USART2_FillTxBuffer('x');
		}
		// Maximum 4 bytes.
		for (idx=3 ; idx>=0 ; idx--) {
			current_value = (tx_value & (0xFF << (8*idx))) >> (8*idx);
			if (current_value != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				USART2_FillTxBuffer(USART_HexaToAscii((current_value & 0xF0) >> 4));
				USART2_FillTxBuffer(USART_HexaToAscii(current_value & 0x0F));
			}
			if (idx == 0) {
				break;
			}
		}
		break;
	case USART_FORMAT_DECIMAL:
		// Maximum 10 digits.
		for (idx=9 ; idx>=0 ; idx--) {
			current_power = USART_Pow10(idx);
			current_value = (tx_value - previous_decade) / current_power;
			previous_decade += current_value * current_power;
			if (current_value != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				USART2_FillTxBuffer(current_value + '0');
			}
			if (idx == 0) {
				break;
			}
		}
		break;
	case USART_FORMAT_ASCII:
		// Raw byte.
		if (tx_value <= 0xFF) {
			USART2_FillTxBuffer(tx_value);
		}
		break;
	}
	// Enable interrupt.
#ifdef USE_TXE_INTERRUPT
	USART2 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
#endif
	NVIC_EnableInterrupt(IT_USART2);
}

/* SEND A BYTE ARRAY THROUGH USART2.
 * @param tx_string:	Byte array to send.
 * @return:				None.
 */
void USART2_SendString(char* tx_string) {
	// Disable interrupt.
	NVIC_DisableInterrupt(IT_USART2);
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		USART2_FillTxBuffer((unsigned char) *(tx_string++));
	}
	// Enable interrupt.
#ifdef USE_TXE_INTERRUPT
	USART2 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
#endif
	NVIC_EnableInterrupt(IT_USART2);
}
