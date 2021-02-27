/*
 * usart.c
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#include "usart.h"

#include "at.h"
#include "gpio.h"
#include "lptim.h"
#include "mode.h"
#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim.h"
#include "usart_reg.h"

#ifdef ATM
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
static volatile USART_Context usart_ctx;
#endif

/*** USART local functions ***/

#ifdef HW1_0
/* USART2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void USART2_IRQHandler(void) {

#ifdef USE_TXE_INTERRUPT
	// TXE interrupt.
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
	// RXNE interrupt.
	if (((USART2 -> ISR) & (0b1 << 5)) != 0) {
		// Transmit incoming byte to AT command manager.
		AT_FillRxBuffer(USART2 -> RDR);
		// Clear RXNE flag.
		USART2 -> RQR |= (0b1 << 3);
	}
	// Overrun error interrupt.
	if (((USART2 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART2 -> ICR |= (0b1 << 3);
	}
}
#endif

#ifdef HW2_0
/* USART2 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void USART1_IRQHandler(void) {

#ifdef USE_TXE_INTERRUPT
	// TXE interrupt.
	if (((USART1 -> ISR) & (0b1 << 7)) != 0) {
		if ((usart_ctx.tx_buf_read_idx) != (usart_ctx.tx_buf_write_idx)) {
			USART1 -> TDR = (usart_ctx.tx_buf)[usart_ctx.tx_buf_read_idx]; // Fill transmit data register with new byte.
			usart_ctx.tx_buf_read_idx++; // Increment TX read index.
			if (usart_ctx.tx_buf_read_idx == USART_TX_BUFFER_SIZE) {
				usart_ctx.tx_buf_read_idx = 0; // Manage roll-over.
			}
		}
		else {
			// No more bytes, disable TXE interrupt.
			USART1 -> CR1 &= ~(0b1 << 7); // TXEIE = '0'.
		}
	}
#endif
	// RXNE interrupt.
	if (((USART1 -> ISR) & (0b1 << 5)) != 0) {
		// Transmit incoming byte to AT command manager.
		AT_FillRxBuffer(USART1 -> RDR);
	}
	// Overrun error interrupt.
	if (((USART1 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		USART1 -> ICR |= (0b1 << 3);
	}
}
#endif

/* FILL USART TX BUFFER WITH A NEW BYTE.
 * @param tx_byte:	Byte to append.
 * @return:			None.
 */
static void USARTx_FillTxBuffer(unsigned char tx_byte) {
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
#ifdef HW1_0
	USART2 -> TDR = tx_byte;
	// Wait for transmission to complete.
	while (((USART2 -> ISR) & (0b1 << 7)) == 0); // Wait for TXE='1'.
#endif
#ifdef HW2_0
	USART1 -> TDR = tx_byte;
	// Wait for transmission to complete.
	while (((USART1 -> ISR) & (0b1 << 7)) == 0); // Wait for TXE='1'.
#endif
#endif
}

/* CONVERTS A 4-BIT WORD TO THE ASCII CODE OF THE CORRESPONDING HEXADECIMAL CHARACTER.
 * @param n:	The word to converts.
 * @return:		The results of conversion.
 */
static char USARTx_HexaToAscii(unsigned char hexa_value) {
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
static unsigned int USARTx_Pow10(unsigned char power) {
	unsigned int result = 0;
	unsigned int pow10_buf[10] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
	if (power <= 9) {
		result = pow10_buf[power];
	}
	return result;
}
#endif

/*** USART functions ***/

#ifdef HW1_0
/* CONFIGURE USART2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART2_Init(void) {
#ifdef ATM
#ifdef USE_TXE_INTERRUPT
	// Init context.
	unsigned int idx = 0;
	for (idx=0 ; idx<USART_TX_BUFFER_SIZE ; idx++) usart_ctx.tx_buf[idx] = 0;
	usart_ctx.tx_buf_write_idx = 0;
	usart_ctx.tx_buf_read_idx = 0;
#endif
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 17); // USART2EN='1'.
	// Configure TX and RX GPIOs.
	GPIO_Configure(&GPIO_USART2_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_USART2_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Configure peripheral.
	USART2 -> CR1 = 0; // Disable peripheral before configuration (UE='0'), 1 stop bit and 8 data bits (M='00').
	USART2 -> CR2 = 0; // 1 stop bit (STOP='00').
	USART2 -> CR3 = 0;
	USART2 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='1').
	USART2 -> BRR = ((RCC_GetSysclkKhz() * 1000) / (USART_BAUD_RATE)); // BRR = (fCK)/(baud rate). See p.730 of RM0377 datasheet.
	USART2 -> CR1 &= ~(0b11 << 2); // Disable transmitter (TE='0') and receiver (RE='0') by default.
	// Enable transmitter and receiver.
	USART2 -> CR1 |= (0b11 << 2); // TE='1' and RE='1'.
	USART2 -> CR1 |= (0b1 << 5); // RXNEIE='1'.
	// Enable peripheral.
	USART2 -> CR1 |= (0b1 << 0);
#else
	// Configure TX and RX GPIOs.
	GPIO_Configure(&GPIO_USART2_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_USART2_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Set interrupt priority.
	NVIC_SetPriority(NVIC_IT_USART2, 3);
}
#endif

#ifdef HW2_0
/* CONFIGURE USART1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void USART1_Init(void) {
#ifdef ATM
#ifdef USE_TXE_INTERRUPT
	// Init context.
	unsigned int idx = 0;
	for (idx=0 ; idx<USART_TX_BUFFER_SIZE ; idx++) usart_ctx.tx_buf[idx] = 0;
	usart_ctx.tx_buf_write_idx = 0;
	usart_ctx.tx_buf_read_idx = 0;
#endif
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 14); // USART1EN='1'.
	// Configure TX and RX GPIOs.
	GPIO_Configure(&GPIO_USART1_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_USART1_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Configure peripheral.
	USART1 -> CR1 = 0; // Disable peripheral before configuration (UE='0'), 1 stop bit and 8 data bits (M='00').
	USART1 -> CR2 = 0; // 1 stop bit (STOP='00').
	USART1 -> CR3 = 0;
	USART1 -> CR3 |= (0b1 << 12); // No overrun detection (OVRDIS='1').
	USART1 -> BRR = ((RCC_GetSysclkKhz() * 1000) / (USART_BAUD_RATE)); // BRR = (fCK)/(baud rate). See p.730 of RM0377 datasheet.
	USART1 -> CR1 &= ~(0b11 << 2); // Disable transmitter (TE='0') and receiver (RE='0') by default.
	// Enable transmitter and receiver.
	USART1 -> CR1 |= (0b11 << 2); // TE='1' and RE='1'.
	USART1 -> CR1 |= (0b1 << 5); // RXNEIE='1'.
	// Enable peripheral.
	USART1 -> CR1 |= (0b1 << 0);
#else
	// Configure TX and RX GPIOs.
	GPIO_Configure(&GPIO_USART1_TX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_USART1_RX, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Set interrupt priority.
	NVIC_SetPriority(NVIC_IT_USART1, 3);
}
#endif

#ifdef ATM
/* SEND A BYTE THROUGH USART.
 * @param byte_to_send:	The byte to send.
 * @param format:		Display format (see ByteDisplayFormat enumeration in usart.h).
 * @param print_prexix	Print '0b' or '0x' prefix is non zero.
 * @return: 			None.
 */
void USARTx_SendValue(unsigned int tx_value, USART_Format format, unsigned char print_prefix) {
	// Disable interrupt.
#ifdef HW1_0
	NVIC_DisableInterrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_DisableInterrupt(NVIC_IT_USART1);
#endif
	// Local variables.
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
			USARTx_FillTxBuffer('0');
			USARTx_FillTxBuffer('b');
		}
		// Maximum 32 bits.
		for (idx=31 ; idx>=0 ; idx--) {
			if (tx_value & (0b1 << idx)) {
				USARTx_FillTxBuffer('1'); // = '1'.
				first_non_zero_found = 1;
			}
			else {
				if ((first_non_zero_found != 0) || (idx == 0)) {
					USARTx_FillTxBuffer('0'); // = '0'.
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
			USARTx_FillTxBuffer('0');
			USARTx_FillTxBuffer('x');
		}
		// Maximum 4 bytes.
		for (idx=3 ; idx>=0 ; idx--) {
			current_value = (tx_value & (0xFF << (8*idx))) >> (8*idx);
			if (current_value != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				USARTx_FillTxBuffer(USARTx_HexaToAscii((current_value & 0xF0) >> 4));
				USARTx_FillTxBuffer(USARTx_HexaToAscii(current_value & 0x0F));
			}
			if (idx == 0) {
				break;
			}
		}
		break;
	case USART_FORMAT_DECIMAL:
		// Maximum 10 digits.
		for (idx=9 ; idx>=0 ; idx--) {
			current_power = USARTx_Pow10(idx);
			current_value = (tx_value - previous_decade) / current_power;
			previous_decade += current_value * current_power;
			if (current_value != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				USARTx_FillTxBuffer(current_value + '0');
			}
			if (idx == 0) {
				break;
			}
		}
		break;
	case USART_FORMAT_ASCII:
		// Raw byte.
		if (tx_value <= 0xFF) {
			USARTx_FillTxBuffer(tx_value);
		}
		break;
	}
	// Enable interrupt.
#ifdef HW1_0
#ifdef USE_TXE_INTERRUPT
	USART2 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
#endif
	NVIC_EnableInterrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
#ifdef USE_TXE_INTERRUPT
	USART1 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
#endif
	NVIC_EnableInterrupt(NVIC_IT_USART1);
#endif
}

/* SEND A BYTE ARRAY THROUGH USART2.
 * @param tx_string:	Byte array to send.
 * @return:				None.
 */
void USARTx_SendString(char* tx_string) {
	// Disable interrupt.
#ifdef HW1_0
	NVIC_DisableInterrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_DisableInterrupt(NVIC_IT_USART1);
#endif
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		USARTx_FillTxBuffer((unsigned char) *(tx_string++));
	}
	// Enable interrupt.
	#ifdef HW1_0
	#ifdef USE_TXE_INTERRUPT
		USART2 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
	#endif
		NVIC_EnableInterrupt(NVIC_IT_USART2);
	#endif
	#ifdef HW2_0
	#ifdef USE_TXE_INTERRUPT
		USART1 -> CR1 |= (0b1 << 7); // (TXEIE = '1').
	#endif
		NVIC_EnableInterrupt(NVIC_IT_USART1);
	#endif
}

#endif
