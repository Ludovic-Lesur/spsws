/*
 * usart.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef USART_H
#define USART_H

#include "mode.h"

#ifdef ATM
/*** USART structures ***/

typedef enum {
	USART_FORMAT_BINARY,
	USART_FORMAT_HEXADECIMAL,
	USART_FORMAT_DECIMAL,
	USART_FORMAT_ASCII
} USART_Format;
#endif

/*** USART functions ***/

#ifdef HW1_0
void USART2_Init(void);
#endif
#ifdef HW2_0
void USART1_Init(void);
#endif
#ifdef ATM
void USARTx_SendValue(unsigned int tx_value, USART_Format format, unsigned char print_prefix);
void USARTx_SendString(char* tx_string);
#endif

#endif /* USART_H */
