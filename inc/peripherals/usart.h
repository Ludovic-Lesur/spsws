/*
 * usart.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#ifndef USART_H
#define USART_H

/*** USART structures ***/

typedef enum {
	USART_FORMAT_BINARY,
	USART_FORMAT_HEXADECIMAL,
	USART_FORMAT_DECIMAL,
	USART_FORMAT_ASCII
} USART_Format;

/*** USART functions ***/

void USART_Init(void);
void USART_PowerOn(void);
void USART_PowerOff(void);
void USART_Off(void);
void USART_SendValue(unsigned int tx_value, USART_Format format, unsigned char print_prefix);
void USART_SendString(char* tx_string);

#endif /* USART_H */
