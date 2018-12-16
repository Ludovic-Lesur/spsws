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
	USART_Binary,
	USART_Hexadecimal,
	USART_Decimal,
	USART_ASCII
} UsartDisplayFormat;

/*** USART functions ***/

void USART_Init(void);
void USART_PowerOn(void);
void USART_PowerOff(void);
void USART_Off(void);
void USART_SendValue(unsigned int value, UsartDisplayFormat display_format);
void USART_SendString(char* tx_string);

#endif /* USART_H */
