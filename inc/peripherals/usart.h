/*
 * usart.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_USART_H_
#define PERIPHERALS_USART_H_

/*** USART structures ***/

typedef enum {
	USART_Binary,
	USART_Hexadecimal,
	USART_Decimal,
	USART_ASCII
} UsartDisplayFormat;

/*** USART functions ***/

void USART_Init(void);
void USART_Off(void);
void USART_SendValue(unsigned int value, UsartDisplayFormat display_format);
void USART_SendString(char* tx_string);

#endif /* PERIPHERALS_USART_H_ */
