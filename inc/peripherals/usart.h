/*
 * usart.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_USART_H_
#define PERIPHERALS_USART_H_

/*** USART functions ***/

void USART_Init(void);
void USART_Off(void);
void USART_SendString(unsigned char* string_to_send);

#endif /* PERIPHERALS_USART_H_ */
