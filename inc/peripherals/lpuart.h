/*
 * lpuart.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_LPUART_H
#define PERIPHERALS_LPUART_H

/*** LPUART functions ***/

void LPUART_Init(void);
void LPUART_SendByte(unsigned char byte_to_send);
void LPUART_SendString(char* string_to_send);

#endif /* PERIPHERALS_LPUART_H */
