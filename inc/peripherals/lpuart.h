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
void LPUART_StartRx(void);
void LPUART_StopRx(void);
void LPUART_Off(void);
void LPUART_SendByte(unsigned char byte_to_send);

#endif /* PERIPHERALS_LPUART_H */
