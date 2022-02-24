/*
 * usart.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef USART_H
#define USART_H

#include "mode.h"

/*** USART functions ***/

#ifdef HW1_0
void USART2_Init(void);
#endif
#ifdef HW2_0
void USART1_Init(void);
#endif
#ifdef ATM
void USARTx_SendString(char* tx_string);
#endif

#endif /* USART_H */
