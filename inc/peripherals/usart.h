/*
 * usart.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef USART_H
#define USART_H

#include "mode.h"

/*** USART structures ***/

#ifdef ATM
typedef enum {
	USART_SUCCESS = 0,
	USART_ERROR_TX_TIMEOUT,
	USART_ERROR_STRING_LENGTH,
	USART_ERROR_LAST
} USART_status_t;
#endif

/*** USART functions ***/

#ifdef HW1_0
void USART2_init(void);
#endif
#ifdef HW2_0
void USART1_init(void);
#endif
#ifdef ATM
USART_status_t USARTx_send_string(char* tx_string);
#endif

#endif /* USART_H */
