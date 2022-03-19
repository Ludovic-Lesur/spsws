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

typedef enum {
	USART_SUCCESS = 0,
	USART_ERROR_TX_TIMEOUT,
	USART_ERROR_STRING_LENGTH,
	USART_ERROR_BASE_LAST = 0x0100
} USART_status_t;

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

#define USART_status_check(error_base) { if (usart_status != USART_SUCCESS) { status = error_base + usart_status; goto errors; }}
#define USART_error_check() { ERROR_status_check(usart_status, USART_SUCCESS, ERROR_BASE_USART); }

#endif /* USART_H */
