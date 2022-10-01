/*
 * usart.h
 *
 *  Created on: 11 aug 2018
 *      Author: Ludo
 */

#ifndef __USART_H__
#define __USART_H__

#include "mode.h"
#include "types.h"

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
void USARTx_enable_interrupt(void);
void USARTx_disable_interrupt(void);
USART_status_t USARTx_send_string(int8_t* tx_string);
#endif

#define USART_status_check(error_base) { if (usart_status != USART_SUCCESS) { status = error_base + usart_status; goto errors; }}
#define USART_error_check() { ERROR_status_check(usart_status, USART_SUCCESS, ERROR_BASE_USART); }
#define USART_error_check_print() { ERROR_status_check_print(usart_status, USART_SUCCESS, ERROR_BASE_USART); }

#endif /* __USART_H__ */
