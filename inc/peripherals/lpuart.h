/*
 * lpuart.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef LPUART_H
#define LPUART_H

#include "lptim.h"

/*** LPUART structures ***/

typedef enum {
	LPUART_SUCCESS = 0,
	LPUART_ERROR_LPTIM,
	LPUART_ERROR_TX_TIMEOUT = (LPUART_ERROR_LPTIM + LPTIM_ERROR_LAST),
	LPUART_ERROR_LAST
} LPUART_status_t;

/*** LPUART functions ***/

void LPUART1_init(unsigned char lpuart_use_lse);
void LPUART1_update_brr(void);
void LPUART1_enable_tx(void);
void LPUART1_enable_rx(void);
void LPUART1_disable(void);
LPUART_status_t LPUART1_power_on(void);
LPUART_status_t LPUART1_power_off(void);
LPUART_status_t LPUART1_send_byte(unsigned char tx_byte);

#define LPUART1_status_check(error_base) { if (lpuart1_status != LPUART_SUCCESS) { status = error_base + lpuart1_status; goto errors; }}

#endif /* LPUART_H */
