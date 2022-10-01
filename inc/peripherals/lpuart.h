/*
 * lpuart.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __LPUART_H__
#define __LPUART_H__

#include "lptim.h"
#include "types.h"

/*** LPUART structures ***/

typedef enum {
	LPUART_SUCCESS = 0,
	LPUART_ERROR_TX_TIMEOUT,
	LPUART_ERROR_BASE_LPTIM = 0x0100,
	LPUART_ERROR_BASE_LAST = (LPUART_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} LPUART_status_t;

/*** LPUART functions ***/

void LPUART1_init(uint8_t lpuart_use_lse);
LPUART_status_t LPUART1_power_on(void);
void LPUART1_power_off(void);
LPUART_status_t LPUART1_send_byte(uint8_t tx_byte);

#define LPUART1_status_check(error_base) { if (lpuart1_status != LPUART_SUCCESS) { status = error_base + lpuart1_status; goto errors; }}
#define LPUART1_error_check() { ERROR_status_check(lpuart1_status, LPUART_SUCCESS, ERROR_BASE_LPUART1); }
#define LPUART1_error_check_print() { ERROR_status_check_print(lpuart1_status, LPUART_SUCCESS, ERROR_BASE_LPUART1); }

#endif /* __LPUART_H__ */
