/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef __LPTIM_H__
#define __LPTIM_H__

#include "mode.h"

/*** LPTIM structures ***/

typedef enum {
	LPTIM_SUCCESS = 0,
	LPTIM_ERROR_DELAY_UNDERFLOW,
	LPTIM_ERROR_DELAY_OVERFLOW,
	LPTIM_ERROR_WRITE_ARR,
	LPTIM_ERROR_BASE_LAST = 0x0100
} LPTIM_status_t;

/*** LPTIM functions ***/

void LPTIM1_init(unsigned int lsi_freq_hz);
LPTIM_status_t LPTIM1_delay_milliseconds(unsigned int delay_ms, unsigned char stop_mode);
#ifdef WIND_VANE_ULTIMETER
LPTIM_status_t LPTIM1_start(void);
void LPTIM1_stop(void);
unsigned int LPTIM1_get_counter(void);
#endif

#define LPTIM1_status_check(error_base) { if (lptim_status != LPTIM_SUCCESS) { status = error_base + lptim_status; goto errors; }}
#define LPTIM1_error_check() { ERROR_status_check(lptim_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM1); }
#define LPTIM1_error_check_print() { ERROR_status_check_print(lptim_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM1); }

#endif /* __LPTIM_H__ */
