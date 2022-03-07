/*
 * lptim.h
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#ifndef LPTIM_H
#define LPTIM_H

#include "mode.h"

/*** LPTIM structures ***/

typedef enum {
	LPTIM_SUCCESS = 0,
	LPTIM_ERROR_DELAY_UNDERFLOW,
	LPTIM_ERROR_DELAY_OVERFLOW,
	LPTIM_ERROR_WRITE_ARR,
	LPTIM_ERROR_LAST
} LPTIM_status_t;

/*** LPTIM functions ***/

void LPTIM1_init(unsigned int lsi_freq_hz);
void LPTIM1_enable(void);
void LPTIM1_disable(void);
LPTIM_status_t LPTIM1_delay_milliseconds(unsigned int delay_ms, unsigned char stop_mode);
#ifdef WIND_VANE_ULTIMETER
LPTIM_status_t LPTIM1_start(void);
void LPTIM1_stop(void);
unsigned int LPTIM1_get_counter(void);
#endif

#define LPTIM1_status_check(error_base) { if (lptim1_status != LPTIM_SUCCESS) { status = error_base + lptim1_status; goto errors; }}

#endif /* LPTIM_H */
