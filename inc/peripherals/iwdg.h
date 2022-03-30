/*
 * iwdg.h
 *
 *  Created on: 30 march 2018
 *      Author: Ludovic
 */

#ifndef IWDG_H
#define IWDG_H

/*** IWDG macros ***/

#define IWDG_REFRESH_PERIOD_SECONDS		10

/*** IWDG structures ***/

typedef enum {
	IWDG_SUCCESS = 0,
	IWDG_ERROR_TIMEOUT,
	IWDG_ERROR_BASE_LAST = 0x0100
} IWDG_status_t;

/*** IWDG functions ***/

IWDG_status_t IWDG_init(void);
void IWDG_reload(void);

#define IWDG_status_check(error_base) { if (iwdg_status != IWDG_SUCCESS) { status = error_base + iwdg_status; goto errors; }}
#define IWDG_error_check() { ERROR_status_check(iwdg_status, IWDG_SUCCESS, ERROR_BASE_IWDG); }
#define IWDG_error_check_print() { ERROR_status_check_print(iwdg_status, IWDG_SUCCESS, ERROR_BASE_IWDG); }

#endif /* IWDG_H */
