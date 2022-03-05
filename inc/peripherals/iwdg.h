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
	IWDG_ERROR_LAST
} IWDG_status_t;

/*** IWDG functions ***/

IWDG_status_t IWDG_init(void);
void IWDG_reload(void);

#endif /* IWDG_H */
