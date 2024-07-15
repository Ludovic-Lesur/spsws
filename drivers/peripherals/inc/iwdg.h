/*
 * iwdg.h
 *
 *  Created on: 30 mar. 2018
 *      Author: Ludo
 */

#ifndef __IWDG_H__
#define __IWDG_H__

/*** IWDG macros ***/

// Based on worst case 56kHz LSI clock frequency, minimum IWDG period is 18 seconds.
// Adding 3 second margin to perform reload operation, the maximum free delay is limited to 15 seconds.
#define IWDG_FREE_DELAY_SECONDS_MAX		15

/*** IWDG structures ***/

/*!******************************************************************
 * \enum IWDG_status_t
 * \brief IWDG driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	IWDG_SUCCESS = 0,
	IWDG_ERROR_TIMEOUT,
	// Last base value.
	IWDG_ERROR_BASE_LAST = 0x0100
} IWDG_status_t;

/*** IWDG functions ***/

/*!******************************************************************
 * \fn IWDG_status_t IWDG_init(void)
 * \brief Start independent watchdog.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
IWDG_status_t IWDG_init(void);

/*!******************************************************************
 * \fn void IWDG_reload(void)
 * \brief Refresh independent watchdog.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
void IWDG_reload(void);

/*******************************************************************/
#define IWDG_exit_error(error_base) { if (iwdg_status != IWDG_SUCCESS) { status = (error_base + iwdg_status); goto errors; } }

/*******************************************************************/
#define IWDG_stack_error(void) { if (iwdg_status != IWDG_SUCCESS) { ERROR_stack_add(ERROR_BASE_IWDG + iwdg_status); } }

/*******************************************************************/
#define IWDG_stack_exit_error(error_code) { if (iwdg_status != IWDG_SUCCESS) { ERROR_stack_add(ERROR_BASE_IWDG + iwdg_status); status = error_code; goto errors; } }

#endif /* __IWDG_H__ */
