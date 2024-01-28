/*
 * lpuart.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __LPUART_H__
#define __LPUART_H__

#include "rcc.h"
#include "types.h"

/*** LPUART structures ***/

/*!******************************************************************
 * \enum LPUART_status_t
 * \brief LPUART driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	LPUART_SUCCESS = 0,
	LPUART_ERROR_NULL_PARAMETER,
	LPUART_ERROR_TX_TIMEOUT,
	LPUART_ERROR_TC_TIMEOUT,
	// Low level drivers errors.
	LPUART_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	LPUART_ERROR_BASE_LAST = (LPUART_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} LPUART_status_t;

/*!******************************************************************
 * \fn LPUART_character_match_irq_cb_t
 * \brief LPUART character match interrupt callback.
 *******************************************************************/
typedef void (*LPUART_character_match_irq_cb_t)(void);

/*** LPUART functions ***/

/*!******************************************************************
 * \fn LPUART_status_t LPUART1_init(LPUART_character_match_irq_cb_t irq_callback)
 * \brief Init LPUART1 peripheral.
 * \param[in]  	irq_callback: Function to call on character match interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPUART_status_t LPUART1_init(LPUART_character_match_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn void LPUART1_de_init(void)
 * \brief Release LPUART1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LPUART1_de_init(void);

/*!******************************************************************
 * \fn LPUART_status_t LPUART1_write(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over LPUART1.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPUART_status_t LPUART1_write(uint8_t* data, uint32_t data_size_bytes);

/*******************************************************************/
#define LPUART1_exit_error(error_base) { if (lpuart1_status != LPUART_SUCCESS) { status = (error_base + lpuart1_status); goto errors; } }

/*******************************************************************/
#define LPUART1_stack_error(void) { if (lpuart1_status != LPUART_SUCCESS) { ERROR_stack_add(ERROR_BASE_LPUART1 + lpuart1_status); } }

/*******************************************************************/
#define LPUART1_stack_exit_error(error_code) { if (lpuart1_status != LPUART_SUCCESS) { ERROR_stack_add(ERROR_BASE_LPUART1 + lpuart1_status); status = error_code; goto errors; } }

#endif /* __LPUART_H__ */
