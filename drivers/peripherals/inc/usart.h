/*
 * usart.h
 *
 *  Created on: 11 aug. 2018
 *      Author: Ludo
 */

#ifndef __USART_H__
#define __USART_H__

#include "mode.h"
#include "rcc.h"
#include "types.h"

/*** USART structures ***/

/*!******************************************************************
 * \enum USART_status_t
 * \brief USART driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	USART_SUCCESS = 0,
	USART_ERROR_NULL_PARAMETER,
	USART_ERROR_TX_TIMEOUT,
	// Low level drivers errors.
	USART_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	USART_ERROR_BASE_LAST = (USART_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} USART_status_t;

/*!******************************************************************
 * \fn USART_rx_irq_cb_t
 * \brief USART RX interrupt callback.
 *******************************************************************/
typedef void (*USART_rx_irq_cb_t)(uint8_t data);

/*** USART functions ***/

#if (defined HW1_0) && (defined ATM)
/*!******************************************************************
 * \fn USART_status_t USART2_init(USART_rx_irq_cb_t irq_callback)
 * \brief Init USART2 peripheral.
 * \param[in]  	irq_callback: Function to call on interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART2_init(USART_rx_irq_cb_t irq_callback);
#endif

#if (defined HW1_0) && (defined ATM)
/*!******************************************************************
 * \fn void USART2_de_init(void)
 * \brief Release USART2 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_de_init(void);
#endif

#if (defined HW1_0) && (defined ATM)
/*!******************************************************************
 * \fn void USART2_enable_rx(void)
 * \brief Enable USART2 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_enable_rx(void);
#endif

#if (defined HW1_0) && (defined ATM)
/*!******************************************************************
 * \fn void USART2_disable_rx(void)
 * \brief Disable USART2 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART2_disable_rx(void);
#endif

#if (defined HW1_0) && (defined ATM)
/*!******************************************************************
 * \fn USART_status_t USART2_write(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over USART2.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART2_write(uint8_t* data, uint32_t data_size_bytes);
#endif

#if (defined HW2_0) && (defined ATM)
/*!******************************************************************
 * \fn USART_status_t USART1_init(USART_rx_irq_cb_t irq_callback)
 * \brief Init USART1 peripheral.
 * \param[in]  	irq_callback: Function to call on interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART1_init(USART_rx_irq_cb_t irq_callback);
#endif

#if (defined HW2_0) && (defined ATM)
/*!******************************************************************
 * \fn void USART1_de_init(void)
 * \brief Release USART1 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART1_de_init(void);
#endif

#if (defined HW2_0) && (defined ATM)
/*!******************************************************************
 * \fn void USART1_enable_rx(void)
 * \brief Enable USART1 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART1_enable_rx(void);
#endif

#if (defined HW2_0) && (defined ATM)
/*!******************************************************************
 * \fn void USART1_disable_rx(void)
 * \brief Disable USART1 RX operation.
 * \param[in]   none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void USART1_disable_rx(void);
#endif

#if (defined HW2_0) && (defined ATM)
/*!******************************************************************
 * \fn USART_status_t USART1_write(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over USART1.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART1_write(uint8_t* data, uint32_t data_size_bytes);
#endif

/*******************************************************************/
#define USART1_exit_error(error_base) { if (usart1_status != USART_SUCCESS) { status = (error_base + usart1_status); goto errors; } }

/*******************************************************************/
#define USART1_stack_error(void) { if (usart1_status != USART_SUCCESS) { ERROR_stack_add(ERROR_BASE_USART1 + usart1_status); } }

/*******************************************************************/
#define USART1_stack_exit_error(error_code) { if (usart1_status != USART_SUCCESS) { ERROR_stack_add(ERROR_BASE_USART1 + usart1_status); status = error_code; goto errors; } }

/*******************************************************************/
#define USART2_exit_error(error_base) { if (usart2_status != USART_SUCCESS) { status = (error_base + usart2_status); goto errors; } }

/*******************************************************************/
#define USART2_stack_error(void) { if (usart2_status != USART_SUCCESS) { ERROR_stack_add(ERROR_BASE_USART2 + usart2_status); } }

/*******************************************************************/
#define USART2_stack_exit_error(error_code) { if (usart2_status != USART_SUCCESS) { ERROR_stack_add(ERROR_BASE_USART2 + usart2_status); status = error_code; goto errors; } }

#endif /* __USART_H__ */
