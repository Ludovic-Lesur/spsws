/*
 * tim.h
 *
 *  Created on: 04 may 2018
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

#include "rcc.h"
#include "types.h"

/*** TIM structures ***/

/*!******************************************************************
 * \enum TIM_status_t
 * \brief TIM driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	TIM_SUCCESS = 0,
	TIM_ERROR_NULL_PARAMETER,
	TIM_ERROR_CAPTURE_TIMEOUT,
	TIM_ERROR_CHANNEL,
	TIM_ERROR_DURATION_UNDERFLOW,
	TIM_ERROR_DURATION_OVERFLOW,
	TIM_ERROR_PERIOD_UNDERFLOW,
	TIM_ERROR_PERIOD_OVERFLOW,
	TIM_ERROR_WAITING_MODE,
	TIM_ERROR_COMPLETION_WATCHDOG,
	// Low level drivers errors.
	TIM_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	TIM_ERROR_BASE_LAST = (TIM_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} TIM_status_t;

/*!******************************************************************
 * \enum TIM2_channel_t
 * \brief TIM channels list.
 *******************************************************************/
typedef enum {
	TIM2_CHANNEL_1 = 0,
	TIM2_CHANNEL_2,
	TIM2_CHANNEL_3,
	TIM2_CHANNEL_4,
	TIM2_CHANNEL_LAST
} TIM2_channel_t;

/*!******************************************************************
 * \enum TIM_waiting_mode_t
 * \brief Timer completion waiting modes.
 *******************************************************************/
typedef enum {
	TIM_WAITING_MODE_ACTIVE = 0,
	TIM_WAITING_MODE_SLEEP,
	TIM_WAITING_MODE_LOW_POWER_SLEEP,
	TIM_WAITING_MODE_LAST
} TIM_waiting_mode_t;

/*!******************************************************************
 * \fn TIM_completion_irq_cb_t
 * \brief TIM completion callback.
 *******************************************************************/
typedef void (*TIM_completion_irq_cb_t)(void);

/*** TIM functions ***/

/*!******************************************************************
 * \fn TIM_status_t TIM2_init(void)
 * \brief Init TIM2 peripheral for general purpose timer operation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM2_init(void);

/*!******************************************************************
 * \fn void TIM2_de_init(void)
 * \brief Release TIM2 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM2_de_init(void);

/*!******************************************************************
 * \fn TIM_status_t TIM2_start(TIM2_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode)
 * \brief Start a timer channel.
 * \param[in]  	channel: Channel to start.
 * \param[in]	duration_ms: Timer duration in ms.
 * \param[in]	waiting_mode: Completion waiting mode.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM2_start(TIM2_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode);

/*!******************************************************************
 * \fn TIM_status_t TIM2_stop(TIM2_channel_t channel)
 * \brief Stop a timer channel.
 * \param[in]  	channel: Channel to stop.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM2_stop(TIM2_channel_t channel);

/*!******************************************************************
 * \fn TIM_status_t TIM2_get_status(TIM2_channel_t channel, uint8_t* timer_has_elapsed)
 * \brief Get the status of a timer channel.
 * \param[in]  	channel: Channel to read.
 * \param[out]	timer_has_elapsed: Pointer to bit that will contain the timer status (0 for running, 1 for complete).
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM2_get_status(TIM2_channel_t channel, uint8_t* timer_has_elapsed);

/*!******************************************************************
 * \fn TIM_status_t TIM2_wait_completion(TIM2_channel_t channel, TIM_waiting_mode_t waiting_mode)
 * \brief Blocking function waiting for a timer channel completion.
 * \param[in]  	channel: Channel to wait for.
 * \param[in]	waiting_mode: Completion waiting mode.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM2_wait_completion(TIM2_channel_t channel, TIM_waiting_mode_t waiting_mode);

/*!******************************************************************
 * \fn void TIM21_init(void)
 * \brief Init TIM21 peripheral for internal oscillators frequency measurement.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM21_init(void);

/*!******************************************************************
 * \fn void TIM21_de_init(void)
 * \brief Release TIM21 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM21_de_init(void);

/*!******************************************************************
 * \fn TIM_status_t TIM21_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count)
 * \brief Perform MCO clock capture.
 * \param[in]  	none
 * \param[out] 	ref_clock_pulse_count: Pointer to the number of pulses of the timer reference clock during the capture.
 * \param[out]	mco_pulse_count: Pointer to the number of pulses of the MCO clock during the capture.
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM21_mco_capture(uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count);

/*!******************************************************************
 * \fn TIM_status_t TIM22_init(uint32_t period_ns, TIM_completion_irq_cb_t irq_callback)
 * \brief Init TIM22 peripheral for Sigfox uplink modulation.
 * \param[in]  	period_ns: Timer period in ns.
 * \param[in]	irq_callback: Function to call on interrupt.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
TIM_status_t TIM22_init(uint32_t period_ns, TIM_completion_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn void TIM22_de_init(void)
 * \brief Release TIM22 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM22_de_init(void);

/*!******************************************************************
 * \fn void TIM22_start(void)
 * \brief Start TIM22 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM22_start(void);

/*!******************************************************************
 * \fn void TIM22_stop(void)
 * \brief Stop TIM22 peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void TIM22_stop(void);

/*******************************************************************/
#define TIM2_exit_error(error_base) { if (tim2_status != TIM_SUCCESS) { status = (error_base + tim2_status); goto errors; } }

/*******************************************************************/
#define TIM2_stack_error(void) { if (tim2_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM2 + tim2_status); } }

/*******************************************************************/
#define TIM2_stack_exit_error(error_code) { if (tim2_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM2 + tim2_status); status = error_code; goto errors; } }

/*******************************************************************/
#define TIM21_exit_error(error_base) { if (tim21_status != TIM_SUCCESS) { status = (error_base + tim21_status); goto errors; } }

/*******************************************************************/
#define TIM21_stack_error(void) { if (tim21_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM21 + tim21_status); } }

/*******************************************************************/
#define TIM21_stack_exit_error(error_code) { if (tim21_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM21 + tim21_status); status = error_code; goto errors; } }

/*******************************************************************/
#define TIM22_exit_error(error_base) { if (tim22_status != TIM_SUCCESS) { status = (error_base + tim22_status); goto errors; } }

/*******************************************************************/
#define TIM22_stack_error(void) { if (tim22_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM22 + tim22_status); } }

/*******************************************************************/
#define TIM22_stack_exit_error(error_code) { if (tim22_status != TIM_SUCCESS) { ERROR_stack_add(ERROR_BASE_TIM22 + tim22_status); status = error_code; goto errors; } }

#endif /* __TIM_H__ */
