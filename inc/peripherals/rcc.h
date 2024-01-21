/*
 * rcc.h
 *
 *  Created on: 28 apr. 2018
 *      Author: Ludo
 */

#ifndef __RCC_H__
#define __RCC_H__

#include "flash.h"
#include "types.h"

/*** RCC structures ***/

/*!******************************************************************
 * \enum RCC_status_t
 * \brief RCC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	RCC_SUCCESS = 0,
	RCC_ERROR_NULL_PARAMETER,
	RCC_ERROR_CLOCK,
	RCC_ERROR_HSI_READY,
	RCC_ERROR_HSI_SWITCH,
	RCC_ERROR_HSE_READY,
	RCC_ERROR_HSE_SWITCH,
	RCC_ERROR_MSI_RANGE,
	RCC_ERROR_MSI_READY,
	RCC_ERROR_MSI_SWITCH,
	RCC_ERROR_LSE_READY,
	RCC_ERROR_HSI_CALIBRATION,
	RCC_ERROR_LSI_CALIBRATION,
	// Low level drivers errors.
	RCC_ERROR_BASE_FLASH = 0x0100,
	// Last base value.
	RCC_ERROR_BASE_LAST = (RCC_ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST)
} RCC_status_t;

/*!******************************************************************
 * \enum RCC_clock_t
 * \brief RCC clocks list.
 *******************************************************************/
typedef enum {
	RCC_CLOCK_LSI = 0,
	RCC_CLOCK_LSE,
	RCC_CLOCK_MSI,
	RCC_CLOCK_HSI,
	RCC_CLOCK_HSE,
	RCC_CLOCK_SYSTEM,
	RCC_CLOCK_LAST
} RCC_clock_t;

/*!******************************************************************
 * \enum RCC_msi_range_t
 * \brief RCC MSI oscillator frequency ranges.
 *******************************************************************/
typedef enum {
	RCC_MSI_RANGE_0_65KHZ = 0,
	RCC_MSI_RANGE_1_131KHZ,
	RCC_MSI_RANGE_2_262KHZ,
	RCC_MSI_RANGE_3_524KKZ,
	RCC_MSI_RANGE_4_1MHZ,
	RCC_MSI_RANGE_5_2MHZ,
	RCC_MSI_RANGE_6_4MHZ,
	RCC_MSI_RANGE_LAST
} RCC_msi_range_t;

/*** RCC functions ***/

/*!******************************************************************
 * \fn void RCC_init(void)
 * \brief Init MCU default clock tree.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_init(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_hsi(void)
 * \brief Switch system clock to 16MHz HSI.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_switch_to_hsi(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range)
 * \brief Switch system clock to MSI.
 * \param[in]  	msi_range: MSI frequency to set.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range);

/*!******************************************************************
 * \fn RCC_status_t RCC_calibrate(void)
 * \brief Measure internal oscillators frequency with external reference clock.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_calibrate(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz)
 * \brief Get clock frequency.
 * \param[in]  	clock: Clock to read.
 * \param[out] 	frequency_hz: Pointer to the clock frequency in Hz.
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz);

/*!******************************************************************
 * \fn RCC_status_t RCC_get_status(RCC_clock_t clock, uint32_t* clock_is_ready
 * \brief Get clock status.
 * \param[in]  	clock: Clock to read.
 * \param[out] 	clock_is_ready: Pointer to the clock status (1 if the clock is running correctly, 0 otherwise).
 * \retval		Function execution status.
 *******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready);

/*******************************************************************/
#define RCC_exit_error(error_base) { if (rcc_status != RCC_SUCCESS) { status = (error_base + rcc_status); goto errors; } }

/*******************************************************************/
#define RCC_stack_error(void) { if (rcc_status != RCC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RCC + rcc_status); } }

/*******************************************************************/
#define RCC_stack_exit_error(error_code) { if (rcc_status != RCC_SUCCESS) { ERROR_stack_add(ERROR_BASE_RCC + rcc_status); status = error_code; goto errors; } }

#endif /* __RCC_H__ */
