/*
 * rain.h
 *
 *  Created on: 5 may 2019
 *      Author: Ludo
 */

#ifndef __RAIN_H__
#define __RAIN_H__

#include "mode.h"
#include "types.h"

/*** RAIN structures ***/

/*!******************************************************************
 * \enum RAIN_status_t
 * \brief RAIN driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	RAIN_SUCCESS = 0,
	RAIN_ERROR_NULL_PARAMETER,
	// Last base value.
	RAIN_ERROR_BASE_LAST = 0x0100
} RAIN_status_t;

/*** RAIN functions ***/

#if (defined SPSWS_RAIN_MEASUREMENT) || (defined SPSWS_FLOOD_MEASUREMENT)
/*!******************************************************************
 * \fn void RAIN_init(void)
 * \brief Init RAIN driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RAIN_init(void);
#endif

#if (defined SPSWS_RAIN_MEASUREMENT) || (defined SPSWS_FLOOD_MEASUREMENT)
/*!******************************************************************
 * \fn void RAIN_start_continuous_measure(void)
 * \brief Start rainfall measurement.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RAIN_start_continuous_measure(void);
#endif

#if (defined SPSWS_RAIN_MEASUREMENT) || (defined SPSWS_FLOOD_MEASUREMENT)
/*!******************************************************************
 * \fn void RAIN_stop_continuous_measure(void)
 * \brief Stop rainfall measurement.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RAIN_stop_continuous_measure(void);
#endif

#ifdef SPSWS_RAIN_MEASUREMENT
/*!******************************************************************
 * \fn void RAIN_reset_rainfall(void)
 * \brief Reset rainfall measurement.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RAIN_reset_rainfall(void);
#endif

#ifdef SPSWS_RAIN_MEASUREMENT
/*!******************************************************************
 * \fn RAIN_status_t RAIN_get_rainfall(uint8_t* rainfall_mm)
 * \brief Read rainfall.
 * \param[in]  	none
 * \param[out] 	rainfall_mm: Pointer to byte that will contain the rainfall count in mm.
 * \retval		Function execution status.
 *******************************************************************/
RAIN_status_t RAIN_get_rainfall(uint8_t* rainfall_mm);
#endif

#ifdef SPSWS_FLOOD_MEASUREMENT
/*!******************************************************************
 * \fn RAIN_status_t RAIN_get_flood_level(uint8_t* flood_level)
 * \brief Read rainfall.
 * \param[in]  	none
 * \param[out] 	flood_level: Pointer to byte that will contain the flood level.
 * \retval		Function execution status.
 *******************************************************************/
RAIN_status_t RAIN_get_flood_level(uint8_t* flood_level);
#endif

/*******************************************************************/
#define RAIN_exit_error(base) { ERROR_check_exit(rain_status, RAIN_SUCCESS, base) }

/*******************************************************************/
#define RAIN_stack_error(base) { ERROR_check_stack(rain_status, RAIN_SUCCESS, base) }

/*******************************************************************/
#define RAIN_stack_exit_error(base, code) { ERROR_check_stack_exit(rain_status, RAIN_SUCCESS, base, code) }

#endif /* __RAIN_H__ */
