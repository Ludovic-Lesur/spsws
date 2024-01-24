/*
 * wind.h
 *
 *  Created on: 24 nov. 2018
 *      Author: Ludo
 */

#ifndef __WIND_H__
#define __WIND_H__

#include "lptim.h"
#include "mapping.h"
#include "math.h"
#include "max11136.h"
#include "mode.h"
#include "power.h"
#include "types.h"

/*** WIND macros ***/

#ifdef SPSWS_WIND_MEASUREMENT
#define WIND_SPEED_MEASUREMENT_PERIOD_SECONDS		1
#define WIND_DIRECTION_MEASUREMENT_PERIOD_SECONDS	10
#endif

/*** WIND structures ***/

/*!******************************************************************
 * \enum WIND_status_t
 * \brief WIND driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	WIND_SUCCESS = 0,
	WIND_ERROR_NULL_PARAMETER,
	// Low level drivers errors.
	WIND_ERROR_BASE_LPTIM1 = 0x0100,
	WIND_ERROR_BASE_POWER = (WIND_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST),
	WIND_ERROR_BASE_MAX11136 = (WIND_ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
	WIND_ERROR_BASE_MATH = (WIND_ERROR_BASE_MAX11136 + MAX11136_ERROR_BASE_LAST),
	// Last base value.
	WIND_ERROR_BASE_LAST = (WIND_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} WIND_status_t;

/*** WIND functions ***/

#ifdef SPSWS_WIND_MEASUREMENT
/*!******************************************************************
 * \fn void WIND_init(void)
 * \brief Init wind driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void WIND_init(void);
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*!******************************************************************
 * \fn void WIND_start_continuous_measure(void)
 * \brief Start continuous wind measurements.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void WIND_start_continuous_measure(void);
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*!******************************************************************
 * \fn void WIND_stop_continuous_measure(void)
 * \brief Stop continuous wind measurements.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void WIND_stop_continuous_measure(void);
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*!******************************************************************
 * \fn void WIND_reset_data(void);
 * \brief Reset wind measurements.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void WIND_reset_data(void);
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*!******************************************************************
 * \fn WIND_status_t WIND_tick_second(void);
 * \brief Update wind measurements every second.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
WIND_status_t WIND_tick_second(void);
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*!******************************************************************
 * \fn WIND_status_t WIND_get_speed(uint32_t* average_speed_mh, uint32_t* peak_speed_mh)
 * \brief Read wind speeds.
 * \param[in]  	none
 * \param[out] 	average_speed_mh: Average wind speed since last reset in m/h.
 * \param[out] 	peak_speed_mh: Peak wind speed since last reset in m/h.
 * \retval		Function execution status.
 *******************************************************************/
WIND_status_t WIND_get_speed(uint32_t* average_speed_mh, uint32_t* peak_speed_mh);
#endif

#ifdef SPSWS_WIND_MEASUREMENT
/*!******************************************************************
 * \fn WIND_status_t WIND_get_direction(uint32_t* average_direction_degrees)
 * \brief Read wind direction.
 * \param[in]  	none
 * \param[out] 	average_direction_degrees: Average wind direction since last reset in degrees.
 * \retval		Function execution status.
 *******************************************************************/
WIND_status_t WIND_get_direction(uint32_t* average_direction_degrees);
#endif

/*******************************************************************/
#define WIND_exit_error(error_base) { if (wind_status != WIND_SUCCESS) { status = (error_base + wind_status); goto errors; } }

/*******************************************************************/
#define WIND_stack_error(void) { if (wind_status != WIND_SUCCESS) { ERROR_stack_add(ERROR_BASE_WIND + wind_status); } }

/*******************************************************************/
#define WIND_stack_exit_error(error_code) { if (wind_status != WIND_SUCCESS) { ERROR_stack_add(ERROR_BASE_WIND + wind_status); status = error_code; goto errors; } }

#endif  /* __WIND_H__ */
