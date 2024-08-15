/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef __AT_H__
#define __AT_H__

#include "mode.h"
#include "sigfox_types.h"
#include "types.h"

/*** AT functions ***/

#ifdef ATM
/*!******************************************************************
 * \fn void AT_init(void)
 * \brief Init AT interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_init(void);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void AT_task(void)
 * \brief AT interface task.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_task(void);
#endif

#if (defined ATM) && (defined SPSWS_WIND_MEASUREMENT)
/*!******************************************************************
 * \fn uint8_t AT_get_wind_measurement_flag(void);
 * \brief Read the wind measurement enable flag.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Wind measurement flag from commands.
 *******************************************************************/
uint8_t AT_get_wind_measurement_flag(void);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void AT_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm)
 * \brief Print a downlink frame (only used by the RFP addon during downlink test modes).
 * \param[in]  	dl_payload: Downlink payload to print.
 * \param[in] 	dl_payload_size: Number of bytes to print.
 * \param[in]	rssi_dbm: RSSI of the received downlink frame (16-bits signed value).
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
void AT_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void AT_print_rainfall(uint32_t rain_edge_count)
 * \brief Print rainfall edge count.
 * \param[in]  	rain_edge_count: Value to print.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_print_rainfall(uint32_t rain_edge_count);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void AT_print_wind_speed(uint32_t speed_mh)
 * \brief Print wind speed.
 * \param[in]  	speed_mh: Value to print in m/h.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_print_wind_speed(uint32_t speed_mh);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void AT_print_wind_direction(uint32_t direction_degrees, int32_t direction_x, int32_t direction_y)
 * \brief Print wind direction.
 * \param[in]  	direction_degrees: Direction in degrees.
 * \param[in]	direction_x: Horizontal value of the wind vector.
 * \param[in]	direction_y: Vertical value of the wind vector.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AT_print_wind_direction(uint32_t direction_degrees, int32_t direction_x, int32_t direction_y);
#endif

#endif /* __AT_H__ */
