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
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void AT_init(void);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void AT_task(void)
 * \brief AT interface task.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void AT_task(void);
#endif

#ifdef ATM
/*!******************************************************************
 * \fn void AT_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm)
 * \brief Print a downlink frame (only used by the RFP addon during downlink test modes).
 * \param[in]   dl_payload: Downlink payload to print.
 * \param[in]   dl_payload_size: Number of bytes to print.
 * \param[in]   rssi_dbm: RSSI of the received downlink frame (16-bits signed value).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
void AT_print_dl_payload(sfx_u8* dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm);
#endif

#endif /* __AT_H__ */
