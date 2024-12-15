/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#ifndef __CLI_H__
#define __CLI_H__

#include "at.h"
#include "sigfox_types.h"
#include "spsws_flags.h"
#include "types.h"

/*** CLI structures ***/

/*!******************************************************************
 * \enum CLI_status_t
 * \brief CLI driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    CLI_SUCCESS = 0,
    // Low level drivers errors.
    CLI_ERROR_BASE_AT = 0x0100,
    // Last base value.
    CLI_ERROR_BASE_LAST = (CLI_ERROR_BASE_AT + AT_ERROR_BASE_LAST)
} CLI_status_t;

#ifdef SPSWS_MODE_CLI

/*** CLI functions ***/

/*!******************************************************************
 * \fn CLI_status_t CLI_init(void)
 * \brief Init CLI driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_init(void);

/*!******************************************************************
 * \fn CLI_status_t CLI_de_init(void)
 * \brief Release CLI driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_de_init(void);

/*!******************************************************************
 * \fn CLI_status_t CLI_process(void)
 * \brief Process CLI driver
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_process(void);

/*!******************************************************************
 * \fn void CLI_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm)
 * \brief Print a downlink frame (only used by the RFP addon during downlink test modes).
 * \param[in]   dl_payload: Downlink payload to print.
 * \param[in]   dl_payload_size: Number of bytes to print.
 * \param[in]   rssi_dbm: RSSI of the received downlink frame (16-bits signed value).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
void CLI_print_dl_payload(sfx_u8* dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm);

/*******************************************************************/
#define CLI_exit_error(base) { ERROR_check_exit(cli_status, CLI_SUCCESS, base) }

/*******************************************************************/
#define CLI_stack_error(base) { ERROR_check_stack(cli_status, CLI_SUCCESS, base) }

/*******************************************************************/
#define CLI_stack_exit_error(base, code) { ERROR_check_stack_exit(cli_status, CLI_SUCCESS, base, code) }

#endif /* SPSWS_MODE_CLI */

#endif /* __CLI_H__ */
