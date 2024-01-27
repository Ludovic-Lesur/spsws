/*
 * aes.h
 *
 *  Created on: 19 jun. 2018
 *      Author: Ludo
 */

#ifndef __AES_H__
#define __AES_H__

#include "types.h"

/*** AES structures ***/

/*!******************************************************************
 * \enum AES_status_t
 * \brief AES driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	AES_SUCCESS = 0,
	AES_ERROR_NULL_PARAMETER,
	AES_ERROR_TIMEOUT,
	// Last base value.
	AES_ERROR_BASE_LAST = 0x0100
} AES_status_t;

/*** AES functions ***/

/*!******************************************************************
 * \fn void AES_init(void)
 * \brief Init AES peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AES_init(void);

/*!******************************************************************
 * \fn void AES_de_init(void)
 * \brief Release AES peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void AES_de_init(void);

/*!******************************************************************
 * \fn AES_status_t AES_encrypt(uint8_t* data_in, uint8_t* data_out, uint8_t* key)
 * \brief Compute AES-128.
 * \param[in]  	data_in: Input data.
 * \param[in]	key: AES key.
 * \param[out] 	data_out: Output data.
 * \retval		Function execution status.
 *******************************************************************/
AES_status_t AES_encrypt(uint8_t* data_in, uint8_t* data_out, uint8_t* key);

/*******************************************************************/
#define AES_exit_error(error_base) { if (aes_status != AES_SUCCESS) { status = (error_base + aes_status); goto errors; } }

/*******************************************************************/
#define AES_stack_error(void) { if (aes_status != AES_SUCCESS) { ERROR_stack_add(ERROR_BASE_AES + aes_status); } }

/*******************************************************************/
#define AES_stack_exit_error(error_code) { if (aes_status != AES_SUCCESS) { ERROR_stack_add(ERROR_BASE_AES + aes_status); status = error_code; goto errors; } }

#endif /* __AES_H__ */
