/*
 * sensors_hw.h
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#ifndef __SENSORS_HW_H__
#define __SENSORS_HW_H__

#include "error.h"
#include "types.h"

/*** SENSORS HW functions ***/

/*!******************************************************************
 * \fn ERROR_t SENSORS_HW_init(void)
 * \brief Init common sensors hardware interface.
 * \param[in]  	i2c_error_base: Specific I2C error base of the sensor.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ERROR_code_t SENSORS_HW_init(ERROR_code_t i2c_error_base);

/*!******************************************************************
 * \fn ERROR_t SENSORS_HW_de_init(void)
 * \brief Release common sensors hardware interface.
 * \param[in]  	i2c_error_base: Specific I2C error base of the sensor.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ERROR_code_t SENSORS_HW_de_init(ERROR_code_t i2c_error_base);

/*!******************************************************************
 * \fn ERROR_t SENSORS_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag)
 * \brief Write data to sensor over I2C bus.
 * \param[in]  	i2c_error_base: Specific I2C error base of the sensor.
 * \param[in]  	i2c_address: 7-bits sensor address.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[in]	stop_flag: Generate stop condition at the end of the transfer if non zero.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ERROR_code_t SENSORS_HW_i2c_write(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag);

/*!******************************************************************
 * \fn ERROR_t SENSORS_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes)
 * \brief Read data from sensor over I2C bus.
 * \param[in]  	i2c_error_base: Specific I2C error base of the sensor.
 * \param[in]  	i2c_address: 7-bits sensor address.
 * \param[in]	data_size_bytes: Number of bytes to read.
 * \param[out]	data: Byte array that will contain the read data.
 * \retval		Function execution status.
 *******************************************************************/
ERROR_code_t SENSORS_HW_i2c_read(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes);

/*!******************************************************************
 * \fn ERROR_t SENSORS_HW_delay_milliseconds(uint32_t delay_ms)
 * \brief Delay function.
 * \param[in]  	delay_error_base: Specific delay error base of the sensor.
 * \param[in]  	delay_ms: Delay to wait in ms.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ERROR_code_t SENSORS_HW_delay_milliseconds(ERROR_code_t delay_error_base, uint32_t delay_ms);

#endif /* __SENSORS_HW_H__ */
