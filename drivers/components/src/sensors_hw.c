/*
 * sensors_hw.c
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#include "sensors_hw.h"

#include "error.h"
#include "error_base.h"
#include "gpio_mapping.h"
#include "i2c.h"
#include "lptim.h"
#include "sen15901_hw.h"
#include "spsws_flags.h"
#include "types.h"

/*** SENSORS HW local macros ***/

#define SENSORS_I2C_INSTANCE    I2C_INSTANCE_I2C1

/*** SENSORS HW local global variables ***/

#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
static SEN15901_HW_tick_second_irq_cb_t sensors_hw_sen15901_tick_second_callback = NULL;
#endif

/*** SENSORS HW functions ***/

/*******************************************************************/
ERROR_code_t SENSORS_HW_init(ERROR_code_t i2c_error_base) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // Init I2C.
    i2c_status = I2C_init(SENSORS_I2C_INSTANCE, &GPIO_SENSORS_I2C);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_de_init(ERROR_code_t i2c_error_base) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // Init I2C.
    i2c_status = I2C_de_init(SENSORS_I2C_INSTANCE, &GPIO_SENSORS_I2C);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_i2c_write(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // I2C transfer.
    i2c_status = I2C_write(SENSORS_I2C_INSTANCE, i2c_address, data, data_size_bytes, stop_flag);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_i2c_read(ERROR_code_t i2c_error_base, uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    I2C_status_t i2c_status = I2C_SUCCESS;
    // I2C transfer.
    i2c_status = I2C_read(SENSORS_I2C_INSTANCE, i2c_address, data, data_size_bytes);
    I2C_exit_error(i2c_error_base);
errors:
    return status;
}

/*******************************************************************/
ERROR_code_t SENSORS_HW_delay_milliseconds(ERROR_code_t delay_error_base, uint32_t delay_ms) {
    // Local variables.
    ERROR_code_t status = SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(delay_error_base);
errors:
    return status;
}

#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
/*******************************************************************/
void SENSORS_HW_set_sen15901_tick_second_callback(SEN15901_HW_tick_second_irq_cb_t tick_second_callback) {
    sensors_hw_sen15901_tick_second_callback = tick_second_callback;
}
#endif

#ifdef SPSWS_WIND_RAINFALL_MEASUREMENTS
/*******************************************************************/
void SENSORS_HW_get_sen15901_tick_second_callback(SEN15901_HW_tick_second_irq_cb_t* tick_second_callback) {
    (*tick_second_callback) = sensors_hw_sen15901_tick_second_callback;
}
#endif
