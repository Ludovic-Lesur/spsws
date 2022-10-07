/*
 * si1133.c
 *
 *  Created on: 28 nov. 2018
 *      Author: Ludo
 */

#include "si1133.h"

#include "i2c.h"
#include "lptim.h"
#include "si1133_reg.h"
#include "types.h"

/*** SI1133 local macros ***/

#define SI1133_BURST_WRITE_MAX_LENGTH	10
#define SI1133_SUB_DELAY_MS				10
#define SI1133_TIMEOUT_MS				2000

/*** SI1133 local global variables ***/

static uint8_t si1133_uv_index;

/*** SI1133 local functions ***/

/* WRITE REGISTER(S) OF SI1133.
 * @param i2c_address:		Sensor address.
 * @param register_address:	Address of the register to set.
 * @param value_buf:		Values buffer to write in the selected register.
 * @param value_buf_length:	Number of values to write.
 * @return status:			Function execution status.
 */
static SI1133_status_t _SI1133_write_register(uint8_t i2c_address, uint8_t register_address, uint8_t* value_buf, uint8_t value_buf_length) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint8_t register_write_command[SI1133_BURST_WRITE_MAX_LENGTH];
	uint8_t tx_buf_length = value_buf_length + 1; // +1 for register address.
	uint8_t idx = 0;
	// Check parameters.
	if (register_address >= SI1133_REG_LAST) {
		status = SI1133_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	if (value_buf == NULL) {
		status = SI1133_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Prepare TX buffer.
	register_write_command[0] = register_address;
	// Clamp buffer length.
	if (tx_buf_length >= SI1133_BURST_WRITE_MAX_LENGTH) {
		tx_buf_length = (SI1133_BURST_WRITE_MAX_LENGTH - 1); // -1 for register address.
	}
	for (idx=1 ; idx<tx_buf_length ; idx++) {
		register_write_command[idx] = value_buf[idx - 1];
	}
	// I2C transfer.
	i2c1_status = I2C1_write(i2c_address, register_write_command, tx_buf_length, 1);
	I2C1_status_check(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/* READ A REGISTER OF SI1133.
 * @param i2c_address:		Sensor address.
 * @param register_address:	Address of the register to set.
 * @param value:			Pointer to 8-bits value that will contain the current value of the selected register.
 * @return status:			Function execution status.
 */
static SI1133_status_t _SI1133_read_register(uint8_t i2c_address, uint8_t register_address, uint8_t* value) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	// Check parameters.
	if (register_address >= SI1133_REG_LAST) {
		status = SI1133_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	if (value == NULL) {
		status = SI1133_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// I2C transfer.
	i2c1_status = I2C1_write(i2c_address, &register_address, 1, 1);
	I2C1_status_check(SI1133_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(i2c_address, value, 1);
	I2C1_status_check(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/* GENERIC FUNCTION TO WAIT FOR A FLAG.
 * @param i2c_address:		Sensor address
 * @param register_address: Address of the register to read.
 * @param bit_index:		Bit index of the flag to wait for.
 * @param timeout_error:	Error to return in case of timeout.
 * @return:					Function execution status.
 */
static SI1133_status_t _SI1133_wait_flag(uint8_t i2c_address, uint8_t register_address, uint8_t bit_index, SI1133_status_t timeout_error) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint8_t reg_value = 0;
	uint32_t loop_count_ms = 0;
	// Check parameters.
	if (register_address >= SI1133_REG_LAST) {
		status = SI1133_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	if (bit_index > 7) {
		status = SI1133_ERROR_REGISTER_BIT_INDEX;
		goto errors;
	}
	// Read register.
	status = _SI1133_read_register(i2c_address, register_address, &reg_value);
	if (status != SI1133_SUCCESS) goto errors;
	// Wait for flag to be set.
	while ((reg_value & (0b1 << bit_index)) == 0) {
		// Low power delay.
		lptim1_status = LPTIM1_delay_milliseconds(SI1133_SUB_DELAY_MS, 1);
		LPTIM1_status_check(SI1133_ERROR_BASE_LPTIM);
		// Exit if timeout.
		loop_count_ms += SI1133_SUB_DELAY_MS;
		if (loop_count_ms > SI1133_TIMEOUT_MS) {
			status = timeout_error;
			goto errors;
		}
		// Read register.
		status = _SI1133_read_register(i2c_address, register_address, &reg_value);
		if (status != SI1133_SUCCESS) goto errors;

	}
errors:
	return status;
}

/* WAIT FOR SI1133 TO BE READY.
 * @param i2c_address:	Sensor address.
 * @return status:		Function execution status.
 */

// Function declaration for next function.
static SI1133_status_t _SI1133_send_command(uint8_t i2c_address, uint8_t command);

/* GET CURRENT COMMAND COUNTER.
 * @param i2c_address:		Sensor address.
 * @param command_counter:	Pointer to 8-bits value that will contain current command counter.
 * @param error_flag:		Pointer to 8-bits value that will contain command error flag.
 * @return status:			Function execution status.
 */
static SI1133_status_t _SI1133_get_status(uint8_t i2c_address, uint8_t* command_counter, uint8_t* error_flag) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	uint8_t response0 = 0;
	if ((command_counter == NULL) || (error_flag == NULL)) {
		status = SI1133_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Get counter.
	status = _SI1133_read_register(i2c_address, SI1133_REG_RESPONSE0, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	// Extract command counter and flag.
	(*command_counter) = response0 & 0x0F;
	(*error_flag) = response0 & 0x10;
	// Reset counter when overflow.
	if ((*command_counter) >= 0x0F) {
		status = _SI1133_send_command(i2c_address, SI1133_CMD_RESET_CMD_CTR);
		if (status != SI1133_SUCCESS) goto errors;
	}
errors:
	return status;
}

/* WAIT FOR COMMAND COMPLETION (COMMAND COUNTER CHANGE OR ERROR).
 * @param i2c_address:		Sensor address.
 * @param previous_counter:	Previous value of the command counter.
 * @param error_base:		Error base to use in case of error.
 * @return status:			Function execution status.
 */
static SI1133_status_t _SI1133_wait_for_command_completion(uint8_t i2c_address, uint8_t previous_counter, SI1133_status_t error_base) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint8_t response0 = 0;
	uint8_t current_counter = 0;
	uint8_t error_flag = 0;
	uint32_t loop_count_ms = 0;
	// Wait for error or command counter change.
	do {
		// Read status register.
		status = _SI1133_get_status(i2c_address, &current_counter, &error_flag);
		if (status != SI1133_SUCCESS) goto errors;
		// Check flag.
		if (error_flag != 0) {
			status = error_base + (response0 & 0x0F);
			goto errors;
		}
		// Low power delay.
		lptim1_status = LPTIM1_delay_milliseconds(SI1133_SUB_DELAY_MS, 1);
		LPTIM1_status_check(SI1133_ERROR_BASE_LPTIM);
		// Exit if timeout.
		loop_count_ms += SI1133_SUB_DELAY_MS;
		if (loop_count_ms > SI1133_TIMEOUT_MS) {
			status = SI1133_ERROR_COMMAND_COUNTER;
			goto errors;
		}
	}
	while ((error_flag == 0) && (current_counter == previous_counter));
errors:
	return status;
}

/* SEND A COMMAND TO SI1133.
 * @param i2c_address:	Sensor address.
 * @param command:		Command to send.
 * @return status:		Function execution status.
 */
static SI1133_status_t _SI1133_send_command(uint8_t i2c_address, uint8_t command) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	uint8_t previous_counter = 0;
	uint8_t error_flag = 0;
	// Check parameters.
	if (command > SI1133_CMD_LAST) {
		status = SI1133_ERROR_COMMAND;
		goto errors;
	}
	// Get current value of counter in RESPONSE0 register.
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		status = _SI1133_get_status(i2c_address, &previous_counter, &error_flag);
		if (status != SI1133_SUCCESS) goto errors;
	}
	// Send command.
	status = _SI1133_write_register(i2c_address, SI1133_REG_COMMAND, &command, 1);
	if (status != SI1133_SUCCESS) goto errors;
	// Wait for completion.
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		status = _SI1133_wait_for_command_completion(i2c_address, previous_counter, SI1133_ERROR_COMMAND_COMPLETION);
		if (status != SI1133_SUCCESS) goto errors;
	}
errors:
	return status;
}

/* SET A SI1133 INTERNAL PARAMETER.
 * @param i2c_address:	Sensor address.
 * @param parameter:	Parameter address.
 * @param value:		Value to write in parameter.
 * @return status:		Function execution status.
 */
static SI1133_status_t _SI1133_set_parameter(uint8_t i2c_address, uint8_t parameter, uint8_t value) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	uint8_t parameter_write_command[2];
	uint8_t previous_counter = 0;
	uint8_t error_flag = 0;
	// Check parameters.
	if (parameter > SI1133_PARAM_LAST) {
		status = SI1133_ERROR_PARAMETER;
		goto errors;
	}
	// Build command.
	parameter_write_command[0] = value;
	parameter_write_command[1] = 0x80 + (parameter & 0x3F);
	// Get current value of counter in RESPONSE0 register.
	status = _SI1133_get_status(i2c_address, &previous_counter, &error_flag);
	if (status != SI1133_SUCCESS) goto errors;
	// Send command.
	status = _SI1133_write_register(i2c_address, SI1133_REG_HOSTIN0, parameter_write_command, 2);
	if (status != SI1133_SUCCESS) goto errors;
	// Wait for completion.
	status = _SI1133_wait_for_command_completion(i2c_address, previous_counter, SI1133_ERROR_PARAMETER_COMPLETION);
	if (status != SI1133_SUCCESS) goto errors;
errors:
	return status;
}

/* CONFIGURE SI1133 SENSOR.
 * @param i2c_address:	Sensor address.
 * @return status:		Function execution status.
 */
static SI1133_status_t _SI1133_configure(uint8_t si1133_i2c_address) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	uint8_t irq0_enable = 0x01;
	// Configure channel 0 to compute UV index.
	status = _SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_CH_LIST, 0x01); // Enable channel 0.
	if (status != SI1133_SUCCESS) goto errors;
	status = _SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_ADCCONFIG0, 0x18); // ADCMUX='11000' (UV index).
	if (status != SI1133_SUCCESS) goto errors;
	status = _SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_ADCSENS0, 0x71);
	if (status != SI1133_SUCCESS) goto errors;
	status = _SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_ADCPOST0, 0x00); // 16-bits results.
	if (status != SI1133_SUCCESS) goto errors;
	status = _SI1133_write_register(si1133_i2c_address, SI1133_REG_IRQ_ENABLE, &irq0_enable, 1);
	if (status != SI1133_SUCCESS) goto errors;
errors:
	return status;
}

/*** SI1133 functions ***/

/* PERFORM SI1133 UV INDEX MEASUREMENT.
 * @param i2c_address:	Sensor address.
 * @return status:		Function execution status.
 */
SI1133_status_t SI1133_perform_measurements(uint8_t i2c_address) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	uint8_t response0 = 0;
	uint16_t raw_uv = 0;
	// Wait for sensor to be ready.
	status = _SI1133_wait_flag(i2c_address, SI1133_REG_RESPONSE0, 5, SI1133_ERROR_READY);
	if (status != SI1133_SUCCESS) goto errors;
	// Configure sensor.
	status = _SI1133_configure(i2c_address);
	if (status != SI1133_SUCCESS) goto errors;
	// Start conversion.
	status = _SI1133_send_command(i2c_address, SI1133_CMD_FORCE_CH);
	if (status != SI1133_SUCCESS) goto errors;
	// Wait for conversion to complete (IRQ0='1').
	status = _SI1133_wait_flag(i2c_address, SI1133_REG_IRQ_STATUS, 0, SI1133_ERROR_TIMEOUT);
	if (status != SI1133_SUCCESS) goto errors;
	// Get result.
	status = _SI1133_read_register(i2c_address, SI1133_REG_HOSTOUT0, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	raw_uv |= (response0 << 8);
	status = _SI1133_read_register(i2c_address, SI1133_REG_HOSTOUT1, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	// Convert to UV index.
	// UV index = k * ((m * raw^2) + raw) where k = 0.008284 = 1 / 121 and m = -0.000231 = -1 / 4329.
	raw_uv |= response0;
	si1133_uv_index = ((((-1) * raw_uv * raw_uv) / 4329) + raw_uv) / (121);
errors:
	return status;
}

/* READ UV INDEX FROM SI1133 SENSOR.
 * @param uv_index:	Pointer to 8-bits value that will contain UV index (0 to 11).
 * @return status:	Function execution status.
 */
SI1133_status_t SI1133_get_uv_index(uint8_t* uv_index) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	// Check parameters.
	if (uv_index == NULL) {
		status = SI1133_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Get result.
	(*uv_index) = si1133_uv_index;
errors:
	return status;
}

