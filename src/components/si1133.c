/*
 * si1133.c
 *
 *  Created on: 28 nov. 2018
 *      Author: Ludo
 */

#include "si1133.h"

#include "i2c.h"
#include "si1133_reg.h"

/*** SI1133 local macros ***/

#define SI1133_BURST_WRITE_MAX_LENGTH	10
#define SI1133_TIMEOUT_COUNT			1000000

/*** SI1133 local global variables ***/

static unsigned char si1133_uv_index;

/*** SI1133 local functions ***/

/* WRITE REGISTER(S) OF SI1133.
 * @param i2c_address:		Sensor address.
 * @param register_address:	Address of the register to set.
 * @param value_buf:		Values buffer to write in the selected register.
 * @param value_buf_length:	Number of values to write.
 * @return status:			Function execution status.
 */
static SI1133_status_t SI1133_write_register(unsigned char i2c_address, unsigned char register_address, unsigned char* value_buf, unsigned char value_buf_length) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	unsigned char register_write_command[SI1133_BURST_WRITE_MAX_LENGTH];
	unsigned char tx_buf_length = value_buf_length + 1; // +1 for register address.
	unsigned char idx = 0;
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
	i2c_status = I2C1_write(i2c_address, register_write_command, tx_buf_length, 1);
	I2C1_status_check(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/* READ A REGISTER OF SI1133.
 * @param i2c_address:		Sensor address.
 * @param register_address:	Address of the register to set.
 * @param value:			Pointer to byte that will contain the current value of the selected register.
 * @return status:			Function execution status.
 */
static SI1133_status_t SI1133_read_register(unsigned char i2c_address, unsigned char register_address, unsigned char* value) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// I2C transfer.
	i2c_status = I2C1_write(i2c_address, &register_address, 1, 1);
	I2C1_status_check(SI1133_ERROR_BASE_I2C);
	i2c_status = I2C1_read(i2c_address, value, 1);
	I2C1_status_check(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/* WAIT FOR SI1133 TO BE READY.
 * @param i2c_address:	Sensor address.
 * @return status:		Function execution status.
 */
static SI1133_status_t SI1133_wait_until_sleep(unsigned char i2c_address) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	unsigned char response0 = 0;
	unsigned int loop_count = 0;
	// Wait until SLEEP flag is set.
	do {
		status = SI1133_read_register(i2c_address, SI1133_REG_RESPONSE0, &response0);
		if (status != SI1133_SUCCESS) goto errors;
		loop_count++;
		if (loop_count > SI1133_TIMEOUT_COUNT) {
			status = SI1133_ERROR_READY;
			goto errors;
		}
	}
	while ((response0 & (0b1 << 5)) == 0);
errors:
	return status;
}

// Function declaration for next function.
static SI1133_status_t SI1133_send_command(unsigned char i2c_address, unsigned char command);

/* GET CURRENT COMMAND COUNTER.
 * @param i2c_address:		Sensor address.
 * @param command_counter:	Pointer to byte that will contain current command counter.
 * @return status:			Function execution status.
 */
static SI1133_status_t SI1133_get_command_counter(unsigned char i2c_address, unsigned char* command_counter) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	unsigned char response0 = 0;
	// Get counter.
	status = SI1133_read_register(i2c_address, SI1133_REG_RESPONSE0, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	// Extract command counter.
	(*command_counter) = response0 & 0x0F;
	// Reset counter when overflow.
	if ((*command_counter) >= 0x0F) {
		status = SI1133_send_command(i2c_address, SI1133_CMD_RESET_CMD_CTR);
		if (status != SI1133_SUCCESS) goto errors;
	}
errors:
	return status;
}

/* SEND A COMMAND TO SI1133.
 * @param i2c_address:	Sensor address.
 * @param command:		Command to send.
 * @return status:		Function execution status.
 */
static SI1133_status_t SI1133_send_command(unsigned char i2c_address, unsigned char command) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	unsigned char response0 = 0;
	unsigned char current_counter = 0;
	unsigned char previous_counter = 0;
	unsigned int loop_count = 0;
	// Wait for sensor to be ready.
	status = SI1133_wait_until_sleep(i2c_address);
	if (status != SI1133_SUCCESS) goto errors;
	// Get current value of counter in RESPONSE0 register.
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		status = SI1133_get_command_counter(i2c_address, &previous_counter);
		if (status != SI1133_SUCCESS) goto errors;
	}
	current_counter = previous_counter;
	// Read CMD_ERR flag.
	status = SI1133_read_register(i2c_address, SI1133_REG_RESPONSE0, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	// Read CMD_ERR flag.
	if ((response0 & 0x10) != 0) {
		status = SI1133_ERROR_COMMAND;
		goto errors;
	}
	// Send command.
	status = SI1133_write_register(i2c_address, SI1133_REG_COMMAND, &command, 1);
	if (status != SI1133_SUCCESS) goto errors;
	// Expect a change in counter.
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		do {
			status = SI1133_get_command_counter(i2c_address, &current_counter);
			if (status != SI1133_SUCCESS) goto errors;
			status = SI1133_read_register(i2c_address, SI1133_REG_RESPONSE0, &response0);
			if (status != SI1133_SUCCESS) goto errors;
			// Exit if timeout or command error flag set.
			loop_count++;
			if ((loop_count > SI1133_TIMEOUT_COUNT) || ((response0 & 0x10) == 0)) {
				status = SI1133_ERROR_COMMAND_COUNTER;
				goto errors;
			}
		}
		while (current_counter == previous_counter);
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
static SI1133_status_t SI1133_set_parameter(unsigned char i2c_address, unsigned char parameter, unsigned char value) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	unsigned char parameter_write_command[2];
	unsigned char response0 = 0;
	unsigned char previous_counter = 0;
	unsigned char current_counter = 0;
	unsigned int loop_count = 0;
	// Build command.
	parameter_write_command[0] = value;
	parameter_write_command[1] = 0x80 + (parameter & 0x3F);
	// Wait for sensor to be ready.
	status = SI1133_wait_until_sleep(i2c_address);
	if (status != SI1133_SUCCESS) goto errors;
	// Get current value of counter in RESPONSE0 register.
	status = SI1133_get_command_counter(i2c_address, &previous_counter);
	if (status != SI1133_SUCCESS) goto errors;
	// Update counter.
	current_counter = previous_counter;
	// Send command.
	status = SI1133_write_register(i2c_address, SI1133_REG_HOSTIN0, parameter_write_command, 2);
	if (status != SI1133_SUCCESS) goto errors;
	// Expect a change in counter.
	do {
		status = SI1133_get_command_counter(i2c_address, &current_counter);
		if (status != SI1133_SUCCESS) goto errors;
		status = SI1133_read_register(i2c_address, SI1133_REG_RESPONSE0, &response0);
		if (status != SI1133_SUCCESS) goto errors;
		// Exit if timeout or command error flag set.
		loop_count++;
		if ((loop_count > SI1133_TIMEOUT_COUNT) || ((response0 & 0x10) == 0)) {
			status = SI1133_ERROR_PARAMETER;
			goto errors;
		}
	}
	while (current_counter == previous_counter);
errors:
	return status;
}

/* CONFIGURE SI1133 SENSOR.
 * @param i2c_address:	Sensor address.
 * @return status:		Function execution status.
 */
static SI1133_status_t SI1133_configure(unsigned char si1133_i2c_address) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	unsigned char irq0_enable = 0x01;
	// Configure channel 0 to compute UV index.
	status = SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_CH_LIST, 0x01); // Enable channel 0.
	if (status != SI1133_SUCCESS) goto errors;
	status = SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_ADCCONFIG0, 0x18); // ADCMUX='11000' (UV index).
	if (status != SI1133_SUCCESS) goto errors;
	status = SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_ADCSENS0, 0x71);
	if (status != SI1133_SUCCESS) goto errors;
	status = SI1133_set_parameter(si1133_i2c_address, SI1133_PARAM_ADCPOST0, 0x00); // 16-bits results.
	if (status != SI1133_SUCCESS) goto errors;
	status = SI1133_write_register(si1133_i2c_address, SI1133_REG_IRQ_ENABLE, &irq0_enable, 1);
	if (status != SI1133_SUCCESS) goto errors;
errors:
	return status;
}

/*** SI1133 functions ***/

/* PERFORM SI1133 UV INDEX MEASUREMENT.
 * @param:	None.
 * @return:	None.
 */
SI1133_status_t SI1133_perform_measurements(unsigned char si1133_i2c_address) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	unsigned char response0 = 0;
	unsigned int loop_count = 0;
	unsigned short raw_uv = 0;
	// Configure sensor.
	status = SI1133_configure(si1133_i2c_address);
	if (status != SI1133_SUCCESS) goto errors;
	// Start conversion.
	status = SI1133_send_command(si1133_i2c_address, SI1133_CMD_FORCE_CH);
	if (status != SI1133_SUCCESS) goto errors;
	// Wait for conversion to complete.
	do {
		status = SI1133_read_register(si1133_i2c_address, SI1133_REG_IRQ_STATUS, &response0);
		if (status != SI1133_SUCCESS) goto errors;
		// Wait for IRQ0='1' or timeout.
		loop_count++;
		if (loop_count > SI1133_TIMEOUT_COUNT) {
			status = SI1133_ERROR_TIMEOUT;
		}
	}
	while ((response0 & 0x01) == 0);
	// Get result.
	status = SI1133_read_register(si1133_i2c_address, SI1133_REG_HOSTOUT0, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	raw_uv |= (response0 << 8);
	status = SI1133_read_register(si1133_i2c_address, SI1133_REG_HOSTOUT1, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	// Convert to UV index.
	// UV index = k * ((m * raw^2) + raw) where k = 0.008284 = 1 / 121 and m = -0.000231 = -1 / 4329.
	raw_uv |= response0;
	si1133_uv_index = ((((-1) * raw_uv * raw_uv) / 4329) + raw_uv) / (121);
errors:
	return status;
}

/* READ UV INDEX FROM SI1133 SENSOR.
 * @param uv_index:	Pointer to byte that will contain UV index (0 to 11).
 * @return:			None.
 */
void SI1133_get_uv_index(unsigned char* uv_index) {
	// Get result.
	(*uv_index) = si1133_uv_index;
}

