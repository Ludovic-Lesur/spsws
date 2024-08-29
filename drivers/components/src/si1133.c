/*
 * si1133.c
 *
 *  Created on: 28 nov. 2018
 *      Author: Ludo
 */

#include "si1133.h"

#include "error.h"
#include "gpio_mapping.h"
#include "i2c.h"
#include "lptim.h"
#include "types.h"

/*** SI1133 local macros ***/

#define SI1133_I2C_INSTANCE				I2C_INSTANCE_I2C1

#define SI1133_BURST_WRITE_MAX_LENGTH	10
#define SI1133_SUB_DELAY_MS				10
#define SI1133_TIMEOUT_MS				2000

#define SI1133_REG_PART_ID  			0x00
#define SI1133_REG_HW_ID   				0x01
#define SI1133_REG_REV_ID   			0x02
#define SI1133_REG_HOSTIN0   			0x0A
#define SI1133_REG_COMMAND   			0x0B
#define SI1133_REG_IRQ_ENABLE   		0x0F
#define SI1133_REG_RESPONSE1   			0x10
#define SI1133_REG_RESPONSE0   			0x11
#define SI1133_REG_IRQ_STATUS  			0x12
#define SI1133_REG_HOSTOUT0   			0x13
#define SI1133_REG_HOSTOUT1   			0x14
#define SI1133_REG_HOSTOUT2   			0x15
#define SI1133_REG_HOSTOUT3   			0x16
#define SI1133_REG_HOSTOUT4   			0x17
#define SI1133_REG_HOSTOUT5   			0x18
#define SI1133_REG_HOSTOUT6   			0x19
#define SI1133_REG_HOSTOUT7   			0x1A
#define SI1133_REG_HOSTOUT8   			0x1B
#define SI1133_REG_HOSTOUT9   			0x1C
#define SI1133_REG_HOSTOUT10   			0x1D
#define SI1133_REG_HOSTOUT11   			0x1E
#define SI1133_REG_HOSTOUT12   			0x1F
#define SI1133_REG_HOSTOUT13   			0x20
#define SI1133_REG_HOSTOUT14   			0x21
#define SI1133_REG_HOSTOUT15   			0x22
#define SI1133_REG_HOSTOUT16   			0x23
#define SI1133_REG_HOSTOUT17   			0x24
#define SI1133_REG_HOSTOUT18   			0x25
#define SI1133_REG_HOSTOUT19   			0x26
#define SI1133_REG_HOSTOUT20   			0x27
#define SI1133_REG_HOSTOUT21   			0x28
#define SI1133_REG_HOSTOUT22   			0x29
#define SI1133_REG_HOSTOUT23   			0x2A
#define SI1133_REG_HOSTOUT24   			0x2B
#define SI1133_REG_HOSTOUT25   			0x2C
#define SI1133_REG_LAST					0x2D

#define SI1133_PARAM_I2C_ADDR   		0x00
#define SI1133_PARAM_CH_LIST   			0x01
#define SI1133_PARAM_ADCCONFIG0   		0x02
#define SI1133_PARAM_ADCSENS0   		0x03
#define SI1133_PARAM_ADCPOST0   		0x04
#define SI1133_PARAM_MEASCONFIG0   		0x05
#define SI1133_PARAM_ADCCONFIG1   		0x06
#define SI1133_PARAM_ADCSENS1   		0x07
#define SI1133_PARAM_ADCPOST1  	 		0x08
#define SI1133_PARAM_MEASCONFIG1   		0x09
#define SI1133_PARAM_ADCCONFIG2   		0x0A
#define SI1133_PARAM_ADCSENS2   		0x0B
#define SI1133_PARAM_ADCPOST2   		0x0C
#define SI1133_PARAM_MEASCONFIG2   		0x0D
#define SI1133_PARAM_ADCCONFIG3   		0x0E
#define SI1133_PARAM_ADCSENS3   		0x0F
#define SI1133_PARAM_ADCPOST3   		0x10
#define SI1133_PARAM_MEASCONFIG3   		0x11
#define SI1133_PARAM_ADCCONFIG4   		0x12
#define SI1133_PARAM_ADCSENS4  			0x13
#define SI1133_PARAM_ADCPOST4   		0x14
#define SI1133_PARAM_MEASCONFIG4   		0x15
#define SI1133_PARAM_ADCCONFIG5   		0x16
#define	SI1133_PARAM_ADCSENS5   		0x17
#define SI1133_PARAM_ADCPOST5   		0x18
#define SI1133_PARAM_MEASCONFIG5   		0x19
#define SI1133_PARAM_MEASRATE_H   		0x1A
#define SI1133_PARAM_MEASRATE_L   		0x1B
#define SI1133_PARAM_MEASCOUNT0   		0x1C
#define SI1133_PARAM_MEASCOUNT1   		0x1D
#define SI1133_PARAM_MEASCOUNT2   		0x1E
#define SI1133_PARAM_THRESHOLD0_H   	0x25
#define SI1133_PARAM_THRESHOLD0_L   	0x26
#define SI1133_PARAM_THRESHOLD1_H   	0x27
#define SI1133_PARAM_THRESHOLD1_L   	0x28
#define SI1133_PARAM_THRESHOLD2_H   	0x29
#define SI1133_PARAM_THRESHOLD2_L   	0x2A
#define SI1133_PARAM_BURST   			0x2B
#define SI1133_PARAM_LAST				0x2C

#define SI1133_CMD_RESET_CMD_CTR   		0x00
#define SI1133_CMD_RESET   				0x01
#define SI1133_CMD_NEW_ADDR  			0x02
#define SI1133_CMD_FORCE_CH   			0x11
#define SI1133_CMD_PAUSE_CH  			0x12
#define SI1133_CMD_START   				0x13
#define SI1133_CMD_PARAM_QUERY  		0x40
#define SI1133_CMD_PARAM_SET   			0x80
#define SI1133_CMD_LAST					0x81

/*** SI1133 local global variables ***/

static int32_t si1133_uv_index;

/*** SI1133 local functions ***/

/*******************************************************************/
static SI1133_status_t _SI1133_write_register(uint8_t i2c_address, uint8_t register_address, uint8_t* value_buf, uint8_t value_buf_length) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
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
	i2c_status = I2C_write(SI1133_I2C_INSTANCE, i2c_address, register_write_command, tx_buf_length, 1);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_read_register(uint8_t i2c_address, uint8_t register_address, uint8_t* value) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
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
	i2c_status = I2C_write(SI1133_I2C_INSTANCE, i2c_address, &register_address, 1, 1);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
	i2c_status = I2C_read(SI1133_I2C_INSTANCE, i2c_address, value, 1);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_wait_flag(uint8_t i2c_address, uint8_t register_address, uint8_t bit_index, SI1133_status_t timeout_error) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
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
		lptim_status = LPTIM_delay_milliseconds(SI1133_SUB_DELAY_MS, LPTIM_DELAY_MODE_STOP);
		LPTIM_exit_error(SI1133_ERROR_BASE_LPTIM);
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

/*******************************************************************/
static SI1133_status_t _SI1133_send_command(uint8_t i2c_address, uint8_t command);

/*******************************************************************/
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

/*******************************************************************/
static SI1133_status_t _SI1133_wait_for_command_completion(uint8_t i2c_address, uint8_t previous_counter, SI1133_status_t error_base) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
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
		lptim_status = LPTIM_delay_milliseconds(SI1133_SUB_DELAY_MS, LPTIM_DELAY_MODE_STOP);
		LPTIM_exit_error(SI1133_ERROR_BASE_LPTIM);
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

/*******************************************************************/
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

/*******************************************************************/
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

/*******************************************************************/
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

/*******************************************************************/
SI1133_status_t SI1133_init(void) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// Init I2C.
	i2c_status = I2C_init(SI1133_I2C_INSTANCE, &GPIO_SENSORS_I2C);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SI1133_status_t SI1133_de_init(void) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	I2C_status_t i2c_status = I2C_SUCCESS;
	// Init I2C.
	i2c_status = I2C_de_init(SI1133_I2C_INSTANCE, &GPIO_SENSORS_I2C);
	I2C_exit_error(SI1133_ERROR_BASE_I2C);
errors:
	return status;
}

/*******************************************************************/
SI1133_status_t SI1133_perform_measurements(uint8_t i2c_address) {
	// Local variables.
	SI1133_status_t status = SI1133_SUCCESS;
	uint8_t response0 = 0;
	int32_t raw_uv = 0;
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
	raw_uv |= (int32_t) (response0 << 8);
	status = _SI1133_read_register(i2c_address, SI1133_REG_HOSTOUT1, &response0);
	if (status != SI1133_SUCCESS) goto errors;
	// Convert to UV index.
	// UV index = k * ((m * raw^2) + raw) where k = 0.008284 = 1 / 121 and m = -0.000231 = -1 / 4329.
	raw_uv |= (int32_t) response0;
	si1133_uv_index = ((((-1) * raw_uv * raw_uv) / 4329) + raw_uv) / (121);
errors:
	return status;
}

/*******************************************************************/
SI1133_status_t SI1133_get_uv_index(int32_t* uv_index) {
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

