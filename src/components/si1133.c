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
#define SI1133_UV_INDEX_ERROR_VALUE		0xFF
#define SI1133_TIMEOUT_COUNT			1000000

/*** SI1133 local structures ***/

typedef struct {
	unsigned char si1133_uv_index;
} SI1133_Context;

/*** SI1133 local global variables ***/

static SI1133_Context si1133_ctx;

/*** SI1133 local functions ***/

/* WRITE REGISTER(S) OF SI1133.
 * @param addr:		Address of the register to set.
 * @param value:	Value to write in the selected register.
 * @return:			None.
 */
static unsigned char SI1133_WriteRegisters(unsigned char si1133_i2c_address, unsigned char register_address, unsigned char* value_buf, unsigned char value_buf_length) {
	unsigned char register_write_command[SI1133_BURST_WRITE_MAX_LENGTH];
	unsigned char tx_buf_length = value_buf_length + 1; // +1 for register address.
	register_write_command[0] = register_address;
	unsigned char byte_idx = 0;
	// Clamp buffer length.
	if (tx_buf_length >= SI1133_BURST_WRITE_MAX_LENGTH) {
		tx_buf_length = SI1133_BURST_WRITE_MAX_LENGTH - 1; // -1 for register address.
	}
	// Fill data.
	for (byte_idx=1 ; byte_idx<tx_buf_length ; byte_idx++) {
		register_write_command[byte_idx] = value_buf[byte_idx-1];
	}
	unsigned char i2c_access = I2C1_write(si1133_i2c_address, register_write_command, tx_buf_length, 1);
	if (i2c_access == 0) return 0;
	return 1;
}

/* READ A REGISTER OF SI1133.
 * @param addr:		Address of the register to read.
 * @param value:	Pointer to byte that will contain the current value of the selected register.
 * @return:			None.
 */
static unsigned char SI1133_ReadRegister(unsigned char si1133_i2c_address, unsigned char register_address, unsigned char* value) {
	unsigned char local_addr = register_address;
	unsigned char i2c_access = I2C1_write(si1133_i2c_address, &local_addr, 1, 1);
	if (i2c_access == 0) return 0;
	i2c_access = I2C1_read(si1133_i2c_address, value, 1);
	if (i2c_access == 0) return 0;
	return 1;
}

/* WAIT FOR SI1133 TO BE READY.
 * @param:	None.
 * @return:	1 in case of success, 0 in case of failure.
 */
static unsigned char SI1133_WaitUntilSleep(unsigned char si1133_i2c_address) {
	unsigned char response0 = 0;
	unsigned char i2c_access = 0;
	unsigned int loop_count = 0;
	// Wait until SLEEP flag is set.
	do {
		i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_RESPONSE0, &response0);
		if (i2c_access == 0) return 0;
		loop_count++;
		if (loop_count > SI1133_TIMEOUT_COUNT) return 0;
	}
	while ((response0 & (0b1 << 5)) == 0);
	return 1;
}

// Function declaration for next function.
static unsigned char SI1133_SendCommand(unsigned char si1133_i2c_address, unsigned char command);

/* GET CURRENT COMMAND COUNTER.
 * @param si1133_i2c_address:	Sensor address.
 * @param command_counter:		Pointer to byte that will contain current command counter.
 * @return:						1 in case of success, 0 in case of failure.
 */
static unsigned char SI1133_GetCommandCounter(unsigned char si1133_i2c_address, unsigned char* command_counter) {
	// Get counter.
	unsigned char response0 = 0;
	unsigned char i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_RESPONSE0, &response0);
	if (i2c_access == 0) return 0;
	(*command_counter) = response0 & 0x0F;
	// Reset counter when overflow.
	if ((*command_counter) >= 0x0F) {
		i2c_access = SI1133_SendCommand(si1133_i2c_address, SI1133_CMD_RESET_CMD_CTR);
		if (i2c_access == 0) return 0;
	}
	return 1;
}

/* SEND A COMMAND TO SI1133.
 * @param command:	Command to send.
 * @return:			1 in case of success, 0 in case of failure.
 */
static unsigned char SI1133_SendCommand(unsigned char si1133_i2c_address, unsigned char command) {
	// Get current value of counter in RESPONSE0 register.
	unsigned char current_counter = 0;
	unsigned char previous_counter = 0;
	unsigned char i2c_access = 0;
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		i2c_access = SI1133_GetCommandCounter(si1133_i2c_address, &previous_counter);
		if (i2c_access == 0) return 0;
	}
	// Check CMD_ERR flag.
	unsigned char response0 = 0;
	i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_RESPONSE0, &response0);
	if (i2c_access == 0) return 0;
	if ((response0 & 0x10) == 0) {
		// Send command.
		i2c_access = SI1133_WriteRegisters(si1133_i2c_address, SI1133_REG_COMMAND, &command, 1);
		if (i2c_access == 0) return 0;
	}
	// Expect a change in counter.
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		unsigned int loop_count = 0;
		do {
			i2c_access = SI1133_GetCommandCounter(si1133_i2c_address, &current_counter);
			if (i2c_access == 0) return 0;
			i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_RESPONSE0, &response0);
			if (i2c_access == 0) return 0;
			loop_count++;
			if (loop_count > SI1133_TIMEOUT_COUNT) return 0;
		}
		while ((current_counter == previous_counter) && ((response0 & 0x10) == 0));
	}
	return 1;
}

/* SET A SI1133 INTERNAL PARAMETER.
 * @param param:	Parameter address.
 * @param value:	Value to write in parameter.
 * @return:			1 in case of success, 0 in case of failure.
 */
static unsigned char SI1133_SetParameter(unsigned char si1133_i2c_address, unsigned char param, unsigned char value) {
	// Build command.
	unsigned char parameter_write_command[2] = {0, 0};
	parameter_write_command[0] = value;
	parameter_write_command[1] = 0x80 + (param & 0x3F);
	// Wait for sensor to be ready.
	unsigned char i2c_access = SI1133_WaitUntilSleep(si1133_i2c_address);
	if (i2c_access == 0) return 0;
	// Get current value of counter in RESPONSE0 register.
	unsigned char response0 = 0;
	unsigned char previous_counter = 0;
	i2c_access = SI1133_GetCommandCounter(si1133_i2c_address, &previous_counter);
	if (i2c_access == 0) return 0;
	unsigned char current_counter = previous_counter;
	// Send command.
	i2c_access = SI1133_WriteRegisters(si1133_i2c_address, SI1133_REG_HOSTIN0, parameter_write_command, 2);
	if (i2c_access == 0) return 0;
	// Expect a change in counter.
	unsigned int loop_count = 0;
	do {
		i2c_access = SI1133_GetCommandCounter(si1133_i2c_address, &current_counter);
		if (i2c_access == 0) return 0;
		i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_RESPONSE0, &response0);
		if (i2c_access == 0) return 0;
		loop_count++;
		if (loop_count > SI1133_TIMEOUT_COUNT) return 0;
	}
	while ((current_counter == previous_counter) && ((response0 & 0x10) == 0));
	return 1;
}

/* CONFIGURE SI1133 SENSOR.
 * @param:	None.
 * @return:	1 in case of success, 0 in case of failure.
 */
static unsigned char SI1133_Configure(unsigned char si1133_i2c_address) {
	// Configure channel 0 to compute UV index.
	// Parameter settings.
	unsigned char i2c_access = SI1133_SetParameter(si1133_i2c_address, SI1133_PARAM_CH_LIST, 0x01); // Enable channel 0.
	if (i2c_access == 0) return 0;
	i2c_access = SI1133_SetParameter(si1133_i2c_address, SI1133_PARAM_ADCCONFIG0, 0x18); // ADCMUX='11000' (UV index).
	if (i2c_access == 0) return 0;
	i2c_access = SI1133_SetParameter(si1133_i2c_address, SI1133_PARAM_ADCSENS0, 0x71);
	if (i2c_access == 0) return 0;
	i2c_access = SI1133_SetParameter(si1133_i2c_address, SI1133_PARAM_ADCPOST0, 0x00); // 16-bits results.
	if (i2c_access == 0) return 0;
	unsigned char irq0_enable = 0x01;
	i2c_access = SI1133_WriteRegisters(si1133_i2c_address, SI1133_REG_IRQ_ENABLE, &irq0_enable, 1);
	if (i2c_access == 0) return 0;
	return 1;
}

/*** SI1133 functions ***/

/* INIT SI1133 SENSOR.
 * @param:	None.
 * @return:	None.
 */
void SI1133_Init(void) {
	// Init context.
	si1133_ctx.si1133_uv_index = SI1133_UV_INDEX_ERROR_VALUE;
}

/* PERFORM SI1133 UV INDEX MEASUREMENT.
 * @param:	None.
 * @return:	None.
 */
void SI1133_PerformMeasurements(unsigned char si1133_i2c_address) {
	// Configure sensor.
	unsigned char i2c_access = SI1133_Configure(si1133_i2c_address);
	if (i2c_access == 0) return;
	// Start conversion.
	i2c_access = SI1133_SendCommand(si1133_i2c_address, SI1133_CMD_FORCE_CH);
	if (i2c_access == 0) return;
	// Wait for conversion to complete.
	unsigned char response0 = 0;
	unsigned int loop_count = 0;
	do {
		i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_IRQ_STATUS, &response0);
		if (i2c_access == 0) return;
		loop_count++;
		if (loop_count > SI1133_TIMEOUT_COUNT) return;
	}
	while ((response0 & 0x01) == 0); // Wait for IRQ0='1'.
	// Get result.
	unsigned short raw_uv = 0;
	i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_HOSTOUT0, &response0);
	if (i2c_access == 0) return;
	raw_uv |= (response0 << 8);
	i2c_access = SI1133_ReadRegister(si1133_i2c_address, SI1133_REG_HOSTOUT1, &response0);
	if (i2c_access == 0) return;
	raw_uv |= response0;
	// Convert to UV index.
	// UV index = k * ((m * raw^2) + raw) where k = 0.008284 = 1 / 121 and m = -0.000231 = -1 / 4329.
	si1133_ctx.si1133_uv_index = ((((-1) * raw_uv * raw_uv) / 4329) + raw_uv) / (121);
}

/* READ UV INDEX FROM SI1133 SENSOR.
 * @param uv_index:	Pointer to byte that will contain UV index (0 to 11).
 * @return:			None.
 */
void SI1133_GetUvIndex(unsigned char* uv_index) {
	// Get result.
	(*uv_index) = si1133_ctx.si1133_uv_index;
	// Reset results for next conversion.
	si1133_ctx.si1133_uv_index = SI1133_UV_INDEX_ERROR_VALUE;
}

