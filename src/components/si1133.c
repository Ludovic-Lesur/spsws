/*
 * si1133.c
 *
 *  Created on: 28 nov. 2018
 *      Author: Ludovic
 */

#include "si1133.h"

#include "i2c.h"
#include "si1133_reg.h"

/*** SI1133 local macros ***/

#define SI1133_I2C_ADDRESS				0x52
#define SI1133_BURST_WRITE_MAX_LENGTH	10

/*** SI1133 local functions ***/

/* WRITE REGISTER(S) OF SI1133.
 * @param addr:		Address of the register to set.
 * @param value:	Value to write in the selected register.
 * @return:			None.
 */
void SI1133_WriteRegisters(unsigned char addr, unsigned char* value_buf, unsigned char value_buf_length) {
	unsigned char register_write_command[SI1133_BURST_WRITE_MAX_LENGTH];
	unsigned char tx_buf_length = value_buf_length + 1; // +1 for register address.
	register_write_command[0] = addr;
	unsigned char byte_idx = 0;

	/* Clamp buffer length */
	if (tx_buf_length >= SI1133_BURST_WRITE_MAX_LENGTH) {
		tx_buf_length = SI1133_BURST_WRITE_MAX_LENGTH - 1; // -1 for register address.
	}

	/* Fill data */
	for (byte_idx=1 ; byte_idx<tx_buf_length ; byte_idx++) {
		register_write_command[byte_idx] = value_buf[byte_idx-1];
	}
	I2C_Write(SI1133_I2C_ADDRESS, register_write_command, tx_buf_length);
}

/* READ A REGISTER OF SI1133.
 * @param addr:		Address of the register to read.
 * @param value:	Pointer to byte that will contain the current value of the selected register.
 * @return:			None.
 */
void SI1133_ReadRegister(unsigned char addr, unsigned char* value) {
	unsigned char local_addr = addr;
	I2C_Write(SI1133_I2C_ADDRESS, &local_addr, 1);
	I2C_Read(SI1133_I2C_ADDRESS, value, 1);
}

/* WAIT FOR SI1133 TO BE READY.
 * @param:	None.
 * @return:	None.
 */
void SI1133_WaitUntilSleep(void) {
	unsigned char response0 = 0;
	// Wait until SLEEP flag is set.
	do {
		SI1133_ReadRegister(SI1133_REG_RESPONSE0, &response0);
	}
	while ((response0 & (0b1 << 5)) == 0);
}

// Function declaration for next function.
void SI1133_SendCommand(unsigned char command);

/* GET CURRENT COMMAND COUNTER.
 * @param:	None.
 * @return:	Current command counter (RESPONSE0 register).
 */
unsigned char SI1133_GetCommandCounter(void) {

	/* Get counter */
	unsigned char response0 = 0;
	SI1133_ReadRegister(SI1133_REG_RESPONSE0, &response0);
	unsigned char current_counter = response0 & 0x0F;

	/* Reset counter when overflow */
	if (current_counter >= 0x0F) {
		SI1133_SendCommand(SI1133_CMD_RESET_CMD_CTR);
	}
	return current_counter;
}

/* SEND A COMMAND TO SI1133.
 * @param command:	Command to send.
 * @return:			None.
 */
void SI1133_SendCommand(unsigned char command) {

	/* Get current value of counter in RESPONSE0 register */
	unsigned char current_counter = 0;
	unsigned char previous_counter = 0;
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		previous_counter = SI1133_GetCommandCounter();
	}

	/* Check CMD_ERR flag */
	unsigned char response0 = 0;
	SI1133_ReadRegister(SI1133_REG_RESPONSE0, &response0);
	if ((response0 & 0x10) == 0) {

		/* Send command */
		SI1133_WriteRegisters(SI1133_REG_COMMAND, &command, 1);
	}

	/* Expect a change in counter */
	if (command != SI1133_CMD_RESET_CMD_CTR) {
		do {
			current_counter = SI1133_GetCommandCounter();
			SI1133_ReadRegister(SI1133_REG_RESPONSE0, &response0);
		}
		while ((current_counter == previous_counter) && ((response0 & 0x10) == 0));
	}
}

/* SET A SI1133 INTERNAL PARAMETER.
 * @param param:	Parameter address.
 * @param value:	Value to write in parameter.
 * @return:			None.
 */
void SI1133_SetParameter(unsigned char param, unsigned char value) {

	/* Build command */
	unsigned char parameter_write_command[2] = {0, 0};
	parameter_write_command[0] = value;
	parameter_write_command[1] = 0x80 + (param & 0x3F);

	/* Wait for sensor to be ready */
	SI1133_WaitUntilSleep();

	/* Get current value of counter in RESPONSE0 register */
	unsigned char response0 = 0;
	unsigned char previous_counter = SI1133_GetCommandCounter();
	unsigned char current_counter = previous_counter;

	/* Send command */
	SI1133_WriteRegisters(SI1133_REG_HOSTIN0, parameter_write_command, 2);

	/* Expect a change in counter */
	do {
		current_counter = SI1133_GetCommandCounter();
		SI1133_ReadRegister(SI1133_REG_RESPONSE0, &response0);
	}
	while ((current_counter == previous_counter) && ((response0 & 0x10) == 0));
}

/* CONFIGURE SI1133 SENSOR.
 * @param:	None.
 * @return:	None.
 */
void SI1133_Configure(void) {

	/* Configure channel 0 to compute UV index */
	// Parameter settings.
	SI1133_SetParameter(SI1133_PARAM_CH_LIST, 0x01); // Enable channel 0.
	SI1133_SetParameter(SI1133_PARAM_ADCCONFIG0, 0x18); // ADCMUX='11000' (UV index).
	SI1133_SetParameter(SI1133_PARAM_ADCSENS0, 0x71);
	SI1133_SetParameter(SI1133_PARAM_ADCPOST0, 0x00); // 16-bits results.
	unsigned char irq0_enable = 0x01;
	SI1133_WriteRegisters(SI1133_REG_IRQ_ENABLE, &irq0_enable, 1);
}

/* READ UV INDEX FROM SI1133 SENSOR.
 * @param uv_index:	Pointer to byte that will contain UV index (0 to 11).
 * @return:			None.
 */
void SI1133_GetUvIndex(unsigned char* uv_index) {

	/* Confiure sensor */
	SI1133_Configure();

	/* Start conversion */
	SI1133_SendCommand(SI1133_CMD_FORCE_CH);

	/* Wait for conversion to complete */
	unsigned char response0;
	do {
		SI1133_ReadRegister(SI1133_REG_IRQ_STATUS, &response0);
	}
	while ((response0 & 0x01) == 0); // Wait for IRQ0='1'.

	/* Get result */
	unsigned short raw_uv = 0;
	SI1133_ReadRegister(SI1133_REG_HOSTOUT0, &response0);
	raw_uv |= (response0 << 8);
	SI1133_ReadRegister(SI1133_REG_HOSTOUT1, &response0);
	raw_uv |= response0;

	/* Convert to UV index */
	// UV index = k * ((m * raw^2) + raw) where k = 0.008284 = 1 / 121 and m = -0.000231 = -1 / 4329.
	(*uv_index) = ((((-1) * raw_uv * raw_uv) / 4329) + raw_uv) / (121);
}

