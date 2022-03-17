/*
 * dps310.c
 *
 *  Created on: 27 nov. 2018
 *      Author: Ludo
 */

#include "dps310.h"

#include "dps310_reg.h"
#include "i2c.h"
#include "math.h"

/*** DPS310 local macros ***/

#define DPS310_TIMEOUT_COUNT				1000000
#define DPS310_SAMPLING_FACTOR_KT			524288
#define DPS310_SAMPLING_FACTOR_KP			1572864

/*** DPS310 local structures ***/

typedef struct {
	// Measurements.
	signed int tmp_raw;
	signed int prs_raw;
	// Calibration coefficients.
	signed int coef_c0;
	signed int coef_c1;
	signed int coef_c00;
	signed int coef_c10;
	signed int coef_c01;
	signed int coef_c11;
	signed int coef_c20;
	signed int coef_c21;
	signed int coef_c30;
} DPS310_context_t;

/*** DPS310 local global variables ***/

static DPS310_context_t dps310_ctx;

/*** DPS310 local functions ***/

/* WRITE A REGISTER OF DPS310.
 * @param i2c_address:		Sensor address
 * @param register_address:	Address of the register to set.
 * @param value:			Value to write in the selected register.
 * @return status			Function execution status.
 */
static DPS310_status_t DPS310_write_register(unsigned char i2c_address, unsigned char register_address, unsigned char value) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	unsigned char register_write_command[2] = {register_address, value};
	// I2C transfer.
	i2c1_status = I2C1_write(i2c_address, register_write_command, 2, 1);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
errors:
	return status;
}

/* READ A REGISTER OF DPS310.
 * @param i2c_address:		Sensor address
 * @param register_address:	Address of the register to read.
 * @param value:			Pointer to byte that will contain the current value of the selected register.
 * @return status			Function execution status.
 */
static DPS310_status_t DPS310_read_register(unsigned char i2c_address, unsigned char register_address, unsigned char* value) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	unsigned char local_addr = register_address;
	// I2C transfer.
	i2c1_status = I2C1_write(i2c_address, &local_addr, 1, 1);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(i2c_address, value, 1);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
errors:
	return status;
}

/* READ ALL SENSOR CALIBRATION COEFFICIENTS.
 * @param i2c_address:	Sensor address.
 * @return status		Function execution status.
 */
static DPS310_status_t DPS310_read_calibration_coefficients(unsigned char i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	unsigned char read_byte = 0;
	unsigned int loop_count = 0;
	// Reset all coefficients.
	unsigned int c0 = 0;
	unsigned int c1 = 0;
	unsigned int c00 = 0;
	unsigned int c10 = 0;
	unsigned int c01 = 0;
	unsigned int c11 = 0;
	unsigned int c20 = 0;
	unsigned int c21 = 0;
	unsigned int c30 = 0;
	// Wait for coefficients to be ready for reading.
	while ((read_byte & (0b1 << 7)) == 0) {
		// Wait for COEF_RDY='1'.
		status = DPS310_read_register(i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (status != DPS310_SUCCESS) goto errors;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) {
			status = DPS310_ERROR_COEFFICIENTS_TIMEOUT;
			goto errors;
		}
	}
	// Read all coefficients.
	read_byte = 0;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C0B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c0 |= (read_byte << 4);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C0A_C1B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c0 |= (read_byte & 0xF0) >> 4;
	c1 |= (read_byte & 0x0F) << 8;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C1A, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c1 |= read_byte;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C00C, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c00 |= (read_byte << 12);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C00B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c00 |= (read_byte << 4);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C00A_C10C, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c00 |= (read_byte & 0xF0) >> 4;
	c10 |= (read_byte & 0x0F) << 16;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C10B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c10 |= (read_byte << 8);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C10A, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c10 |= read_byte;
	status = DPS310_read_register(i2c_address,DPS310_REG_COEF_C01B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c01 |= (read_byte << 8);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C01A, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c01 |= read_byte;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C11B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c11 |= (read_byte << 8);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C11A, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c11 |= read_byte;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C20B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c20 |= (read_byte << 8);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C20A, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c20 |= read_byte;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C21B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c21 |= (read_byte << 8);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C21A, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c21 |= read_byte;
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C30B, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c30 |= (read_byte << 8);
	status = DPS310_read_register(i2c_address, DPS310_REG_COEF_C30A, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	c30 |= read_byte;
	// Convert to sign values.
	math_status = MATH_two_complement(c0, 11, &dps310_ctx.coef_c0);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c1, 11, &dps310_ctx.coef_c1);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c00, 19, &dps310_ctx.coef_c00);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c01, 15, &dps310_ctx.coef_c01);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c10, 19, &dps310_ctx.coef_c10);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c11, 15, &dps310_ctx.coef_c11);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c20, 15, &dps310_ctx.coef_c20);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c21, 15, &dps310_ctx.coef_c21);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
	math_status = MATH_two_complement(c30, 15, &dps310_ctx.coef_c30);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
errors:
	return status;
}

/* PERFORM TEMPERATURE MEASUREMENT.
 * @param i2c_address:	Sensor address.
 * @return status		Function execution status.
 */
static DPS310_status_t DPS310_compute_raw_temperature(unsigned char i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	unsigned char read_byte = 0;
	unsigned int loop_count = 0;
	unsigned int tmp_raw = 0;
	// Wait for sensor to be ready.
	while ((read_byte & (0b1 << 6)) == 0) {
		// Wait for SENSOR_RDY='1' or timeout.
		status = DPS310_read_register(i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (status != DPS310_SUCCESS) goto errors;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) {
			status = DPS310_ERROR_SENSOR_TIMEOUT;
			goto errors;
		}
	}
	// Trigger temperature measurement.
	status = DPS310_write_register(i2c_address, DPS310_REG_TMP_CFG, 0x80); // External sensor, rate=1meas/s, no oversampling.
	if (status != DPS310_SUCCESS) goto errors;
	status = DPS310_write_register(i2c_address, DPS310_REG_MEAS_CFG, 0x02);
	if (status != DPS310_SUCCESS) goto errors;
	// Wait for temperature to be ready.
	loop_count = 0;
	while ((read_byte & (0b1 << 5)) == 0) {
		// Wait for TMP_RDY='1' or timeout.
		status = DPS310_read_register(i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (status != DPS310_SUCCESS) goto errors;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) {
			status = DPS310_ERROR_TEMPERATURE_TIMEOUT;
			goto errors;
		}
	}
	// Read temperature.
	// B2.
	status = DPS310_read_register(i2c_address, DPS310_REG_TMP_B2, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	tmp_raw |= (read_byte << 16);
	// B1.
	status = DPS310_read_register(i2c_address, DPS310_REG_TMP_B1, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	tmp_raw |= (read_byte << 8);
	// B0.
	status = DPS310_read_register(i2c_address, DPS310_REG_TMP_B0, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	tmp_raw |= read_byte;
	// Compute two complement.
	math_status = MATH_two_complement(tmp_raw, 23, &dps310_ctx.tmp_raw);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
errors:
	return status;
}

/* PERFORM PRESSURE MEASUREMENT.
 * @param dps310_i2c_address:	Sensor address.
 * @return:						1 in case of success, 0 in case of failure.
 */
static unsigned char DPS310_compute_raw_pressure(unsigned char dps310_i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	unsigned char read_byte = 0;
	unsigned int loop_count = 0;
	unsigned int prs_raw = 0;
	// Wait for sensor to be ready.
	while ((read_byte & (0b1 << 6)) == 0) {
		// Wait for SENSOR_RDY='1' or timeout.
		status = DPS310_read_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (status != DPS310_SUCCESS) goto errors;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) {
			status = DPS310_ERROR_SENSOR_TIMEOUT;
			goto errors;
		}
	}
	// Trigger pressure measurement.
	status = DPS310_write_register(dps310_i2c_address, DPS310_REG_PRS_CFG, 0x01); // Rate=1meas/s, no oversampling.
	if (status != DPS310_SUCCESS) goto errors;
	status = DPS310_write_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, 0x01);
	if (status != DPS310_SUCCESS) goto errors;
	// Wait for pressure to be ready.
	read_byte = 0;
	loop_count = 0;
	while ((read_byte & (0b1 << 4)) == 0) {
		// Wait for PRS_RDY='1'or timeout.
		status = DPS310_read_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (status != DPS310_SUCCESS) goto errors;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) {
			status = DPS310_ERROR_PRESSURE_TIMEOUT;
			goto errors;
		}
	}
	// Read pressure.
	// B2.
	status = DPS310_read_register(dps310_i2c_address, DPS310_REG_PRS_B2, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	prs_raw |= (read_byte << 16);
	// B1.
	status = DPS310_read_register(dps310_i2c_address, DPS310_REG_PRS_B1, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	prs_raw |= (read_byte << 8);
	// B0.
	status = DPS310_read_register(dps310_i2c_address, DPS310_REG_PRS_B0, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	prs_raw |= read_byte;
	// Compute two complement.
	math_status = MATH_two_complement(prs_raw, 23, &dps310_ctx.prs_raw);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
errors:
	return status;
}

/*** DPS310 functions ***/

/* PERFORM PRESSURE AND TEMPERATURE MEASUREMENT.
 * @param i2c_address:	Sensor address.
 * @return status :		Function execution status.
 */
DPS310_status_t DPS310_perform_measurements(unsigned char i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	// Reset result.
	dps310_ctx.tmp_raw = 0;
	dps310_ctx.prs_raw = 0;
	// Compute raw results.
	status = DPS310_compute_raw_temperature(i2c_address);
	if (status != DPS310_SUCCESS) goto errors;
	status = DPS310_compute_raw_pressure(i2c_address);
	if (status != DPS310_SUCCESS) goto errors;
	// Read calibration coefficients.
	status = DPS310_read_calibration_coefficients(i2c_address);
errors:
	return status;
}

/* READ PRESSURE FROM DPS310 SENSOR.
 * @param pressure_pa:	Pointer to integer that will contain pressure result (Pa).
 * @return:				None.
 */
void DPS310_get_pressure(unsigned int* pressure_pa) {
	// Local variables.
	signed long long psr_temp = 0;
	signed long long last_term = 0;
	// Compute pressure in Pa.
	psr_temp = dps310_ctx.coef_c20 + (dps310_ctx.prs_raw * dps310_ctx.coef_c30) / DPS310_SAMPLING_FACTOR_KP;
	psr_temp = dps310_ctx.coef_c10 + (dps310_ctx.prs_raw * psr_temp) / DPS310_SAMPLING_FACTOR_KP;
	psr_temp = dps310_ctx.coef_c00 + (dps310_ctx.prs_raw * psr_temp) / DPS310_SAMPLING_FACTOR_KP;
	psr_temp += (dps310_ctx.tmp_raw * dps310_ctx.coef_c01) / DPS310_SAMPLING_FACTOR_KT;
	last_term = dps310_ctx.coef_c11 + (dps310_ctx.prs_raw * dps310_ctx.coef_c21) / DPS310_SAMPLING_FACTOR_KP;
	last_term = (dps310_ctx.prs_raw * last_term) / DPS310_SAMPLING_FACTOR_KP;
	last_term = (dps310_ctx.tmp_raw * last_term) / DPS310_SAMPLING_FACTOR_KT;
	psr_temp += last_term;
	(*pressure_pa) = (unsigned int) psr_temp;
}

/* READ TEMPERATURE FROM DPS310 SENSOR.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (degrees).
 * @return:						None.
 */
void DPS310_get_temperature(signed char* temperature_degrees) {
	// Local variables.
	signed long long tmp_temp = 0;

	// Compute temperature in degrees.
	tmp_temp = (dps310_ctx.coef_c0 / 2) + (dps310_ctx.coef_c1 * dps310_ctx.tmp_raw) / DPS310_SAMPLING_FACTOR_KT;
	(*temperature_degrees) = (unsigned char) tmp_temp;
}
