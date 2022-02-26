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
#define DPS310_RAW_ERROR_VALUE				0X7FFFFFFF
#define DPS310_TEMPERATURE_ERROR_VALUE		0x7F

/*** DPS310 local structures ***/

typedef struct {
	// Sampling factors.
	signed int kT;
	signed int kP;
	// Measurements.
	signed int tmp_raw;
	signed int prs_raw;
	// Calibration coefficients.
	unsigned int coef_c0;
	unsigned int coef_c1;
	unsigned int coef_c00;
	unsigned int coef_c10;
	unsigned int coef_c01;
	unsigned int coef_c11;
	unsigned int coef_c20;
	unsigned int coef_c21;
	unsigned int coef_c30;
} DPS310_context_t;

/*** DPS310 local global variables ***/

static DPS310_context_t dps310_ctx;

/*** DPS310 local functions ***/

/* WRITE A REGISTER OF DPS310.
 * @param dps310_i2c_address:	Sensor address
 * @param register_address:		Address of the register to set.
 * @param value:				Value to write in the selected register.
 * @return:						1 in case of success, 0 in case of failure.
 */
static unsigned char DPS310_write_register(unsigned char dps310_i2c_address, unsigned char register_address, unsigned char value) {
	unsigned char register_write_command[2] = {register_address, value};
	unsigned char i2c_access = I2C1_write(dps310_i2c_address, register_write_command, 2, 1);
	if (i2c_access == 0) return 0;
	return 1;
}

/* READ A REGISTER OF DPS310.
 * @param dps310_i2c_address:	Sensor address
 * @param register_address:		Address of the register to read.
 * @param value:				Pointer to byte that will contain the current value of the selected register.
 * @return:						1 in case of success, 0 in case of failure.
 */
static unsigned char DPS310_read_register(unsigned char dps310_i2c_address, unsigned char register_address, unsigned char* value) {
	unsigned char local_addr = register_address;
	unsigned char i2c_access = I2C1_write(dps310_i2c_address, &local_addr, 1, 1);
	if (i2c_access == 0) return 0;
	i2c_access = I2C1_read(dps310_i2c_address, value, 1);
	if (i2c_access == 0) return 0;
	return 1;
}

/* READ ALL SENSOR CALIBRATION COEFFICIENTS.
 * @param dps310_i2c_address:	Sensor address.
 * @return:						1 in case of success, 0 in case of failure.
 */
static unsigned char DPS310_read_calibration_coefficients(unsigned char dps310_i2c_address) {
	// Reset all coefficients.
	dps310_ctx.coef_c0 = 0;
	dps310_ctx.coef_c1 = 0;
	dps310_ctx.coef_c00 = 0;
	dps310_ctx.coef_c10 = 0;
	dps310_ctx.coef_c01 = 0;
	dps310_ctx.coef_c10 = 0;
	dps310_ctx.coef_c11 = 0;
	dps310_ctx.coef_c20 = 0;
	dps310_ctx.coef_c21 = 0;
	dps310_ctx.coef_c30 = 0;
	// Wait for coefficients to be ready for reading.
	unsigned char read_byte = 0;
	unsigned char i2c_access = 0;
	unsigned int loop_count = 0;
	while ((read_byte & (0b1 << 7)) == 0) { // Wait for COEF_RDY='1'.
		i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (i2c_access == 0) return 0;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) return 0;
	}
	// Read all coefficients.
	read_byte = 0;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C0B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c0 |= (read_byte << 4);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C0A_C1B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c0 |= (read_byte & 0xF0) >> 4;
	dps310_ctx.coef_c1 |= (read_byte & 0x0F) << 8;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C1A, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c1 |= read_byte;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C00C, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c00 |= (read_byte << 12);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C00B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c00 |= (read_byte << 4);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C00A_C10C, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c00 |= (read_byte & 0xF0) >> 4;
	dps310_ctx.coef_c10 |= (read_byte & 0x0F) << 16;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C10B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c10 |= (read_byte << 8);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C10A, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c10 |= read_byte;
	i2c_access = DPS310_read_register(dps310_i2c_address,DPS310_REG_COEF_C01B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c01 |= (read_byte << 8);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C01A, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c01 |= read_byte;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C11B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c11 |= (read_byte << 8);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C11A, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c11 |= read_byte;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C20B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c20 |= (read_byte << 8);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C20A, &read_byte);
	if (i2c_access == 0) return 0;
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c20 |= read_byte;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C21B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c21 |= (read_byte << 8);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C21A, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c21 |= read_byte;
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C30B, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c30 |= (read_byte << 8);
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_COEF_C30A, &read_byte);
	if (i2c_access == 0) return 0;
	dps310_ctx.coef_c30 |= read_byte;
	return 1;
}

/* PERFORM TEMPERATURE MEASUREMENT.
 * @param dps310_i2c_address:	Sensor address.
 * @return:						1 in case of success, 0 in case of failure.
 */
static unsigned char DPS310_compute_raw_temperature(unsigned char dps310_i2c_address) {
	// Wait for sensor to be ready.
	unsigned char read_byte = 0;
	unsigned char i2c_access = 0;
	unsigned int loop_count = 0;
	while ((read_byte & (0b1 << 6)) == 0) { // Wait for SENSOR_RDY='1'.
		i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (i2c_access == 0) return 0;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) return 0;
	}
	// Trigger temperature measurement.
	i2c_access = DPS310_write_register(dps310_i2c_address, DPS310_REG_TMP_CFG, 0x80); // External sensor, rate=1meas/s, no oversampling.
	if (i2c_access == 0) return 0;
	dps310_ctx.kT = 524288;
	i2c_access = DPS310_write_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, 0x02);
	if (i2c_access == 0) return 0;
	// Wait for temperature to be ready.
	loop_count = 0;
	while ((read_byte & (0b1 << 5)) == 0) { // Wait for TMP_RDY='1'.
		i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (i2c_access == 0) return 0;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) return 0;
	}
	// Read temperature.
	unsigned int tmp_raw = 0;
	// B2.
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_TMP_B2, &read_byte);
	if (i2c_access == 0) return 0;
	tmp_raw |= (read_byte << 16);
	// B1.
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_TMP_B1, &read_byte);
	if (i2c_access == 0) return 0;
	tmp_raw |= (read_byte << 8);
	// B0.
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_TMP_B0, &read_byte);
	if (i2c_access == 0) return 0;
	tmp_raw |= read_byte;
	dps310_ctx.tmp_raw = MATH_two_complement(tmp_raw, 23);

	return 1;
}

/* PERFORM PRESSURE MEASUREMENT.
 * @param dps310_i2c_address:	Sensor address.
 * @return:						1 in case of success, 0 in case of failure.
 */
static unsigned char DPS310_compute_raw_pressure(unsigned char dps310_i2c_address) {
	// Wait for sensor to be ready.
	unsigned char read_byte = 0;
	unsigned char i2c_access = 0;
	unsigned int loop_count = 0;
	while ((read_byte & (0b1 << 6)) == 0) { // Wait for SENSOR_RDY='1'.
		i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (i2c_access == 0) return 0;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) return 0;
	}
	// Trigger pressure measurement.
	i2c_access = DPS310_write_register(dps310_i2c_address, DPS310_REG_PRS_CFG, 0x01); // Rate=1meas/s, no oversampling.
	if (i2c_access == 0) return 0;
	dps310_ctx.kP = 1572864;
	i2c_access = DPS310_write_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, 0x01);
	if (i2c_access == 0) return 0;
	// Wait for pressure to be ready.
	read_byte = 0;
	loop_count = 0;
	while ((read_byte & (0b1 << 4)) == 0) { // Wait for PRS_RDY='1'.
		i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_MEAS_CFG, &read_byte);
		if (i2c_access == 0) return 0;
		loop_count++;
		if (loop_count > DPS310_TIMEOUT_COUNT) return 0;
	}
	// Read pressure.
	unsigned int prs_raw = 0;
	// B2.
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_PRS_B2, &read_byte);
	if (i2c_access == 0) return 0;
	prs_raw |= (read_byte << 16);
	// B1.
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_PRS_B1, &read_byte);
	if (i2c_access == 0) return 0;
	prs_raw |= (read_byte << 8);
	// B0.
	i2c_access = DPS310_read_register(dps310_i2c_address, DPS310_REG_PRS_B0, &read_byte);
	if (i2c_access == 0) return 0;
	prs_raw |= read_byte;
	dps310_ctx.prs_raw = MATH_two_complement(prs_raw, 23);

	return 1;
}

/*** DPS310 functions ***/

/* INIT DPS310 SENSOR.
 * @param:	None.
 * @return:	None.
 */
void DPS310_init(void) {
	// Sampling factors.
	dps310_ctx.kT = 0;
	dps310_ctx.kP = 0;
	// Measurements.
	dps310_ctx.tmp_raw = DPS310_RAW_ERROR_VALUE;
	dps310_ctx.prs_raw = DPS310_RAW_ERROR_VALUE;
	// Calibration coefficients.
	dps310_ctx.coef_c0 = 0;
	dps310_ctx.coef_c1 = 0;
	dps310_ctx.coef_c00 = 0;
	dps310_ctx.coef_c10 = 0;
	dps310_ctx.coef_c01 = 0;
	dps310_ctx.coef_c11 = 0;
	dps310_ctx.coef_c20 = 0;
	dps310_ctx.coef_c21 = 0;
	dps310_ctx.coef_c30 = 0;
}

/* PERFORM PRESSURE AND TEMPERATURE MEASUREMENT.
 * @param dps310_i2c_address:	Sensor address.
 * @return:						1 in case of success, 0 in case of failure.
 */
void DPS310_perform_measurements(unsigned char dps310_i2c_address) {
	// Compute raw results.
	unsigned char i2c_access = DPS310_compute_raw_temperature(dps310_i2c_address);
	if (i2c_access == 0) return;
	i2c_access = DPS310_compute_raw_pressure(dps310_i2c_address);
	if (i2c_access == 0) return;
	// Read calibration coefficients.
	i2c_access = DPS310_read_calibration_coefficients(dps310_i2c_address);
	if (i2c_access == 0) return;
}

/* READ PRESSURE FROM DPS310 SENSOR.
 * @param pressure_pa:	Pointer to integer that will contain pressure result (Pa).
 * @return:				None.
 */
void DPS310_get_pressure(unsigned int* pressure_pa) {
	// Check sensor result.
	if (dps310_ctx.prs_raw == DPS310_RAW_ERROR_VALUE) {
		// Return error value.
		(*pressure_pa) = DPS310_PRESSURE_ERROR_VALUE;
	}
	else {
		// Read coefficients.
		signed int c00 = MATH_two_complement(dps310_ctx.coef_c00, 19);
		signed int c01 = MATH_two_complement(dps310_ctx.coef_c01, 15);
		signed int c10 = MATH_two_complement(dps310_ctx.coef_c10, 19);
		signed int c11 = MATH_two_complement(dps310_ctx.coef_c11, 15);
		signed int c20 = MATH_two_complement(dps310_ctx.coef_c20, 15);
		signed int c21 = MATH_two_complement(dps310_ctx.coef_c21, 15);
		signed int c30 = MATH_two_complement(dps310_ctx.coef_c30, 15);
		// Compute pressure in Pa.
		signed long long psr_temp = c20 + (dps310_ctx.prs_raw * c30) / dps310_ctx.kP;
		psr_temp = c10 + (dps310_ctx.prs_raw * psr_temp) / dps310_ctx.kP;
		psr_temp = c00 + (dps310_ctx.prs_raw * psr_temp) / dps310_ctx.kP;
		psr_temp += (dps310_ctx.tmp_raw * c01) / dps310_ctx.kT;
		signed long long last_term = c11 + (dps310_ctx.prs_raw * c21) / dps310_ctx.kP;
		last_term = (dps310_ctx.prs_raw * last_term) / dps310_ctx.kP;
		last_term = (dps310_ctx.tmp_raw * last_term) / dps310_ctx.kT;
		psr_temp += last_term;
		(*pressure_pa) = (unsigned int) psr_temp;
	}
}

/* READ TEMPERATURE FROM DPS310 SENSOR.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (degrees).
 * @return:						None.
 */
void DPS310_get_temperature(signed char* temperature_degrees) {
	// Check sensor result.
	if (dps310_ctx.tmp_raw == DPS310_RAW_ERROR_VALUE) {
		// Return error value.
		(*temperature_degrees) = DPS310_TEMPERATURE_ERROR_VALUE;
	}
	else {
		// Compute temperature in degrees.
		signed int c0 = MATH_two_complement(dps310_ctx.coef_c0, 11);
		signed int c1 = MATH_two_complement(dps310_ctx.coef_c1, 11);
		signed long long tmp_temp = (c0 / 2) + (c1 * dps310_ctx.tmp_raw) / dps310_ctx.kT;
		(*temperature_degrees) = (unsigned char) tmp_temp;
	}
}
