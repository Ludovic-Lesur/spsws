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
#include "types.h"

/*** DPS310 local macros ***/

#define DPS310_SUB_DELAY_MS					10
#define DPS310_TIMEOUT_MS					2000
#define DPS310_SAMPLING_FACTOR_KT			524288
#define DPS310_SAMPLING_FACTOR_KP			1572864
#define DPS310_NUMBER_OF_COEF_REGISTERS		18

/*** DPS310 local structures ***/

typedef struct {
	// Measurements.
	int32_t tmp_raw;
	int32_t prs_raw;
	// Calibration coefficients.
	int32_t coef_c0;
	int32_t coef_c1;
	int32_t coef_c00;
	int32_t coef_c10;
	int32_t coef_c01;
	int32_t coef_c11;
	int32_t coef_c20;
	int32_t coef_c21;
	int32_t coef_c30;
	uint8_t coef_ready;
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
static DPS310_status_t _DPS310_write_register(uint8_t i2c_address, uint8_t register_address, uint8_t value) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint8_t register_write_command[2] = {register_address, value};
	// I2C transfer.
	i2c1_status = I2C1_write(i2c_address, register_write_command, 2, 1);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
errors:
	return status;
}

/* READ A REGISTER OF DPS310.
 * @param i2c_address:		Sensor address
 * @param register_address:	Address of the register to read.
 * @param value:			Pointer to 8-bits value that will contain the current value of the selected register.
 * @return status			Function execution status.
 */
static DPS310_status_t _DPS310_read_register(uint8_t i2c_address, uint8_t register_address, uint8_t* value) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	uint8_t local_addr = register_address;
	// I2C transfer.
	i2c1_status = I2C1_write(i2c_address, &local_addr, 1, 1);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(i2c_address, value, 1);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
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
static DPS310_status_t _DPS310_wait_flag(uint8_t i2c_address, uint8_t register_address, uint8_t bit_index, DPS310_status_t timeout_error) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint8_t reg_value = 0;
	uint32_t loop_count_ms = 0;
	// Read register.
	status = _DPS310_read_register(i2c_address, register_address, &reg_value);
	if (status != DPS310_SUCCESS) goto errors;
	// Wait for flag to be set.
	while ((reg_value & (0b1 << bit_index)) == 0) {
		// Low power delay.
		lptim1_status = LPTIM1_delay_milliseconds(DPS310_SUB_DELAY_MS, 1);
		LPTIM1_status_check(DPS310_ERROR_BASE_LPTIM);
		// Exit if timeout.
		loop_count_ms += DPS310_SUB_DELAY_MS;
		if (loop_count_ms > DPS310_TIMEOUT_MS) {
			status = timeout_error;
			goto errors;
		}
		// Read register.
		status = _DPS310_read_register(i2c_address, register_address, &reg_value);
		if (status != DPS310_SUCCESS) goto errors;
	}
errors:
	return status;
}

/* READ ALL SENSOR CALIBRATION COEFFICIENTS.
 * @param i2c_address:	Sensor address.
 * @return status		Function execution status.
 */
static DPS310_status_t _DPS310_read_calibration_coefficients(uint8_t i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	I2C_status_t i2c1_status = I2C_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	uint8_t local_addr = DPS310_REG_COEF_C0B;
	uint8_t idx = 0;
	uint8_t coef_registers[DPS310_NUMBER_OF_COEF_REGISTERS];
	// Reset all coefficients.
	for (idx=0 ; idx<DPS310_NUMBER_OF_COEF_REGISTERS ; idx++) coef_registers[idx] = 0;
	uint32_t c0 = 0;
	uint32_t c1 = 0;
	uint32_t c00 = 0;
	uint32_t c10 = 0;
	uint32_t c01 = 0;
	uint32_t c11 = 0;
	uint32_t c20 = 0;
	uint32_t c21 = 0;
	uint32_t c30 = 0;
	// Wait for coefficients to be ready for reading.
	status = _DPS310_wait_flag(i2c_address, DPS310_REG_MEAS_CFG, 7, DPS310_ERROR_COEFFICIENTS_TIMEOUT);
	if (status != DPS310_SUCCESS) goto errors;
	// Read all coefficients with auto-increment method.
	i2c1_status = I2C1_write(i2c_address, &local_addr, 1, 1);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
	i2c1_status = I2C1_read(i2c_address, coef_registers, DPS310_NUMBER_OF_COEF_REGISTERS);
	I2C1_status_check(DPS310_ERROR_BASE_I2C);
	if (status != DPS310_SUCCESS) goto errors;
	// Compute coefficients.
	c0 |= (coef_registers[0] << 4) | ((coef_registers[1] & 0xF0) >> 4);
	c1 |= ((coef_registers[1] & 0x0F) << 8) | (coef_registers[2]);
	c00 |= (coef_registers[3] << 12) | (coef_registers[4] << 4) | ((coef_registers[5] & 0xF0) >> 4);
	c10 |= ((coef_registers[5] & 0x0F) << 16) | (coef_registers[6] << 8) | (coef_registers[7]);
	c01 |= (coef_registers[8] << 8) | (coef_registers[9]);
	c11 |= (coef_registers[10] << 8) | (coef_registers[11]);
	c20 |= (coef_registers[12] << 8) | (coef_registers[13]);
	c21 |= (coef_registers[14] << 8) | (coef_registers[15]);
	c30 |= (coef_registers[16] << 8) | (coef_registers[17]);
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
	// Set flag.
	dps310_ctx.coef_ready = 1;
errors:
	return status;
}

/* PERFORM TEMPERATURE MEASUREMENT.
 * @param i2c_address:	Sensor address.
 * @return status		Function execution status.
 */
static DPS310_status_t _DPS310_compute_raw_temperature(uint8_t i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	uint8_t read_byte = 0;
	uint32_t tmp_raw = 0;
	// Wait for sensor to be ready.
	status = _DPS310_wait_flag(i2c_address, DPS310_REG_MEAS_CFG, 6, DPS310_ERROR_SENSOR_TIMEOUT);
	if (status != DPS310_SUCCESS) goto errors;
	// Trigger temperature measurement.
	status = _DPS310_write_register(i2c_address, DPS310_REG_TMP_CFG, 0x80); // External sensor, rate=1meas/s, no oversampling.
	if (status != DPS310_SUCCESS) goto errors;
	status = _DPS310_write_register(i2c_address, DPS310_REG_MEAS_CFG, 0x02);
	if (status != DPS310_SUCCESS) goto errors;
	// Wait for temperature to be ready.
	status = _DPS310_wait_flag(i2c_address, DPS310_REG_MEAS_CFG, 5, DPS310_ERROR_TEMPERATURE_TIMEOUT);
	if (status != DPS310_SUCCESS) goto errors;
	// Read temperature.
	// B2.
	status = _DPS310_read_register(i2c_address, DPS310_REG_TMP_B2, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	tmp_raw |= (read_byte << 16);
	// B1.
	status = _DPS310_read_register(i2c_address, DPS310_REG_TMP_B1, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	tmp_raw |= (read_byte << 8);
	// B0.
	status = _DPS310_read_register(i2c_address, DPS310_REG_TMP_B0, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	tmp_raw |= read_byte;
	// Compute two complement.
	math_status = MATH_two_complement(tmp_raw, 23, &dps310_ctx.tmp_raw);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
errors:
	return status;
}

/* PERFORM PRESSURE MEASUREMENT.
 * @param i2c_address:	Sensor address.
 * @return:				Function execution status.
 */
static DPS310_status_t _DPS310_compute_raw_pressure(uint8_t i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	uint8_t read_byte = 0;
	uint32_t prs_raw = 0;
	// Wait for sensor to be ready.
	status = _DPS310_wait_flag(i2c_address, DPS310_REG_MEAS_CFG, 6, DPS310_ERROR_SENSOR_TIMEOUT);
	if (status != DPS310_SUCCESS) goto errors;
	// Trigger pressure measurement.
	status = _DPS310_write_register(i2c_address, DPS310_REG_PRS_CFG, 0x01); // Rate=1meas/s, no oversampling.
	if (status != DPS310_SUCCESS) goto errors;
	status = _DPS310_write_register(i2c_address, DPS310_REG_MEAS_CFG, 0x01);
	if (status != DPS310_SUCCESS) goto errors;
	// Wait for pressure to be ready.
	status = _DPS310_wait_flag(i2c_address, DPS310_REG_MEAS_CFG, 4, DPS310_ERROR_PRESSURE_TIMEOUT);
	if (status != DPS310_SUCCESS) goto errors;
	// Read pressure.
	// B2.
	status = _DPS310_read_register(i2c_address, DPS310_REG_PRS_B2, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	prs_raw |= (read_byte << 16);
	// B1.
	status = _DPS310_read_register(i2c_address, DPS310_REG_PRS_B1, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	prs_raw |= (read_byte << 8);
	// B0.
	status = _DPS310_read_register(i2c_address, DPS310_REG_PRS_B0, &read_byte);
	if (status != DPS310_SUCCESS) goto errors;
	prs_raw |= read_byte;
	// Compute two complement.
	math_status = MATH_two_complement(prs_raw, 23, &dps310_ctx.prs_raw);
	MATH_status_check(DPS310_ERROR_BASE_MATH);
errors:
	return status;
}

/*** DPS310 functions ***/

/* INIT DPS310 MODULE.
 * @param:	None.
 * @return:	None.
 */
void DPS310_init(void) {
	// Init context.
	dps310_ctx.coef_ready = 0;
}

/* PERFORM PRESSURE AND TEMPERATURE MEASUREMENT.
 * @param i2c_address:	Sensor address.
 * @return status :		Function execution status.
 */
DPS310_status_t DPS310_perform_measurements(uint8_t i2c_address) {
	// Local variables.
	DPS310_status_t status = DPS310_SUCCESS;
	// Reset result.
	dps310_ctx.tmp_raw = 0;
	dps310_ctx.prs_raw = 0;
	// Compute raw results.
	status = _DPS310_compute_raw_temperature(i2c_address);
	if (status != DPS310_SUCCESS) goto errors;
	status = _DPS310_compute_raw_pressure(i2c_address);
	if (status != DPS310_SUCCESS) goto errors;
	// Read calibration coefficients if needed.
	if (dps310_ctx.coef_ready == 0) {
		status = _DPS310_read_calibration_coefficients(i2c_address);
	}
errors:
	return status;
}

/* READ PRESSURE FROM DPS310 SENSOR.
 * @param pressure_pa:	Pointer to 32-bits value that will contain pressure result (Pa).
 * @return:				None.
 */
void DPS310_get_pressure(uint32_t* pressure_pa) {
	// Local variables.
	int64_t psr_temp = 0;
	int64_t last_term = 0;
	// Compute pressure in Pa.
	psr_temp = dps310_ctx.coef_c20 + (dps310_ctx.prs_raw * dps310_ctx.coef_c30) / DPS310_SAMPLING_FACTOR_KP;
	psr_temp = dps310_ctx.coef_c10 + (dps310_ctx.prs_raw * psr_temp) / DPS310_SAMPLING_FACTOR_KP;
	psr_temp = dps310_ctx.coef_c00 + (dps310_ctx.prs_raw * psr_temp) / DPS310_SAMPLING_FACTOR_KP;
	psr_temp += (dps310_ctx.tmp_raw * dps310_ctx.coef_c01) / DPS310_SAMPLING_FACTOR_KT;
	last_term = dps310_ctx.coef_c11 + (dps310_ctx.prs_raw * dps310_ctx.coef_c21) / DPS310_SAMPLING_FACTOR_KP;
	last_term = (dps310_ctx.prs_raw * last_term) / DPS310_SAMPLING_FACTOR_KP;
	last_term = (dps310_ctx.tmp_raw * last_term) / DPS310_SAMPLING_FACTOR_KT;
	psr_temp += last_term;
	(*pressure_pa) = (uint32_t) psr_temp;
}

/* READ TEMPERATURE FROM DPS310 SENSOR.
 * @param temperature_degrees:	Pointer to 8-bits value that will contain temperature result (degrees).
 * @return:						None.
 */
void DPS310_get_temperature(int8_t* temperature_degrees) {
	// Local variables.
	int64_t tmp_temp = 0;
	// Compute temperature in degrees.
	tmp_temp = (dps310_ctx.coef_c0 / 2) + (dps310_ctx.coef_c1 * dps310_ctx.tmp_raw) / DPS310_SAMPLING_FACTOR_KT;
	(*temperature_degrees) = (uint8_t) tmp_temp;
}
