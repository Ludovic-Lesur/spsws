/*
 * dps310.c
 *
 *  Created on: 27 nov. 2018
 *      Author: Ludovic
 */

#include "dps310.h"

#include "dps310_reg.h"
#include "i2c.h"

/*** DPS310 local macros ***/

#define DPS310_I2C_ADDRESS	0x77

/*** DPS310 local structures ***/

typedef struct {
	// Sampling factors.
	signed int dps310_kT;
	signed int dps310_kP;
	// Measurements.
	signed int dps310_tmp_raw;
	signed int dps310_prs_raw;
	// Calibration coefficients.
	unsigned int dps310_coef_c0;
	unsigned int dps310_coef_c1;
	unsigned int dps310_coef_c00;
	unsigned int dps310_coef_c10;
	unsigned int dps310_coef_c01;
	unsigned int dps310_coef_c11;
	unsigned int dps310_coef_c20;
	unsigned int dps310_coef_c21;
	unsigned int dps310_coef_c30;
} DPS310_Context;

/*** DPS310 local global variables ***/

static DPS310_Context dps310_ctx;

/*** DPS310 local functions ***/

/* WRITE A REGISTER OF DPS310.
 * @param addr:		Address of the register to set.
 * @param value:	Value to write in the selected register.
 * @return:			None.
 */
void DPS310_WriteRegister(unsigned char addr, unsigned char value) {
	unsigned char register_write_command[2] = {addr, value};
	I2C_Write(DPS310_I2C_ADDRESS, register_write_command, 2);
}

/* READ A REGISTER OF DPS310.
 * @param addr:		Address of the register to read.
 * @param value:	Pointer to byte that will contain the current value of the selected register.
 * @return:			None.
 */
void DPS310_ReadRegister(unsigned char addr, unsigned char* value) {
	unsigned char local_addr = addr;
	I2C_Write(DPS310_I2C_ADDRESS, &local_addr, 1);
	I2C_Read(DPS310_I2C_ADDRESS, value, 1);
}

/* COMPUTE THE TWO'S COMPLEMENT OF A GIVEN VALUE.
 * @param value:				Value on which the two's complement has to be computed.
 * @param sign_bit_position:	Position of the sign bit.
 * @return result:				Result of computation.
 */
signed int DPS310_ComputeTwoComplement(unsigned int value, unsigned char sign_bit_position) {
	signed int result = 0;
	// Check sign bit.
	if ((value & (0b1 << sign_bit_position)) == 0) {
		// Value is positive: nothing to do.
		result = value;
	}
	else {
		// Value is negative.
		unsigned char bit_idx = 0;
		unsigned int not_value = 0;
		for (bit_idx=0 ; bit_idx<=sign_bit_position ; bit_idx++) {
			if ((value & (0b1 << bit_idx)) == 0) {
				not_value |= (0b1 << bit_idx);
			}
		}
		unsigned int absolute_value = not_value + 1;
		result = (-1) * absolute_value;
	}
	return result;
}

/* READ ALL SENSOR CALIBRATION COEFFICIENTS.
 * @param:	None.
 * @return:	None.
 */
void DPS310_ReadCalibrationCoefficients(void) {

	/* Reset all coefficients */
	dps310_ctx.dps310_coef_c0 = 0;
	dps310_ctx.dps310_coef_c1 = 0;
	dps310_ctx.dps310_coef_c00 = 0;
	dps310_ctx.dps310_coef_c10 = 0;
	dps310_ctx.dps310_coef_c01 = 0;
	dps310_ctx.dps310_coef_c10 = 0;
	dps310_ctx.dps310_coef_c11 = 0;
	dps310_ctx.dps310_coef_c20 = 0;
	dps310_ctx.dps310_coef_c21 = 0;
	dps310_ctx.dps310_coef_c30 = 0;

	/* Read all coefficients */
	unsigned char read_byte = 0;
	DPS310_ReadRegister(DPS310_REG_COEF_C0B, &read_byte);
	dps310_ctx.dps310_coef_c0 |= (read_byte << 4);
	DPS310_ReadRegister(DPS310_REG_COEF_C0A_C1B, &read_byte);
	dps310_ctx.dps310_coef_c0 |= (read_byte & 0xF0) >> 4;
	dps310_ctx.dps310_coef_c1 |= (read_byte & 0x0F) << 8;
	DPS310_ReadRegister(DPS310_REG_COEF_C1A, &read_byte);
	dps310_ctx.dps310_coef_c1 |= read_byte;
	DPS310_ReadRegister(DPS310_REG_COEF_C00C, &read_byte);
	dps310_ctx.dps310_coef_c00 |= (read_byte << 12);
	DPS310_ReadRegister(DPS310_REG_COEF_C00B, &read_byte);
	dps310_ctx.dps310_coef_c00 |= (read_byte << 4);
	DPS310_ReadRegister(DPS310_REG_COEF_C00A_C10C, &read_byte);
	dps310_ctx.dps310_coef_c00 |= (read_byte & 0xF0) >> 4;
	dps310_ctx.dps310_coef_c10 |= (read_byte & 0x0F) << 16;
	DPS310_ReadRegister(DPS310_REG_COEF_C10B, &read_byte);
	dps310_ctx.dps310_coef_c10 |= (read_byte << 8);
	DPS310_ReadRegister(DPS310_REG_COEF_C10A, &read_byte);
	dps310_ctx.dps310_coef_c10 |= read_byte;
	DPS310_ReadRegister(DPS310_REG_COEF_C01B, &read_byte);
	dps310_ctx.dps310_coef_c01 |= (read_byte << 8);
	DPS310_ReadRegister(DPS310_REG_COEF_C01A, &read_byte);
	dps310_ctx.dps310_coef_c01 |= read_byte;
	DPS310_ReadRegister(DPS310_REG_COEF_C11B, &read_byte);
	dps310_ctx.dps310_coef_c11 |= (read_byte << 8);
	DPS310_ReadRegister(DPS310_REG_COEF_C11A, &read_byte);
	dps310_ctx.dps310_coef_c11 |= read_byte;
	DPS310_ReadRegister(DPS310_REG_COEF_C20B, &read_byte);
	dps310_ctx.dps310_coef_c20 |= (read_byte << 8);
	DPS310_ReadRegister(DPS310_REG_COEF_C20A, &read_byte);
	dps310_ctx.dps310_coef_c20 |= read_byte;
	DPS310_ReadRegister(DPS310_REG_COEF_C21B, &read_byte);
	dps310_ctx.dps310_coef_c21 |= (read_byte << 8);
	DPS310_ReadRegister(DPS310_REG_COEF_C21A, &read_byte);
	dps310_ctx.dps310_coef_c21 |= read_byte;
	DPS310_ReadRegister(DPS310_REG_COEF_C30B, &read_byte);
	dps310_ctx.dps310_coef_c30 |= (read_byte << 8);
	DPS310_ReadRegister(DPS310_REG_COEF_C30A, &read_byte);
	dps310_ctx.dps310_coef_c30 |= read_byte;
}

/* PERFORM TEMPERATURE MEASUREMENT.
 * @param:	None.
 * @return:	None.
 */
void DPS310_ComputeRawTemperature(void) {

	/* Trigger temperature measurement */
	DPS310_WriteRegister(DPS310_REG_TMP_CFG, 0x22); // Rate = 4 meas/s, oversampling = 4times.
	dps310_ctx.dps310_kT = 3670016;
	DPS310_WriteRegister(DPS310_REG_MEAS_CFG, 0x02);

	/* Wait for temperature to be ready */
	unsigned char read_byte = 0;
	while ((read_byte & (0b1 << 5)) == 0) { // Wait for TMP_RDY='1'.
		DPS310_ReadRegister(DPS310_REG_MEAS_CFG, &read_byte);
	}

	/* Read temperature */
	unsigned int tmp_raw = 0;
	// B2.
	DPS310_ReadRegister(DPS310_REG_TMP_B2, &read_byte);
	tmp_raw |= (read_byte << 16);
	// B1.
	DPS310_ReadRegister(DPS310_REG_TMP_B1, &read_byte);
	tmp_raw |= (read_byte << 8);
	// B0.
	DPS310_ReadRegister(DPS310_REG_TMP_B0, &read_byte);
	tmp_raw |= read_byte;
	dps310_ctx.dps310_tmp_raw = DPS310_ComputeTwoComplement(tmp_raw, 23);
}

/* PERFORM PRESSURE MEASUREMENT.
 * @param:	None.
 * @return:	None.
 */
void DPS310_ComputeRawPressure(void) {

	/* Trigger pressure measurement */
	DPS310_WriteRegister(DPS310_REG_PRS_CFG, 0x22); // Rate = 4 meas/s, oversampling = 4times.
	dps310_ctx.dps310_kP = 3670016;
	DPS310_WriteRegister(DPS310_REG_MEAS_CFG, 0x01);

	/* Wait for pressure to be ready */
	unsigned char read_byte = 0;
	while ((read_byte & (0b1 << 4)) == 0) { // Wait for PRS_RDY='1'.
		DPS310_ReadRegister(DPS310_REG_MEAS_CFG, &read_byte);
	}

	/* Read pressure */
	unsigned int prs_raw = 0;
	// B2.
	DPS310_ReadRegister(DPS310_REG_PRS_B2, &read_byte);
	prs_raw |= (read_byte << 16);
	// B1.
	DPS310_ReadRegister(DPS310_REG_PRS_B1, &read_byte);
	prs_raw |= (read_byte << 8);
	// B0.
	DPS310_ReadRegister(DPS310_REG_PRS_B0, &read_byte);
	prs_raw |= read_byte;
	dps310_ctx.dps310_prs_raw = DPS310_ComputeTwoComplement(prs_raw, 23);
}

/*** DPS310 functions ***/

/* READ TEMPERATURE FROM DPS310 SENSOR.
 * @param temperature_degrees:	Pointer to byte that will contain temperature result (°C).
 * @return:						None.
 */
void DPS310_GetTemperature(signed char* temperature_degrees) {

	/* Compute raw temperature */
	DPS310_ComputeRawTemperature();

	/* Read calibration coefficients */
	DPS310_ReadCalibrationCoefficients();

	/* Compute temperature in °C */
	signed int c0 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c0, 11);
	signed int c1 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c1, 11);
	signed long long tmp_temp = (c0 / 2) + (c1 * dps310_ctx.dps310_tmp_raw) / dps310_ctx.dps310_kT;
	(*temperature_degrees) = (unsigned char) tmp_temp;
}

/* READ PRESSURE FROM DPS310 SENSOR.
 * @param pressure_pa:	Pointer to integer that will contain pressure result (Pa).
 * @return:				None.
 */
void DPS310_GetPressure(unsigned int* pressure_pa) {

	/* Compute raw pressure and temperature */
	DPS310_ComputeRawTemperature();
	DPS310_ComputeRawPressure();

	/* Read calibration coefficients */
	DPS310_ReadCalibrationCoefficients();

	/* Compute pressure in Pa */
	signed int c00 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c00, 19);
	signed int c01 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c01, 15);
	signed int c10 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c10, 19);
	signed int c11 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c11, 15);
	signed int c20 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c20, 15);
	signed int c21 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c21, 15);
	signed int c30 = DPS310_ComputeTwoComplement(dps310_ctx.dps310_coef_c30, 15);

	signed long long psr_temp = c20 + (dps310_ctx.dps310_prs_raw * c30) / dps310_ctx.dps310_kP;
	psr_temp = c10 + (dps310_ctx.dps310_prs_raw * psr_temp) / dps310_ctx.dps310_kP;
	psr_temp = c00 + (dps310_ctx.dps310_prs_raw * psr_temp) / dps310_ctx.dps310_kP;
	psr_temp += (dps310_ctx.dps310_tmp_raw * c01) / dps310_ctx.dps310_kT;
	signed long long last_term = c11 + (dps310_ctx.dps310_prs_raw * c21) / dps310_ctx.dps310_kP;
	last_term = (dps310_ctx.dps310_prs_raw * last_term) / dps310_ctx.dps310_kP;
	last_term = (dps310_ctx.dps310_tmp_raw * last_term) / dps310_ctx.dps310_kT;
	psr_temp += last_term;
	(*pressure_pa) = (unsigned int) psr_temp;
}
