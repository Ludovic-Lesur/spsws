/*
 * max11136.c
 *
 *  Created on: 17 nov. 2018
 *      Author: Ludo
 */

#include "max11136.h"

#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "max11136_reg.h"
#include "mode.h"
#include "spi.h"
#include "types.h"

/*** MAX11136 local macros ***/

#define MAX11136_NUMBER_OF_CHANNELS			8
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define MAX11136_CHANNEL_WIND_DIRECTION		0
#endif
#ifdef HW1_0
#define MAX11136_CHANNEL_VSRC				4
#define MAX11136_CHANNEL_VCAP				5
#define MAX11136_CHANNEL_LDR				6
#define MAX11136_CHANNEL_REF191				7
#endif
#ifdef HW2_0
#define MAX11136_CHANNEL_LDR				1
#define MAX11136_CHANNEL_REF191				5
#define MAX11136_CHANNEL_VSRC				6
#define MAX11136_CHANNEL_VCAP				7
#endif

#define MAX11136_REF191_VOLTAGE_MV			2048
#define MAX11136_FULL_SCALE					4095

#define MAX11136_VSRC_DIVIDER_RATIO_NUM		269
#define MAX11136_VSRC_DIVIDER_RATIO_DEN		34
#define MAX11136_VCAP_DIVIDER_RATIO_NUM		269
#define MAX11136_VCAP_DIVIDER_RATIO_DEN		34

#define MAX11136_CONVERSION_LOOPS			3

#define MAX11136_SUB_DELAY_MS				100
#define MAX11136_TIMEOUT_MS					2000

/*** MAX11136 local structures ***/

/*******************************************************************/
typedef struct {
	uint16_t data_12bits[MAX11136_NUMBER_OF_CHANNELS];
	uint32_t data[MAX11136_DATA_INDEX_LAST];
	uint8_t status; // Bit 'i' indicates if a result was successfully retrieved for channel 'i'.
} MAX11136_context_t;

/*** MAX11136 local global variables ***/

static MAX11136_context_t max11136_ctx;

/*** MAX11136 local functions ***/

/*******************************************************************/
static MAX11136_status_t __attribute__((optimize("-O0"))) _MAX11136_write_register(uint8_t register_address, uint16_t value) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
#ifdef HW1_0
	SPI_status_t spi1_status = SPI_SUCCESS;
#endif
#ifdef HW2_0
	SPI_status_t spi2_status = SPI_SUCCESS;
#endif
	uint16_t spi_command = 0;
	uint16_t spi_reply = 0;
	// Check parameters.
	if (register_address >= MAX11136_REG_LAST) {
		status = MAX11136_ERROR_REGISTER_ADDRESS;
		goto errors;
	}
	// Build SPI command.
	if (register_address == MAX11136_REG_ADC_MODE_CONTROL) {
		// Data is 15-bits length.
		spi_command |= value & 0x00007FFF;
	}
	else {
		// Data is 11-bits length.
		spi_command |= (register_address & 0x0000001F) << 11;
		spi_command |= (value & 0x000007FF);
	}
	// Send command.
	GPIO_write(&GPIO_MAX11136_CS, 0);
#ifdef HW1_0
	spi1_status = SPI1_write_read(&spi_command, &spi_reply, 1);
	SPI1_exit_error(MAX11136_ERROR_BASE_SPI1);
#endif
#ifdef HW2_0
	spi2_status = SPI2_write_read(&spi_command, &spi_reply, 1);
	SPI2_exit_error(MAX11136_ERROR_BASE_SPI2);
#endif
	GPIO_write(&GPIO_MAX11136_CS, 1);
errors:
	return status;
}

/*******************************************************************/
static MAX11136_status_t __attribute__((optimize("-O0"))) _MAX11136_convert_all_channels(void) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
#ifdef HW1_0
	SPI_status_t spi1_status = SPI_SUCCESS;
#endif
#ifdef HW2_0
	SPI_status_t spi2_status = SPI_SUCCESS;
#endif
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint16_t spi_command = 0;
	uint16_t max11136_dout = 0;
	uint8_t channel = 0;
	uint8_t channel_idx = 0;
	uint32_t loop_count_ms = 0;
	// Configure ADC.
	// Single-ended unipolar (already done at POR).
	// Enable averaging: AVGON='1' and NAVG='00' (4 conversions).
	status = _MAX11136_write_register(MAX11136_REG_ADC_CONFIG, 0x0200);
	if (status != MAX11136_SUCCESS) goto errors;
	// Scan mode = standard internal: SCAN='0011'.
	// Perform conversion from AIN0 to AIN7: CHSEL='0111'.
	// Reset the FIFO: RESET='01'.
	// Auto shutdown: PM='01'.
	// Start conversion: SWCNV='1'.
	status = _MAX11136_write_register(MAX11136_REG_ADC_MODE_CONTROL, 0x1BAA);
	if (status != MAX11136_SUCCESS) goto errors;
	// Wait for EOC to be pulled low.
	while (GPIO_read(&GPIO_MAX11136_EOC) != 0) {
		// Low power delay.
		lptim1_status = LPTIM1_delay_milliseconds(MAX11136_SUB_DELAY_MS, LPTIM_DELAY_MODE_STOP);
		LPTIM1_exit_error(MAX11136_ERROR_BASE_LPTIM1);
		// Exit if timeout.
		loop_count_ms += MAX11136_SUB_DELAY_MS;
		if (loop_count_ms > MAX11136_TIMEOUT_MS) {
			status = MAX11136_ERROR_TIMEOUT;
			goto errors;
		}
	}
	// Wait for all channels to be read or timeout (TBD).
	for (channel_idx=0 ; channel_idx<MAX11136_NUMBER_OF_CHANNELS ; channel_idx++) {
		// Get data from SPI.
		GPIO_write(&GPIO_MAX11136_CS, 0); // Falling edge on CS pin.
#ifdef HW1_0
		spi1_status = SPI1_write_read(&spi_command, &max11136_dout, 1);
		SPI1_exit_error(MAX11136_ERROR_BASE_SPI1);
#endif
#ifdef HW2_0
		spi2_status = SPI2_write_read(&spi_command, &max11136_dout, 1);
		SPI2_exit_error(MAX11136_ERROR_BASE_SPI2);
#endif
		GPIO_write(&GPIO_MAX11136_CS, 1);
		// Parse result = 'CH4 CH2 CH1 CH0 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0'.
		channel = (max11136_dout & 0xF000) >> 12;
		if (channel < MAX11136_NUMBER_OF_CHANNELS) {
			// Fill data.
			max11136_ctx.data_12bits[channel] = max11136_dout & 0x0FFF;
			// Update status.
			max11136_ctx.status |= (0b1 << channel);
		}
	}
errors:
	return status;
}

/*** MAX11136 functions ***/

/*******************************************************************/
void MAX11136_init(void) {
	// Local variables.
	uint8_t idx = 0;
	// Init context.
	max11136_ctx.status = 0;
	for (idx=0 ; idx<MAX11136_DATA_INDEX_LAST ; idx++) max11136_ctx.data[idx] = 0;
	for (idx=0 ; idx<MAX11136_NUMBER_OF_CHANNELS ; idx++) max11136_ctx.data_12bits[idx] = 0;
	// Init SPI.
#ifdef HW1_0
	SPI1_init();
#endif
#ifdef HW2_0
	SPI2_init();
#endif
	// Configure chip select pin.
	GPIO_configure(&GPIO_MAX11136_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_write(&GPIO_MAX11136_CS, 1);
	// Configure EOC GPIO.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_UP);
}

/*******************************************************************/
void MAX11136_de_init(void) {
	// Release chip select pin.
	GPIO_write(&GPIO_MAX11136_CS, 0);
	// Release EOC GPIO.
	GPIO_configure(&GPIO_MAX11136_EOC, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Release SPI.
#ifdef HW1_0
	SPI1_de_init();
#endif
#ifdef HW2_0
	SPI2_de_init();
#endif
}

/*******************************************************************/
MAX11136_status_t MAX11136_perform_measurements(void) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
	uint8_t idx = 0;
	// Reset results.
	max11136_ctx.status = 0;
	for (idx=0 ; idx<MAX11136_DATA_INDEX_LAST ; idx++) max11136_ctx.data[idx] = 0;
	for (idx=0 ; idx<MAX11136_NUMBER_OF_CHANNELS ; idx++) max11136_ctx.data_12bits[idx] = 0;
	// Perform conversion until all channels are successfully retrieved or maximum number of loops is reached.
	idx = 0;
	while (max11136_ctx.status != 0xFF) {
		status = _MAX11136_convert_all_channels();
		if (status != MAX11136_SUCCESS) goto errors;
		idx++;
		if (idx > MAX11136_CONVERSION_LOOPS) {
			status = MAX11136_ERROR_CONVERSION;
			goto errors;
		}
	}
	// Perform units conversion.
	// VSRC.
	max11136_ctx.data[MAX11136_DATA_INDEX_VSRC_MV] = (max11136_ctx.data_12bits[MAX11136_CHANNEL_VSRC] * MAX11136_REF191_VOLTAGE_MV * MAX11136_VSRC_DIVIDER_RATIO_NUM) ;
	max11136_ctx.data[MAX11136_DATA_INDEX_VSRC_MV] /= (max11136_ctx.data_12bits[MAX11136_CHANNEL_REF191] * MAX11136_VSRC_DIVIDER_RATIO_DEN);
	// VCAP.
	max11136_ctx.data[MAX11136_DATA_INDEX_VCAP_MV] = (max11136_ctx.data_12bits[MAX11136_CHANNEL_VCAP] * MAX11136_REF191_VOLTAGE_MV * MAX11136_VCAP_DIVIDER_RATIO_NUM);
	max11136_ctx.data[MAX11136_DATA_INDEX_VCAP_MV] /= (max11136_ctx.data_12bits[MAX11136_CHANNEL_REF191] * MAX11136_VCAP_DIVIDER_RATIO_DEN);
	// LDR.
	max11136_ctx.data[MAX11136_DATA_INDEX_LDR_PERCENT] = (max11136_ctx.data_12bits[MAX11136_CHANNEL_LDR] * 100) / (MAX11136_FULL_SCALE);
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	// Wind direction.
	max11136_ctx.data[MAX11136_DATA_INDEX_WIND_DIRECTION_RATIO] = (max11136_ctx.data_12bits[MAX11136_CHANNEL_WIND_DIRECTION] * 1000) / (MAX11136_FULL_SCALE);
#endif
errors:
	return status;
}

/*******************************************************************/
MAX11136_status_t MAX11136_get_data(MAX11136_data_index_t data_idx, uint32_t* data) {
	// Local variables.
	MAX11136_status_t status = MAX11136_SUCCESS;
	// Check parameters.
	if (data == NULL) {
		status = MAX11136_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (data_idx >= MAX11136_DATA_INDEX_LAST) {
		status = MAX11136_ERROR_DATA_INDEX;
		goto errors;
	}
	(*data) = max11136_ctx.data[data_idx];
errors:
	return status;
}
