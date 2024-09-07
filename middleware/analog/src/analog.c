/*
 * analog.c
 *
 *  Created on: 12 aug. 2024
 *      Author: Ludo
 */

#include "analog.h"

#include "adc.h"
#include "error.h"
#include "max111xx.h"
#include "mode.h"
#include "types.h"

/*** ANALOG local macros ***/

#define ANALOG_VMCU_MV_DEFAULT					3300
#define ANALOG_TMCU_DEGREES_DEFAULT				25

#define ANALOG_REF191_VOLTAGE_MV				2048

#define ANALOG_VSRC_DIVIDER_RATIO_NUM			269
#define ANALOG_VSRC_DIVIDER_RATIO_DEN			34
#define ANALOG_VCAP_DIVIDER_RATIO_NUM			269
#define ANALOG_VCAP_DIVIDER_RATIO_DEN			34

#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
#define ANALOG_MAX11136_CHANNEL_WIND_DIRECTION	MAX111XX_CHANNEL_AIN0
#endif
#ifdef HW1_0
#define ANALOG_MAX11136_CHANNEL_VPV				MAX111XX_CHANNEL_AIN4
#define ANALOG_MAX11136_CHANNEL_VCAP			MAX111XX_CHANNEL_AIN5
#define ANALOG_MAX11136_CHANNEL_LDR				MAX111XX_CHANNEL_AIN6
#define ANALOG_MAX11136_CHANNEL_REF191			MAX111XX_CHANNEL_AIN7
#endif
#ifdef HW2_0
#define ANALOG_MAX11136_CHANNEL_LDR				MAX111XX_CHANNEL_AIN1
#define ANALOG_MAX11136_CHANNEL_REF191			MAX111XX_CHANNEL_AIN5
#define ANALOG_MAX11136_CHANNEL_VPV				MAX111XX_CHANNEL_AIN6
#define ANALOG_MAX11136_CHANNEL_VCAP			MAX111XX_CHANNEL_AIN7
#endif

#define ANALOG_ERROR_VALUE						0xFFFF

/*** ANALOG local structures ***/

/*******************************************************************/
typedef struct {
	int32_t vmcu_mv;
	int32_t ref191_data_12bits;
} ANALOG_context_t;

/*** ANALOG local global variables ***/

static ANALOG_context_t analog_ctx = {.ref191_data_12bits = ANALOG_ERROR_VALUE};

/*** ANALOG local functions ***/

/*******************************************************************/
static ANALOG_status_t _ANALOG_convert_max11136_channel(MAX111XX_channel_t channel, int32_t* adc_data_12bits) {
	// Local variables.
	ANALOG_status_t status = ANALOG_SUCCESS;
	MAX111XX_status_t max111xx_status = MAX111XX_SUCCESS;
	// Check current value.
	if (analog_ctx.ref191_data_12bits == ANALOG_ERROR_VALUE) {
		// Update calibration value.
		max111xx_status = MAX111XX_convert_channel(ANALOG_MAX11136_CHANNEL_REF191, &(analog_ctx.ref191_data_12bits));
		MAX111XX_exit_error(ANALOG_ERROR_BASE_MAX11136);
	}
	max111xx_status = MAX111XX_convert_channel(channel, adc_data_12bits);
	MAX111XX_exit_error(ANALOG_ERROR_BASE_MAX11136);
errors:
	return status;
}

/*** ANALOG functions ***/

/*******************************************************************/
ANALOG_status_t ANALOG_init(void) {
	// Local variables.
	ANALOG_status_t status = ANALOG_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	MAX111XX_status_t max111xx_status = MAX111XX_SUCCESS;
	// Init context.
	analog_ctx.vmcu_mv = ANALOG_VMCU_MV_DEFAULT;
	analog_ctx.ref191_data_12bits = ANALOG_ERROR_VALUE;
	// Init internal ADC.
	adc_status = ADC_init(NULL);
	ADC_exit_error(ANALOG_ERROR_BASE_ADC);
	// Init external ADC.
	max111xx_status = MAX111XX_init();
	MAX111XX_exit_error(ANALOG_ERROR_BASE_MAX11136);
errors:
	return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_de_init(void) {
	// Local variables.
	ANALOG_status_t status = ANALOG_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	MAX111XX_status_t max111xx_status = MAX111XX_SUCCESS;
	// Erase calibration value.
	analog_ctx.ref191_data_12bits = ANALOG_ERROR_VALUE;
	// Release internal ADC.
	adc_status = ADC_de_init();
	ADC_exit_error(ANALOG_ERROR_BASE_ADC);
	// Release external ADC.
	max111xx_status = MAX111XX_de_init();
	MAX111XX_exit_error(ANALOG_ERROR_BASE_MAX11136);
errors:
	return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_convert_channel(ANALOG_channel_t channel, int32_t* analog_data) {
	// Local variables.
	ANALOG_status_t status = ANALOG_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	int32_t adc_data_12bits = 0;
	// Check parameter.
	if (analog_data == NULL) {
		status = ANALOG_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check channel.
	switch (channel) {
	case ANALOG_CHANNEL_VMCU_MV:
		// MCU voltage.
		adc_status = ADC_convert_channel(ADC_CHANNEL_VREFINT, &adc_data_12bits);
		ADC_exit_error(ANALOG_ERROR_BASE_ADC);
		// Convert to mV.
		adc_status = ADC_compute_vmcu(adc_data_12bits, ADC_get_vrefint_voltage_mv(), analog_data);
		ADC_exit_error(ANALOG_ERROR_BASE_ADC);
		// Update local value for temperature computation.
		analog_ctx.vmcu_mv = (*analog_data);
		break;
	case ANALOG_CHANNEL_TMCU_DEGREES:
		// MCU temperature.
		adc_status = ADC_convert_channel(ADC_CHANNEL_TEMPERATURE_SENSOR, &adc_data_12bits);
		ADC_exit_error(ANALOG_ERROR_BASE_ADC);
		// Convert to degrees.
		adc_status = ADC_compute_tmcu(analog_ctx.vmcu_mv, adc_data_12bits, analog_data);
		ADC_exit_error(ANALOG_ERROR_BASE_ADC);
		break;
	case ANALOG_CHANNEL_VPV_MV:
		// Solar cell voltage.
		status = _ANALOG_convert_max11136_channel(ANALOG_MAX11136_CHANNEL_VPV, &adc_data_12bits);
		if (status != ANALOG_SUCCESS) goto errors;
		// Convert to mV.
		(*analog_data) = ((int32_t) adc_data_12bits * ANALOG_REF191_VOLTAGE_MV * ANALOG_VSRC_DIVIDER_RATIO_NUM) / ((int32_t) analog_ctx.ref191_data_12bits * ANALOG_VSRC_DIVIDER_RATIO_DEN);
		break;
	case ANALOG_CHANNEL_VCAP_MV:
		// Supercap voltage.
		status = _ANALOG_convert_max11136_channel(ANALOG_MAX11136_CHANNEL_VCAP, &adc_data_12bits);
		if (status != ANALOG_SUCCESS) goto errors;
		// Convert to mV.
		(*analog_data) = ((int32_t) adc_data_12bits * ANALOG_REF191_VOLTAGE_MV * ANALOG_VCAP_DIVIDER_RATIO_NUM) / ((int32_t) analog_ctx.ref191_data_12bits * ANALOG_VCAP_DIVIDER_RATIO_DEN);
		break;
	case ANALOG_CHANNEL_LDR_PERCENT:
		// Light sensor.
		status = _ANALOG_convert_max11136_channel(ANALOG_MAX11136_CHANNEL_LDR, &adc_data_12bits);
		if (status != ANALOG_SUCCESS) goto errors;
		// Convert to percent.
		(*analog_data) = (adc_data_12bits * 100) / (MAX111XX_FULL_SCALE);
		break;
#ifdef WIND_VANE_ARGENT_DATA_SYSTEMS
	case ANALOG_CHANNEL_WIND_DIRECTION_RATIO:
		// Wind vane direction.
		status = _ANALOG_convert_max11136_channel(ANALOG_MAX11136_CHANNEL_WIND_DIRECTION, &adc_data_12bits);
		if (status != ANALOG_SUCCESS) goto errors;
		// Convert to ratio.
		(*analog_data) = (adc_data_12bits * 1000) / (MAX111XX_FULL_SCALE);
		break;
#endif
	default:
		status = ANALOG_ERROR_CHANNEL;
		goto errors;
	}
errors:
	return status;
}
