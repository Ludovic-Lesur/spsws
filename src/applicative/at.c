/*
 * at.c
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#include "at.h"

#include "adc.h"
#include "addon_sigfox_rf_protocol_api.h"
#include "aes.h"
#include "dps310.h"
#include "error.h"
#include "i2c.h"
#include "lpuart.h"
#include "lptim.h"
#include "math.h"
#include "max11136.h"
#include "mode.h"
#include "neom8n.h"
#include "nvic.h"
#include "nvm.h"
#include "parser.h"
#include "rain.h"
#include "rf_api.h"
#include "rtc.h"
#include "sht3x.h"
#include "si1133.h"
#include "sigfox_api.h"
#include "sigfox_types.h"
#include "sky13317.h"
#include "spi.h"
#include "string.h"
#include "sx1232.h"
#include "tim.h"
#include "usart.h"
#include "wind.h"

#ifdef ATM

/*** AT local macros ***/

// Enabled commands.
#define AT_COMMANDS_SENSORS
#define AT_COMMANDS_GPS
#define AT_COMMANDS_NVM
#define AT_COMMANDS_SIGFOX
#define AT_COMMANDS_CW_RSSI
#define AT_COMMANDS_RC
#define AT_COMMANDS_TEST_MODES
// Common macros.
#define AT_COMMAND_LENGTH_MIN			2
#define AT_COMMAND_BUFFER_LENGTH		128
#define AT_RESPONSE_BUFFER_LENGTH		128
#define AT_STRING_VALUE_BUFFER_LENGTH	16
// Parameters separator.
#define AT_CHAR_SEPARATOR				','
// Responses.
#define AT_RESPONSE_END					"\r\n"
#define AT_RESPONSE_TAB					"     "
// Duration of RSSI command.
#define AT_RSSI_REPORT_PERIOD_MS		500

/*** AT callbacks declaration ***/

static void AT_print_ok(void);
static void AT_print_command_list(void);
#ifdef AT_COMMANDS_SENSORS
static void AT_adc_callback(void);
static void AT_max11136_callback(void);
static void AT_iths_callback(void);
static void AT_eths_callback(void);
static void AT_epts_callback(void);
static void AT_euvs_callback(void);
static void AT_wind_callback(void);
static void AT_rain_callback(void);
#endif
#ifdef AT_COMMANDS_GPS
static void AT_time_callback(void);
static void AT_gps_callback(void);
#endif
#ifdef AT_COMMANDS_NVM
static void AT_nvmr_callback(void);
static void AT_nvm_callback(void);
static void AT_get_id_callback(void);
static void AT_set_id_callback(void);
static void AT_get_key_callback(void);
static void AT_set_key_callback(void);
#endif
#ifdef AT_COMMANDS_SIGFOX
static void AT_so_callback(void);
static void AT_sb_callback(void);
static void AT_sf_callback(void);
#endif
#ifdef AT_COMMANDS_CW_RSSI
static void AT_cw_callback(void);
static void AT_rssi_callback(void);
#endif
#ifdef AT_COMMANDS_RC
static void AT_get_rc_callback(void);
static void AT_set_rc_callback(void);
#endif
#ifdef AT_COMMANDS_TEST_MODES
static void AT_tm_callback(void);
#endif

/*** AT local structures ***/

typedef struct {
	PARSER_mode_t mode;
	char* syntax;
	char* parameters;
	char* description;
	void (*callback)(void);
} AT_command_t;

typedef struct {
	// AT command buffer.
	volatile unsigned char command_buf[AT_COMMAND_BUFFER_LENGTH];
	volatile unsigned int command_buf_idx;
	volatile unsigned char line_end_flag;
	PARSER_context_t parser;
	char response_buf[AT_RESPONSE_BUFFER_LENGTH];
	unsigned int response_buf_idx;
	// Wind measurement flag.
	unsigned char wind_measurement_flag;
	// Sigfox RC.
	sfx_rc_t sigfox_rc;
	sfx_u32 sigfox_rc_std_config[SIGFOX_RC_STD_CONFIG_SIZE];
	unsigned char sigfox_rc_idx;
} AT_context_t;

/*** AT local global variables ***/

static const AT_command_t AT_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", "\0", "Ping command", AT_print_ok},
	{PARSER_MODE_COMMAND, "AT?", "\0", "List all available AT commands", AT_print_command_list},
#ifdef AT_COMMANDS_SENSORS
	{PARSER_MODE_COMMAND, "AT$ADC?", "\0", "Get internal ADC measurements", AT_adc_callback},
	{PARSER_MODE_COMMAND, "AT$MAX11136?", "\0", "Get external ADC measurements (MAX11136)", AT_max11136_callback},
#ifdef HW2_0
	{PARSER_MODE_COMMAND, "AT$ITHS?", "\0", "Get internal PCB temperature and humidity (SHT30)", AT_iths_callback},
#endif
	{PARSER_MODE_COMMAND, "AT$ETHS?", "\0", "Get external temperature and humidity (SHT30)", AT_eths_callback},
	{PARSER_MODE_COMMAND, "AT$EPTS?", "\0", "Get pressure and temperature (DPS310)", AT_epts_callback},
	{PARSER_MODE_COMMAND, "AT$EUVS?", "\0", "Get UV (SI1133)", AT_euvs_callback},
	{PARSER_MODE_HEADER,  "AT$WIND=", "\0", "Enable or disable wind measurements", AT_wind_callback},
	{PARSER_MODE_HEADER,  "AT$RAIN=", "\0", "Enable or disable rain detection", AT_rain_callback},
#endif
#ifdef AT_COMMANDS_GPS
	{PARSER_MODE_HEADER,  "AT$TIME=", "timeout[s]", "Get GPS time (NEOM8N)", AT_time_callback},
	{PARSER_MODE_HEADER,  "AT$GPS=", "timeout[s]", "Get GPS position (NEOM8N)", AT_gps_callback},
#endif
#ifdef AT_COMMANDS_NVM
	{PARSER_MODE_COMMAND, "AT$NVMR", "\0", "Reset NVM data", AT_nvmr_callback},
	{PARSER_MODE_HEADER,  "AT$NVM=", "address[dec]", "Get NVM data", AT_nvm_callback},
	{PARSER_MODE_COMMAND, "AT$ID?", "\0", "Get Sigfox device ID", AT_get_id_callback},
	{PARSER_MODE_HEADER,  "AT$ID=", "id[hex]", "Set Sigfox device ID", AT_set_id_callback},
	{PARSER_MODE_COMMAND, "AT$KEY?", "\0", "Get Sigfox device key", AT_get_key_callback},
	{PARSER_MODE_HEADER,  "AT$KEY=", "key[hex]", "Set Sigfox device key", AT_set_key_callback},
#endif
#ifdef AT_COMMANDS_SIGFOX
	{PARSER_MODE_COMMAND, "AT$SO", "\0", "Sigfox send control message", AT_so_callback},
	{PARSER_MODE_HEADER,  "AT$SB=", "data[bit],(bidir_flag[bit])", "Sigfox send bit", AT_sb_callback},
	{PARSER_MODE_HEADER,  "AT$SF=", "data[hex],(bidir_flag[bit])", "Sigfox send frame", AT_sf_callback},
#endif
#ifdef AT_COMMANDS_CW_RSSI
	{PARSER_MODE_HEADER,  "AT$CW=", "frequency[hz],enable[bit],(output_power[dbm])", "Start or stop continuous radio transmission", AT_cw_callback},
	{PARSER_MODE_HEADER,  "AT$RSSI=", "frequency[hz],duration[s]", "Start or stop continuous RSSI measurement", AT_rssi_callback},
#endif
#ifdef AT_COMMANDS_RC
	{PARSER_MODE_COMMAND, "AT$RC?", "\0", "Get Sigfox radio configuration", AT_get_rc_callback},
	{PARSER_MODE_HEADER,  "AT$RC=", "rc[dec]", "Set Sigfox radio configurationt", AT_set_rc_callback},
#endif
#ifdef AT_COMMANDS_TEST_MODES
	{PARSER_MODE_HEADER,  "AT$TM=", "test_mode[dec]", "Execute Sigfox test mode", AT_tm_callback},
#endif
};
static AT_context_t at_ctx = {
	.sigfox_rc = (sfx_rc_t) RC1,
	.sigfox_rc_idx = SFX_RC1
};
#ifdef AT_COMMANDS_RC
static const sfx_u32 rc2_sm_config[SIGFOX_RC_STD_CONFIG_SIZE] = RC2_SM_CONFIG;
static const sfx_u32 rc4_sm_config[SIGFOX_RC_STD_CONFIG_SIZE] = RC4_SM_CONFIG;
static const sfx_u32 rc3a_config[SIGFOX_RC_STD_CONFIG_SIZE] = RC3A_CONFIG;
static const sfx_u32 rc3c_config[SIGFOX_RC_STD_CONFIG_SIZE] = RC3C_CONFIG;
static const sfx_u32 rc5_config[SIGFOX_RC_STD_CONFIG_SIZE] = RC5_CONFIG;
#endif

/*** AT local functions ***/

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void AT_response_add_string(char* tx_string) {
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		at_ctx.response_buf[at_ctx.response_buf_idx++] = *(tx_string++);
		// Manage rollover.
		if (at_ctx.response_buf_idx >= AT_RESPONSE_BUFFER_LENGTH) {
			at_ctx.response_buf_idx = 0;
		}
	}
}

/* APPEND A VALUE TO THE REPONSE BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void AT_response_add_value(int tx_value, STRING_format_t format, unsigned char print_prefix) {
	// Local variables.
	char str_value[AT_STRING_VALUE_BUFFER_LENGTH];
	unsigned char idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_STRING_VALUE_BUFFER_LENGTH ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	STRING_convert_value(tx_value, format, print_prefix, str_value);
	// Add string.
	AT_response_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void AT_response_send(void) {
	// Local variables.
	unsigned int idx = 0;
	// Send response over UART.
	USARTx_send_string(at_ctx.response_buf);
	// Flush response buffer.
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) at_ctx.response_buf[idx] = STRING_CHAR_NULL;
	at_ctx.response_buf_idx = 0;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void AT_print_ok(void) {
	AT_response_add_string("OK");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error_source:	8-bits error source.
 * @param error_code:	16-bits error code.
 * @return:				None.
 */
static void AT_print_status(SPSWS_status_t status) {
	AT_response_add_string("ERROR ");
	if (status < 0x0100) {
		AT_response_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
		AT_response_add_value(status, STRING_FORMAT_HEXADECIMAL, 0);
	}
	else {
		AT_response_add_value(status, STRING_FORMAT_HEXADECIMAL, 1);
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

#define AT_status_check(status, success, error_base) { if (status != success) { AT_print_status(error_base + status); goto errors;} }

/* PRINT ALL SUPPORTED AT COMMANDS.
 * @param:	None.
 * @return:	None.
 */
static void AT_print_command_list(void) {
	// Local variables.
	unsigned int idx = 0;
	// Commands loop.
	for (idx=0 ; idx<(sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)) ; idx++) {
		// Print syntax.
		AT_response_add_string(AT_COMMAND_LIST[idx].syntax);
		// Print parameters.
		AT_response_add_string(AT_COMMAND_LIST[idx].parameters);
		AT_response_add_string(AT_RESPONSE_END);
		// Print description.
		AT_response_add_string(AT_RESPONSE_TAB);
		AT_response_add_string(AT_COMMAND_LIST[idx].description);
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
	}
}

#ifdef AT_COMMANDS_SENSORS
/* AT$MCU? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_adc_callback(void) {
	// Local variables.
	ADC_status_t adc1_status = ADC_SUCCESS;
	unsigned int vmcu_mv = 0;
	signed char tmcu_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Trigger internal ADC conversions.
	ADC1_enable();
	AT_response_add_string("ADC running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	adc1_status = ADC1_perform_measurements();
	AT_status_check(adc1_status, ADC_SUCCESS, SPSWS_ERROR_BASE_ADC);
	// Read data.
	ADC1_get_tmcu(&tmcu_degrees);
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &vmcu_mv);
	AT_status_check(adc1_status, ADC_SUCCESS, SPSWS_ERROR_BASE_ADC);
	// Print results.
	AT_response_add_string("Vmcu=");
	AT_response_add_value((int) vmcu_mv, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("mV Tmcu=");
	AT_response_add_value((int) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("dC");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	ADC1_disable();
	return;
}

/* AT$ADC? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_max11136_callback(void) {
	// Local variables.
	MAX11136_status_t max11136_status = MAX11136_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	unsigned int data = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Power on ADC.
#ifdef HW1_0
	spi_status = SPI1_power_on();
#endif
#ifdef HW2_0
	spi_status = SPI2_power_on();
#endif
	AT_status_check(spi_status, SPI_SUCCESS, SPSWS_ERROR_BASE_SPI);
	// Run external ADC conversions.
	AT_response_add_string("MAX11136 running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	max11136_status = MAX11136_perform_measurements();
	AT_status_check(max11136_status, MAX11136_SUCCESS, SPSWS_ERROR_BASE_MAX11136);
	// Vsrc.
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VSRC_MV, &data);
	AT_status_check(max11136_status, MAX11136_SUCCESS, SPSWS_ERROR_BASE_MAX11136);
	AT_response_add_string("Vsrc=");
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	// Vcap.
	AT_response_add_string("mV Vcap=");
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &data);
	AT_status_check(max11136_status, MAX11136_SUCCESS, SPSWS_ERROR_BASE_MAX11136);
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	// LDR.
	AT_response_add_string("mV Light=");
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &data);
	AT_status_check(max11136_status, MAX11136_SUCCESS, SPSWS_ERROR_BASE_MAX11136);
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("%");
	// Response end.
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
#ifdef HW1_0
	SPI1_power_off();
#endif
#ifdef HW2_0
	SPI2_power_off();
#endif
	return;
}

/* AT$ITHS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_iths_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	signed char tamb_degrees = 0;
	unsigned char hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	AT_status_check(i2c1_status, I2C_SUCCESS, SPSWS_ERROR_BASE_I2C);
	AT_response_add_string("SHT3X running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sht3x_status = SHT3X_perform_measurements(SHT3X_INTERNAL_I2C_ADDRESS);
	AT_status_check(sht3x_status, SHT3X_SUCCESS, SPSWS_ERROR_BASE_SHT3X);
	// Read data.
	SHT3X_get_temperature(&tamb_degrees);
	SHT3X_get_humidity(&hamb_percent);
	// Print results.
	AT_response_add_string("T=");
	AT_response_add_value((int) tamb_degrees, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("dC H=");
	AT_response_add_value((int) hamb_percent, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("%");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	I2C1_power_off();
	return;
}

/* AT$ETHS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_eths_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	signed char tamb_degrees = 0;
	unsigned char hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	AT_status_check(i2c1_status, I2C_SUCCESS, SPSWS_ERROR_BASE_I2C);
	AT_response_add_string("SHT3X running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sht3x_status = SHT3X_perform_measurements(SHT3X_EXTERNAL_I2C_ADDRESS);
	AT_status_check(sht3x_status, SHT3X_SUCCESS, SPSWS_ERROR_BASE_SHT3X);
	// Read data.
	SHT3X_get_temperature(&tamb_degrees);
	SHT3X_get_humidity(&hamb_percent);
	// Print results.
	AT_response_add_string("T=");
	AT_response_add_value((int) tamb_degrees, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("dC H=");
	AT_response_add_value(hamb_percent, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("%");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	I2C1_power_off();
	return;
}

/* AT$EPTS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_epts_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	DPS310_status_t dps310_status = DPS310_SUCCESS;
	unsigned int pressure_pa = 0;
	signed char tamb_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	AT_status_check(i2c1_status, I2C_SUCCESS, SPSWS_ERROR_BASE_I2C);
	AT_response_add_string("DPS310 running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	dps310_status = DPS310_perform_measurements(DPS310_EXTERNAL_I2C_ADDRESS);
	AT_status_check(dps310_status, DPS310_SUCCESS, SPSWS_ERROR_BASE_DPS310);
	// Read data.
	DPS310_get_pressure(&pressure_pa);
	DPS310_get_temperature(&tamb_degrees);
	// Print results.
	AT_response_add_string("P=");
	AT_response_add_value((int) pressure_pa, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("Pa T=");
	AT_response_add_value((int) tamb_degrees, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("dC");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	I2C1_power_off();
	return;
}

/* AT$EUVS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_euvs_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SI1133_status_t si1133_status = SI1133_SUCCESS;
	unsigned char uv_index = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	AT_status_check(i2c1_status, I2C_SUCCESS, SPSWS_ERROR_BASE_I2C);
	AT_response_add_string("SI1133 running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	si1133_status = SI1133_perform_measurements(SI1133_EXTERNAL_I2C_ADDRESS);
	AT_status_check(si1133_status, SI1133_SUCCESS, SPSWS_ERROR_BASE_SI1133);
	// Read data.
	SI1133_get_uv_index(&uv_index);
	// Print result.
	AT_response_add_string("UVI=");
	AT_response_add_value(uv_index, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	I2C1_power_off();
	return;
}

/* AT$WIND EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_wind_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	WIND_status_t wind_status = WIND_SUCCESS;
	int enable = 0;
	unsigned int wind_speed_average = 0;
	unsigned int wind_speed_peak = 0;
	unsigned int wind_direction = 0;
	// Read enable parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Start or stop wind continuous measurements.
	if (enable == 0) {
		RTC_disable_alarm_b_interrupt();
		RTC_clear_alarm_b_flag();
		WIND_stop_continuous_measure();
		// Update flag.
		at_ctx.wind_measurement_flag = 0;
		// Get results.
		WIND_get_speed(&wind_speed_average, &wind_speed_peak);
		wind_status = WIND_get_direction(&wind_direction);
		AT_status_check(wind_status, WIND_SUCCESS, SPSWS_ERROR_BASE_WIND);
		// Print results.
		AT_response_add_string("AverageSpeed=");
		AT_response_add_value((int) wind_speed_average, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("m/h PeakSpeed=");
		AT_response_add_value((int) wind_speed_peak, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("m/h AverageDirection=");
		AT_response_add_value((int) wind_direction, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("d");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		// Reset data.
		WIND_reset_data();

	}
	else {
		WIND_start_continuous_measure();
		RTC_enable_alarm_b_interrupt();
		// Update flag.
		at_ctx.wind_measurement_flag = 1;
		AT_print_ok();
	}
errors:
	return;
}

/* AT$RAIN EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_rain_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	int enable = 0;
	unsigned char rain_mm = 0;
	// Read enable parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Start or stop rain continuous measurements.
	if (enable == 0) {
		RAIN_stop_continuous_measure();
		// Get result.
		RAIN_get_pluviometry(&rain_mm);
		// Print data.
		AT_response_add_string("Rain=");
		AT_response_add_value((int) rain_mm, STRING_FORMAT_DECIMAL,0);
		AT_response_add_string("mm");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		// Reset data.
		RAIN_reset_data();
	}
	else {
		RAIN_start_continuous_measure();
		AT_print_ok();
	}
errors:
	return;
}
#endif

#ifdef AT_COMMANDS_GPS
/* AT$TIME EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_time_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	int timeout_seconds = 0;
	RTC_time_t gps_timestamp;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &timeout_seconds);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Power on GPS.
	lpuart1_status = LPUART1_power_on();
	AT_status_check(lpuart1_status, LPUART_SUCCESS, SPSWS_ERROR_BASE_LPUART);
	// Start time aquisition.
	AT_response_add_string("GPS running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	neom8n_status = NEOM8N_get_time(&gps_timestamp, (unsigned int) timeout_seconds, 0);
	AT_status_check(neom8n_status, NEOM8N_SUCCESS, SPSWS_ERROR_BASE_NEOM8N);
	// Year.
	AT_response_add_value((gps_timestamp.year), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("-");
	// Month.
	if ((gps_timestamp.month) < 10) {
		AT_response_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_response_add_value((gps_timestamp.month), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("-");
	// Day.
	if ((gps_timestamp.date) < 10) {
		AT_response_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_response_add_value((gps_timestamp.date), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(" ");
	// Hours.
	if ((gps_timestamp.hours) < 10) {
		AT_response_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_response_add_value((gps_timestamp.hours), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(":");
	// Minutes.
	if ((gps_timestamp.minutes) < 10) {
		AT_response_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_response_add_value((gps_timestamp.minutes), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(":");
	// Seconds.
	if ((gps_timestamp.seconds) < 10) {
		AT_response_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_response_add_value((gps_timestamp.seconds), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	LPUART1_power_off();
	return;
}

/* AT$GPS EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_gps_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	int timeout_seconds = 0;
	unsigned int fix_duration_seconds = 0;
	NEOM8N_position_t gps_position;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &timeout_seconds);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Power on GPS.
	lpuart1_status = LPUART1_power_on();
	AT_status_check(lpuart1_status, LPUART_SUCCESS, SPSWS_ERROR_BASE_LPUART);
	// Start GPS fix.
	AT_response_add_string("GPS running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	neom8n_status = NEOM8N_get_position(&gps_position, (unsigned int) timeout_seconds, &fix_duration_seconds);
	AT_status_check(neom8n_status, NEOM8N_SUCCESS, SPSWS_ERROR_BASE_NEOM8N);
	// Latitude.
	AT_response_add_string("Lat=");
	AT_response_add_value((gps_position.lat_degrees), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("d");
	AT_response_add_value((gps_position.lat_minutes), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("'");
	AT_response_add_value((gps_position.lat_seconds), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("''-");
	AT_response_add_string(((gps_position.lat_north_flag) == 0) ? "S" : "N");
	// Longitude.
	AT_response_add_string(" Long=");
	AT_response_add_value((gps_position.long_degrees), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("d");
	AT_response_add_value((gps_position.long_minutes), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("'");
	AT_response_add_value((gps_position.long_seconds), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("''-");
	AT_response_add_string(((gps_position.long_east_flag) == 0) ? "W" : "E");
	// Altitude.
	AT_response_add_string(" Alt=");
	AT_response_add_value((gps_position.altitude), STRING_FORMAT_DECIMAL, 0);
	// Fix duration.
	AT_response_add_string("m Fix=");
	AT_response_add_value(fix_duration_seconds, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("s");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	LPUART1_power_off();
	return;
}
#endif

#ifdef AT_COMMANDS_NVM
/* AT$NVMR EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_nvmr_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Reset all NVM field to default value.
	NVM_enable();
	nvm_status = NVM_reset_default();
	AT_status_check(nvm_status, NVM_SUCCESS, SPSWS_ERROR_BASE_NVM);
	AT_print_ok();
errors:
	NVM_disable();
	return;
}

/* AT$NVM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_nvm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	int address = 0;
	unsigned char nvm_data = 0;
	// Read address parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &address);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Read byte at requested address.
	NVM_enable();
	nvm_status = NVM_read_byte((unsigned short) address, &nvm_data);
	AT_status_check(nvm_status, NVM_SUCCESS, SPSWS_ERROR_BASE_NVM);
	// Print data.
	AT_response_add_value(nvm_data, STRING_FORMAT_HEXADECIMAL, 1);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	NVM_disable();
	return;
}

/* AT$ID? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_get_id_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	unsigned char idx = 0;
	unsigned char id_byte = 0;
	// Retrieve device ID in NVM.
	NVM_enable();
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), &id_byte);
		AT_status_check(nvm_status, NVM_SUCCESS, SPSWS_ERROR_BASE_NVM);
		AT_response_add_value(id_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	NVM_disable();
	return;
}

/* AT$ID EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_set_id_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	unsigned char device_id[ID_LENGTH];
	unsigned char extracted_length = 0;
	unsigned char idx = 0;
	// Read ID parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, ID_LENGTH, device_id, &extracted_length);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Check length.
	if (extracted_length != ID_LENGTH) {
		AT_print_status(SPSWS_ERROR_BASE_PARSER + PARSER_ERROR_BYTE_ARRAY_LENGTH);
		goto errors;
	}
	// Write device ID in NVM.
	NVM_enable();
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), device_id[idx]);
		AT_status_check(nvm_status, NVM_SUCCESS, SPSWS_ERROR_BASE_NVM);
	}
	AT_print_ok();
errors:
	NVM_disable();
	return;
}

/* AT$KEY? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_get_key_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	unsigned char idx = 0;
	unsigned char key_byte = 0;
	// Retrieve device key in NVM.
	NVM_enable();
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), &key_byte);
		AT_status_check(nvm_status, NVM_SUCCESS, SPSWS_ERROR_BASE_NVM);
		AT_response_add_value(key_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	NVM_disable();
	return;
}

/* AT$KEY EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_set_key_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	unsigned char device_key[AES_BLOCK_SIZE];
	unsigned char extracted_length = 0;
	unsigned char idx = 0;
	// Read key parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, AES_BLOCK_SIZE, device_key, &extracted_length);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Check length.
	if (extracted_length != AES_BLOCK_SIZE) {
		AT_print_status(SPSWS_ERROR_BASE_PARSER + PARSER_ERROR_BYTE_ARRAY_LENGTH);
		goto errors;
	}
	// Write device ID in NVM.
	NVM_enable();
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), device_key[idx]);
		AT_status_check(nvm_status, NVM_SUCCESS, SPSWS_ERROR_BASE_NVM);
	}
	AT_print_ok();
errors:
	NVM_disable();
	return;
}
#endif

#ifdef AT_COMMANDS_SIGFOX
/* PRINT SIGFOX DOWNLINK DATA ON AT INTERFACE.
 * @param dl_payload:	Downlink data to print.
 * @return:				None.
 */
static void AT_print_dl_payload(sfx_u8* dl_payload) {
	AT_response_add_string("+RX=");
	unsigned char idx = 0;
	for (idx=0 ; idx<8 ; idx++) {
		AT_response_add_value(dl_payload[idx], STRING_FORMAT_HEXADECIMAL, 0);
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* AT$SO EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_so_callback(void) {
	// Local variables.
	sfx_error_t sfx_status = SFX_ERR_NONE;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Send Sigfox OOB frame.
	sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
	AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
	AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	AT_response_add_string("Sigfox library running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sfx_status = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
	AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	AT_print_ok();
errors:
	SIGFOX_API_close();
	return;
}

/* AT$SB EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_sb_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sfx_status = SFX_ERR_NONE;
	int data = 0;
	int bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// First try with 2 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 0, &data);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &bidir_flag);
		AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
		// Send Sigfox bit with specified downlink request.
		sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sfx_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, (sfx_bool) bidir_flag);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		if (bidir_flag != SFX_FALSE) {
			AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &data);
		AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
		// Send Sigfox bit with no downlink request (by default).
		sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sfx_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, 0);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	}
	AT_print_ok();
errors:
	SIGFOX_API_close();
	return;
}

/* AT$SF EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_sf_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sfx_status = SFX_ERR_NONE;
	sfx_u8 data[SIGFOX_UPLINK_DATA_MAX_SIZE_BYTES];
	unsigned char extracted_length = 0;
	int bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// First try with 2 parameters.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 0, 12, data, &extracted_length);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &bidir_flag);
		AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
		// Send Sigfox frame with specified downlink request.
		sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sfx_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, bidir_flag);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		if (bidir_flag != 0) {
			AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, 12, data, &extracted_length);
		AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
		// Send Sigfox frame with no downlink request (by default).
		sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sfx_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, 0);
		AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	}
	AT_print_ok();
errors:
	SIGFOX_API_close();
	return;
}
#endif

#ifdef AT_COMMANDS_CW_RSSI
/* AT$CW EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_cw_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	sfx_error_t sfx_status = SFX_ERR_NONE;
	int enable = 0;
	int frequency_hz = 0;
	int power_dbm = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &frequency_hz);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// First try with 3 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 0, &enable);
	if (parser_status == PARSER_SUCCESS) {
		// There is a third parameter, try to parse power.
		parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &power_dbm);
		AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
		// CW with given output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sfx_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
			sx1232_status = SX1232_set_rf_output_power((unsigned char) power_dbm);
			AT_status_check(sx1232_status, SX1232_SUCCESS, SPSWS_ERROR_BASE_SX1232);
		}
	}
	else {
		// Power is not given, try to parse enable as last parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
		AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
		// CW with last output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sfx_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
		}
	}
	AT_print_ok();
	return;
errors:
	SIGFOX_API_stop_continuous_transmission();
	return;
}

/* AT$RSSI EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_rssi_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	sfx_error_t sfx_status = SFX_ERR_NONE;
	int frequency_hz = 0;
	int duration_s = 0;
	signed short rssi_dbm = 0;
	unsigned int report_loop = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &frequency_hz);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Read duration parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &duration_s);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Init radio.
	sfx_status = RF_API_init(SFX_RF_MODE_RX);
	AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	sfx_status = RF_API_change_frequency((sfx_u32) frequency_hz);
	AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	// Start continuous listening.
	sx1232_status = SX1232_set_mode(SX1232_MODE_FSRX);
	AT_status_check(sx1232_status, SX1232_SUCCESS, SPSWS_ERROR_BASE_SX1232);
	// Wait TS_FS=60us typical.
	lptim1_status = LPTIM1_delay_milliseconds(5, 0);
	AT_status_check(lptim1_status, LPTIM_SUCCESS, SPSWS_ERROR_BASE_LPTIM);
	sx1232_status = SX1232_set_mode(SX1232_MODE_RX);
	AT_status_check(sx1232_status, SX1232_SUCCESS, SPSWS_ERROR_BASE_SX1232);
	// Wait TS_TR=120us typical.
	lptim1_status = LPTIM1_delay_milliseconds(5, 0);
	AT_status_check(lptim1_status, LPTIM_SUCCESS, SPSWS_ERROR_BASE_LPTIM);
	// Measurement loop.
	while (report_loop < ((duration_s * 1000) / AT_RSSI_REPORT_PERIOD_MS)) {
		// Read RSSI.
		sx1232_status = SX1232_get_rssi(&rssi_dbm);
		AT_status_check(sx1232_status, SX1232_SUCCESS, SPSWS_ERROR_BASE_SX1232);
		// Print RSSI.
		AT_response_add_string("RSSI=");
		AT_response_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("dBm\n");
		AT_response_send();
		// Report delay.
		lptim1_status = LPTIM1_delay_milliseconds(AT_RSSI_REPORT_PERIOD_MS, 0);
		AT_status_check(lptim1_status, LPTIM_SUCCESS, SPSWS_ERROR_BASE_LPTIM);
		report_loop++;
	}
	AT_print_ok();
errors:
	RF_API_stop();
	return;
}
#endif

#ifdef AT_COMMANDS_RC
/* AT$RC? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_get_rc_callback(void) {
	// Read current radio configuration.
	switch (at_ctx.sigfox_rc_idx) {
	case SFX_RC1:
		AT_response_add_string("RC1");
		break;
	case SFX_RC2:
		AT_response_add_string("RC2");
		break;
	case SFX_RC3A:
		AT_response_add_string("RC3A");
		break;
	case SFX_RC3C:
		AT_response_add_string("RC3C");
		break;
	case SFX_RC4:
		AT_response_add_string("RC4");
		break;
	case SFX_RC5:
		AT_response_add_string("RC5");
		break;
	case SFX_RC6:
		AT_response_add_string("RC6");
		break;
	case SFX_RC7:
		AT_response_add_string("RC7");
		break;
	default:
		AT_print_status(SPSWS_ERROR_SIGFOX_RC);
		goto errors;
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	return;
}

/* AT$RC EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_set_rc_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	int rc_index = 0;
	unsigned char idx = 0;
	// Read RC parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &rc_index);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Check RC index.
	if (((sfx_rc_enum_t) rc_index) >= SFX_RC_LIST_MAX_SIZE) {
		AT_print_status(SPSWS_ERROR_SIGFOX_RC);
		goto errors;
	}
	// Update radio configuration.
	switch ((sfx_rc_enum_t) rc_index) {
	case SFX_RC1:
		at_ctx.sigfox_rc = (sfx_rc_t) RC1;
		at_ctx.sigfox_rc_idx = SFX_RC1;
		break;
	case SFX_RC2:
		at_ctx.sigfox_rc = (sfx_rc_t) RC1;
		for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc2_sm_config[idx];
		at_ctx.sigfox_rc_idx = SFX_RC2;
		break;
	case SFX_RC3A:
		at_ctx.sigfox_rc = (sfx_rc_t) RC3A;
		for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc3a_config[idx];
		at_ctx.sigfox_rc_idx = SFX_RC3A;
		break;
	case SFX_RC3C:
		at_ctx.sigfox_rc = (sfx_rc_t) RC3C;
		for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc3c_config[idx];
		at_ctx.sigfox_rc_idx = SFX_RC3C;
		break;
	case SFX_RC4:
		at_ctx.sigfox_rc = (sfx_rc_t) RC4;
		for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc4_sm_config[idx];
		at_ctx.sigfox_rc_idx = SFX_RC4;
		break;
	case SFX_RC5:
		at_ctx.sigfox_rc = (sfx_rc_t) RC5;
		for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc5_config[idx];
		at_ctx.sigfox_rc_idx = SFX_RC5;
		break;
	case SFX_RC6:
		at_ctx.sigfox_rc = (sfx_rc_t) RC6;
		at_ctx.sigfox_rc_idx = SFX_RC6;
		break;
	case SFX_RC7:
		at_ctx.sigfox_rc = (sfx_rc_t) RC7;
		at_ctx.sigfox_rc_idx = SFX_RC7;
		break;
	default:
		AT_print_status(SPSWS_ERROR_SIGFOX_RC);
		goto errors;
	}
	AT_print_ok();
errors:
	return;
}
#endif

#ifdef AT_COMMANDS_TEST_MODES
/* AT$TM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_tm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sfx_status = SFX_ERR_NONE;
	int rc_index = 0;
	int test_mode = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(SPSWS_ERROR_BUSY);
		goto errors;
	}
	// Read RC parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &rc_index);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Read test mode parameter.
	parser_status =  PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &test_mode);
	AT_status_check(parser_status, PARSER_SUCCESS, SPSWS_ERROR_BASE_PARSER);
	// Call test mode function wth public key.
	AT_response_add_string("Sigfox addon running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sfx_status = ADDON_SIGFOX_RF_PROTOCOL_API_test_mode((sfx_rc_enum_t) rc_index, (sfx_test_mode_t) test_mode);
	AT_status_check(sfx_status, SFX_ERR_NONE, SPSWS_ERROR_BASE_SIGFOX);
	AT_print_ok();
errors:
	return;
}
#endif

/* RESET AT PARSER.
 * @param:	None.
 * @return:	None.
 */
static void AT_reset_parser(void) {
	// Reset parsing variables.
	at_ctx.command_buf_idx = 0;
	at_ctx.line_end_flag = 0;
	at_ctx.parser.rx_buf = (unsigned char*) at_ctx.command_buf;
	at_ctx.parser.rx_buf_length = 0;
	at_ctx.parser.separator_idx = 0;
	at_ctx.parser.start_idx = 0;
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void AT_decode(void) {
	// Local variables.
	unsigned int idx = 0;
	unsigned char decode_success = 0;
	// Empty or too short command.
	if (at_ctx.command_buf_idx < AT_COMMAND_LENGTH_MIN) {
		AT_print_status(SPSWS_ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND);
		goto errors;
	}
	// Update parser length.
	at_ctx.parser.rx_buf_length = (at_ctx.command_buf_idx - 1); // To ignore line end.
	// Loop on available commands.
	for (idx=0 ; idx<(sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)) ; idx++) {
		// Check type.
		if (PARSER_compare(&at_ctx.parser, AT_COMMAND_LIST[idx].mode, AT_COMMAND_LIST[idx].syntax) == PARSER_SUCCESS) {
			// Execute callback and exit.
			AT_COMMAND_LIST[idx].callback();
			decode_success = 1;
			break;
		}
	}
	if (decode_success == 0) {
		AT_print_status(SPSWS_ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
		goto errors;
	}
errors:
	AT_reset_parser();
	return;
}

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_init(void) {
	// Init context.
	unsigned int idx = 0;
	for (idx=0 ; idx<AT_COMMAND_BUFFER_LENGTH ; idx++) at_ctx.command_buf[idx] = '\0';
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) at_ctx.response_buf[idx] = '\0';
	at_ctx.response_buf_idx = 0;
	at_ctx.wind_measurement_flag = 0;
	// Reset parser.
	AT_reset_parser();
	// Enable USART interrupt.
#ifdef HW1_0
	NVIC_enable_interrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_enable_interrupt(NVIC_IT_USART1);
#endif
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_task(void) {
	// Trigger decoding function if line end found.
	if (at_ctx.line_end_flag) {
		AT_decode();
	}
	// Check RTC flag for wind measurements.
	if (RTC_get_alarm_b_flag() != 0) {
		// Call WIND callback.
		WIND_measurement_period_callback();
		RTC_clear_alarm_b_flag();
	}
}

/* FILL AT COMMAND BUFFER WITH A NEW BYTE (CALLED BY USART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void AT_fill_rx_buffer(unsigned char rx_byte) {
	// Append byte if LF flag is not allready set.
	if (at_ctx.line_end_flag == 0) {
		// Store new byte.
		at_ctx.command_buf[at_ctx.command_buf_idx] = rx_byte;
		// Manage index.
		at_ctx.command_buf_idx++;
		if (at_ctx.command_buf_idx >= AT_COMMAND_BUFFER_LENGTH) {
			at_ctx.command_buf_idx = 0;
		}
	}
	// Set LF flag to trigger decoding.
	if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
		at_ctx.line_end_flag = 1;
	}
}

/* PRINT SIGFOX LIBRARY RESULT.
 * @param test_result:	Test result.
 * @param rssi:			Downlink signal rssi in dBm.
 */
void AT_print_test_result(unsigned char test_result, int rssi_dbm) {
	// Check result.
	if (test_result == 0) {
		AT_response_add_string("Test failed.");
	}
	else {
		AT_response_add_string("Test passed. RSSI=");
		AT_response_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("dBm");
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* PRINT PLUVIOMETRY DATA.
 * @param rain_edge_count:	Rain gauge edge counter.
 * @return:					None.
 */
void AT_print_rain(unsigned char rain_edge_count) {
	// Print data.
	AT_response_add_string("RainEdgeCount=");
	AT_response_add_value((int) rain_edge_count, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* PRINT WIND SPEED.
 * @param speed_mh:	Wind speed in m/h.
 * @return:					None.
 */
void AT_print_wind_speed(unsigned int speed_mh) {
	// Print data.
	AT_response_add_string("Speed=");
	AT_response_add_value((int) speed_mh, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("m/h");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* PRINT WIND DIRECTION.
 * @param direction_degrees:	Wind direction in degrees.
 * @param direction_x:			Current direction vector x value.
 * @param direction_y:			Current direction vector y value.
 * @return:							None.
 */
void AT_print_wind_direction(unsigned int direction_degrees, int direction_x, int direction_y) {
	// Print data.
	AT_response_add_string("Direction=");
	AT_response_add_value((int) direction_degrees, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("d x=");
	AT_response_add_value(direction_x, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(" y=");
	AT_response_add_value(direction_y, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

#endif
