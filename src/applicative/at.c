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
#include "version.h"
#include "wind.h"

#ifdef ATM

/*** AT local macros ***/

// Enabled commands.
#define AT_COMMANDS_SENSORS
#define AT_COMMANDS_GPS
#define AT_COMMANDS_NVM
#define AT_COMMANDS_SIGFOX
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
static void AT_print_sw_version(void);
static void AT_print_error_stack(void);
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
#ifdef AT_COMMANDS_TEST_MODES
static void AT_tm_callback(void);
static void AT_cw_callback(void);
static void AT_dl_callback(void);
static void AT_rssi_callback(void);
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
	volatile char command_buf[AT_COMMAND_BUFFER_LENGTH];
	volatile unsigned int command_buf_idx;
	volatile unsigned char line_end_flag;
	PARSER_context_t parser;
	char response_buf[AT_RESPONSE_BUFFER_LENGTH];
	unsigned int response_buf_idx;
	// Wind measurement flag.
	unsigned char wind_measurement_flag;
	// Sigfox RC.
	sfx_rc_t sigfox_rc;
} AT_context_t;

/*** AT local global variables ***/

static const AT_command_t AT_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", "\0", "Ping command", AT_print_ok},
	{PARSER_MODE_COMMAND, "AT?", "\0", "List all available AT commands", AT_print_command_list},
	{PARSER_MODE_COMMAND, "AT$V?", "\0", "Get SW version", AT_print_sw_version},
	{PARSER_MODE_COMMAND, "AT$ERROR?", "\0", "Read error stack", AT_print_error_stack},
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
#ifdef AT_COMMANDS_TEST_MODES
	{PARSER_MODE_HEADER,  "AT$TM=", "rc_index[dec],test_mode[dec]", "Execute Sigfox test mode", AT_tm_callback},
	{PARSER_MODE_HEADER,  "AT$CW=", "frequency[hz],enable[bit],(output_power[dbm])", "Start or stop continuous radio transmission", AT_cw_callback},
	{PARSER_MODE_HEADER,  "AT$DL=", "frequency[hz]", "Continuous downlink frames decoding", AT_dl_callback},
	{PARSER_MODE_HEADER,  "AT$RSSI=", "frequency[hz],duration[s]", "Start or stop continuous RSSI measurement", AT_rssi_callback},
#endif
};
static AT_context_t at_ctx;

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
	STRING_status_t string_status = STRING_SUCCESS;
	char str_value[AT_STRING_VALUE_BUFFER_LENGTH];
	unsigned char idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_STRING_VALUE_BUFFER_LENGTH ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	string_status = STRING_value_to_string(tx_value, format, print_prefix, str_value);
	STRING_error_check();
	// Add string.
	AT_response_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void AT_response_send(void) {
	// Local variables.
	USART_status_t usart_status = USART_SUCCESS;
	unsigned int idx = 0;
	// Send response over UART.
	usart_status = USARTx_send_string(at_ctx.response_buf);
	USART_error_check();
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
static void AT_print_status(ERROR_t status) {
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

/* PRINT SW VERSION.
 * @param:	None.
 * @return:	None.
 */
static void AT_print_sw_version(void) {
	AT_response_add_string("GIT_VERSION=");
	AT_response_add_string(GIT_VERSION);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	AT_response_add_string("GIT_MAJOR_VERSION=");
	AT_response_add_value(GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	AT_response_add_string("GIT_MINOR_VERSION=");
	AT_response_add_value(GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	AT_response_add_string("GIT_COMMIT_INDEX=");
	AT_response_add_value(GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	AT_response_add_string("GIT_COMMIT_ID=");
	AT_response_add_value(GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 1);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	AT_response_add_string("GIT_DIRTY_FLAG=");
	AT_response_add_value(GIT_DIRTY_FLAG, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* PRINT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_print_error_stack(void) {
	// Local variables.
	ERROR_t error_stack[ERROR_STACK_DEPTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	unsigned int idx = 0;
	// Read stack.
	ERROR_stack_read(error_stack);
	// Print stack.
	AT_response_add_string("[ ");
	for (idx=0 ; idx<ERROR_STACK_DEPTH ; idx++) {
		AT_response_add_value((int) error_stack[idx], STRING_FORMAT_HEXADECIMAL, 1);
		AT_response_add_string(" ");
	}
	AT_response_add_string("]");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

#ifdef AT_COMMANDS_SENSORS
/* AT$MCU? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_adc_callback(void) {
	// Local variables.
	ADC_status_t adc_status = ADC_SUCCESS;
	unsigned int vmcu_mv = 0;
	signed char tmcu_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Trigger internal ADC conversions.
	AT_response_add_string("ADC running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	adc_status = ADC1_perform_measurements();
	ADC1_error_check_print();
	// Read data.
	ADC1_get_tmcu(&tmcu_degrees);
	adc_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &vmcu_mv);
	ADC1_error_check_print();
	// Print results.
	AT_response_add_string("Vmcu=");
	AT_response_add_value((int) vmcu_mv, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("mV Tmcu=");
	AT_response_add_value((int) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("dC");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
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
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Power on ADC.
#ifdef HW1_0
	spi_status = SPI1_power_on();
	SPI1_error_check_print();
#endif
#ifdef HW2_0
	spi_status = SPI2_power_on();
	SPI2_error_check_print();
#endif
	// Run external ADC conversions.
	AT_response_add_string("MAX11136 running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	max11136_status = MAX11136_perform_measurements();
	MAX11136_error_check_print();
	// Vsrc.
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VSRC_MV, &data);
	MAX11136_error_check_print();
	AT_response_add_string("Vsrc=");
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	// Vcap.
	AT_response_add_string("mV Vcap=");
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &data);
	MAX11136_error_check_print();
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	// LDR.
	AT_response_add_string("mV Light=");
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &data);
	MAX11136_error_check_print();
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
	I2C_status_t i2c_status = I2C_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	signed char tamb_degrees = 0;
	unsigned char hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c_status = I2C1_power_on();
	I2C1_error_check_print();
	AT_response_add_string("SHT3X running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sht3x_status = SHT3X_perform_measurements(SHT3X_INT_I2C_ADDRESS);
	SHT3X_INT_error_check_print();
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
	I2C_status_t i2c_status = I2C_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	signed char tamb_degrees = 0;
	unsigned char hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c_status = I2C1_power_on();
	I2C1_error_check_print();
	AT_response_add_string("SHT3X running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sht3x_status = SHT3X_perform_measurements(SHT3X_EXT_I2C_ADDRESS);
	SHT3X_EXT_error_check_print();
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
	I2C_status_t i2c_status = I2C_SUCCESS;
	DPS310_status_t dps310_status = DPS310_SUCCESS;
	unsigned int pressure_pa = 0;
	signed char tamb_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c_status = I2C1_power_on();
	I2C1_error_check_print();
	AT_response_add_string("DPS310 running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	dps310_status = DPS310_perform_measurements(DPS310_EXTERNAL_I2C_ADDRESS);
	DPS310_error_check_print();
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
	I2C_status_t i2c_status = I2C_SUCCESS;
	SI1133_status_t si1133_status = SI1133_SUCCESS;
	unsigned char uv_index = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c_status = I2C1_power_on();
	I2C1_error_check_print();
	AT_response_add_string("SI1133 running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	si1133_status = SI1133_perform_measurements(SI1133_EXTERNAL_I2C_ADDRESS);
	SI1133_error_check_print();
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
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
	PARSER_error_check_print();
	// Start or stop wind continuous measurements.
	if (enable == 0) {
		WIND_stop_continuous_measure();
		// Update flag.
		at_ctx.wind_measurement_flag = 0;
		// Get speeds.
		WIND_get_speed(&wind_speed_average, &wind_speed_peak);
		AT_response_add_string("AverageSpeed=");
		AT_response_add_value((int) wind_speed_average, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("m/h PeakSpeed=");
		AT_response_add_value((int) wind_speed_peak, STRING_FORMAT_DECIMAL, 0);
		// Get direction.
		AT_response_add_string("m/h AverageDirection=");
		wind_status = WIND_get_direction(&wind_direction);

		if (wind_status == (WIND_ERROR_BASE_MATH + MATH_ERROR_UNDEFINED)) {
			AT_response_add_string("N/A");
		}
		else {
			WIND_error_check_print();
			AT_response_add_value((int) wind_direction, STRING_FORMAT_DECIMAL, 0);
			AT_response_add_string("d");
		}
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		// Reset data.
		WIND_reset_data();
	}
	else {
		WIND_start_continuous_measure();
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
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
	PARSER_error_check_print();
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
	LPUART_status_t lpuart_status = LPUART_SUCCESS;
	int timeout_seconds = 0;
	RTC_time_t gps_timestamp;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 1, &timeout_seconds);
	PARSER_error_check_print();
	// Power on GPS.
	lpuart_status = LPUART1_power_on();
	LPUART1_error_check_print();
	// Start time aquisition.
	AT_response_add_string("GPS running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	neom8n_status = NEOM8N_get_time(&gps_timestamp, (unsigned int) timeout_seconds);
	NEOM8N_error_check_print();
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
	LPUART_status_t lpuart_status = LPUART_SUCCESS;
	int timeout_seconds = 0;
	unsigned int fix_duration_seconds = 0;
	NEOM8N_position_t gps_position;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 1, &timeout_seconds);
	PARSER_error_check_print();
	// Power on GPS.
	lpuart_status = LPUART1_power_on();
	LPUART1_error_check_print();
	// Start GPS fix.
	AT_response_add_string("GPS running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	neom8n_status = NEOM8N_get_position(&gps_position, (unsigned int) timeout_seconds, &fix_duration_seconds);
	NEOM8N_error_check_print();
	// Latitude.
	AT_response_add_string("Lat=");
	AT_response_add_value((gps_position.lat_degrees), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("d");
	AT_response_add_value((gps_position.lat_minutes), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("'");
	AT_response_add_value((gps_position.lat_seconds), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("''");
	AT_response_add_string(((gps_position.lat_north_flag) == 0) ? "S" : "N");
	// Longitude.
	AT_response_add_string(" Long=");
	AT_response_add_value((gps_position.long_degrees), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("d");
	AT_response_add_value((gps_position.long_minutes), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("'");
	AT_response_add_value((gps_position.long_seconds), STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("''");
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
	nvm_status = NVM_reset_default();
	NVM_error_check_print();
	AT_print_ok();
errors:
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
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 1, &address);
	PARSER_error_check_print();
	// Read byte at requested address.
	nvm_status = NVM_read_byte((unsigned short) address, &nvm_data);
	NVM_error_check_print();
	// Print data.
	AT_response_add_value(nvm_data, STRING_FORMAT_HEXADECIMAL, 1);
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
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
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), &id_byte);
		NVM_error_check_print();
		AT_response_add_value(id_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
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
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, ID_LENGTH, 1, device_id, &extracted_length);
	PARSER_error_check_print();
	// Write device ID in NVM.
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), device_id[idx]);
		NVM_error_check_print();
	}
	AT_print_ok();
errors:
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
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), &key_byte);
		NVM_error_check_print();
		AT_response_add_value(key_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
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
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, AES_BLOCK_SIZE, 1, device_key, &extracted_length);
	PARSER_error_check_print();
	// Write device ID in NVM.
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), device_key[idx]);
		NVM_error_check_print();
	}
	AT_print_ok();
errors:
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
	for (idx=0 ; idx<SIGFOX_DOWNLINK_DATA_SIZE_BYTES ; idx++) {
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
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Send Sigfox OOB frame.
	sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
	SIGFOX_API_error_check_print();
	AT_response_add_string("Sigfox library running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sigfox_api_status = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
	SIGFOX_API_error_check_print();
	AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}

/* AT$SB EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_sb_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int data = 0;
	int bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// First try with 2 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 0, &data);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 1, &bidir_flag);
		PARSER_error_check_print();
		// Send Sigfox bit with specified downlink request.
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sigfox_api_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, (sfx_bool) bidir_flag);
		SIGFOX_API_error_check_print();
		if (bidir_flag != SFX_FALSE) {
			AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 1, &data);
		PARSER_error_check_print();
		// Send Sigfox bit with no downlink request (by default).
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sigfox_api_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, 0);
		SIGFOX_API_error_check_print();
	}
	AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}

/* AT$SF EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_sf_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	sfx_u8 data[SIGFOX_UPLINK_DATA_MAX_SIZE_BYTES];
	unsigned char extracted_length = 0;
	int bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// First try with 2 parameters.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 0, 12, 0, data, &extracted_length);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 1, &bidir_flag);
		PARSER_error_check_print();
		// Send Sigfox frame with specified downlink request.
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sigfox_api_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, bidir_flag);
		SIGFOX_API_error_check_print();
		if (bidir_flag != 0) {
			AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, 12, 0, data, &extracted_length);
		PARSER_error_check_print();
		// Send Sigfox frame with no downlink request (by default).
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		AT_response_add_string("Sigfox library running...");
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
		sigfox_api_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, 0);
		SIGFOX_API_error_check_print();
	}
	AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}
#endif

#ifdef AT_COMMANDS_TEST_MODES
/* PRINT SIGFOX DOWNLINK FRAME ON AT INTERFACE.
 * @param dl_payload:	Downlink data to print.
 * @return:				None.
 */
static void AT_print_dl_phy_content(sfx_u8* dl_phy_content, int rssi_dbm) {
	AT_response_add_string("+DL_PHY=");
	unsigned char idx = 0;
	for (idx=0 ; idx<SIGFOX_DOWNLINK_PHY_SIZE_BYTES ; idx++) {
		AT_response_add_value(dl_phy_content[idx], STRING_FORMAT_HEXADECIMAL, 0);
	}
	AT_response_add_string(" RSSI=");
	AT_response_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("dBm");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* AT$TM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_tm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int rc_index = 0;
	int test_mode = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Read RC parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 0, &rc_index);
	PARSER_error_check_print();
	// Read test mode parameter.
	parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 1, &test_mode);
	PARSER_error_check_print();
	// Call test mode function wth public key.
	AT_response_add_string("Sigfox addon running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	sigfox_api_status = ADDON_SIGFOX_RF_PROTOCOL_API_test_mode((sfx_rc_enum_t) rc_index, (sfx_test_mode_t) test_mode);
	SIGFOX_API_error_check_print();
	AT_print_ok();
errors:
	return;
}

/* AT$CW EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_cw_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int enable = 0;
	int frequency_hz = 0;
	int power_dbm = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 0, &frequency_hz);
	PARSER_error_check_print();
	// First try with 3 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 0, &enable);
	if (parser_status == PARSER_SUCCESS) {
		// There is a third parameter, try to parse power.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 1, &power_dbm);
		PARSER_error_check_print();
		// CW with given output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sigfox_api_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			SIGFOX_API_error_check_print();
			sx1232_status = SX1232_set_rf_output_power((unsigned char) power_dbm);
			SX1232_error_check_print();
		}
	}
	else {
		// Power is not given, try to parse enable as last parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
		PARSER_error_check_print();
		// CW with last output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sigfox_api_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			SIGFOX_API_error_check_print();
		}
	}
	AT_print_ok();
	return;
errors:
	sigfox_api_status = SIGFOX_API_stop_continuous_transmission();
	SIGFOX_API_error_check();
	return;
}

/* AT$DL EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_dl_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	sfx_u8 dl_phy_content[SIGFOX_DOWNLINK_PHY_SIZE_BYTES];
	sfx_s16 rssi_dbm = 0;
	sfx_rx_state_enum_t dl_status = DL_PASSED;
	int frequency_hz = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 1, &frequency_hz);
	PARSER_error_check_print();
	// Start radio.
	sigfox_api_status = RF_API_init(SFX_RF_MODE_RX);
	SIGFOX_API_error_check_print();
	sigfox_api_status = RF_API_change_frequency(frequency_hz);
	SIGFOX_API_error_check_print();
	AT_response_add_string("RX GFSK running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	while (dl_status == DL_PASSED) {
		sigfox_api_status = RF_API_wait_frame(dl_phy_content, &rssi_dbm, &dl_status);
		SIGFOX_API_error_check_print();
		// Check result.
		if (dl_status == DL_PASSED) {
			AT_print_dl_phy_content(dl_phy_content, rssi_dbm);
		}
		else {
			AT_response_add_string("RX timeout");
			AT_response_add_string(AT_RESPONSE_END);
			AT_response_send();
		}
	}
errors:
	sigfox_api_status = RF_API_stop();
	SIGFOX_API_error_check();
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
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int frequency_hz = 0;
	int duration_s = 0;
	signed short rssi_dbm = 0;
	unsigned int report_loop = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		AT_print_status(ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 0, &frequency_hz);
	PARSER_error_check_print();
	// Read duration parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, 1, &duration_s);
	PARSER_error_check_print();
	// Init radio.
	sigfox_api_status = RF_API_init(SFX_RF_MODE_RX);
	SIGFOX_API_error_check_print();
	sigfox_api_status = RF_API_change_frequency((sfx_u32) frequency_hz);
	SIGFOX_API_error_check_print();
	// Start continuous listening.
	sx1232_status = SX1232_set_mode(SX1232_MODE_FSRX);
	SX1232_error_check_print();
	// Wait TS_FS=60us typical.
	lptim_status = LPTIM1_delay_milliseconds(5, 0);
	LPTIM1_error_check_print();
	sx1232_status = SX1232_set_mode(SX1232_MODE_RX);
	SX1232_error_check_print();
	// Wait TS_TR=120us typical.
	lptim_status = LPTIM1_delay_milliseconds(5, 0);
	LPTIM1_error_check_print();
	// Measurement loop.
	while (report_loop < ((duration_s * 1000) / AT_RSSI_REPORT_PERIOD_MS)) {
		// Read RSSI.
		sx1232_status = SX1232_get_rssi(&rssi_dbm);
		SX1232_error_check_print();
		// Print RSSI.
		AT_response_add_string("RSSI=");
		AT_response_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("dBm\n");
		AT_response_send();
		// Report delay.
		lptim_status = LPTIM1_delay_milliseconds(AT_RSSI_REPORT_PERIOD_MS, 0);
		LPTIM1_error_check_print();
		report_loop++;
	}
	AT_print_ok();
errors:
	sigfox_api_status = RF_API_stop();
	SIGFOX_API_error_check();
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
	at_ctx.parser.rx_buf = (char*) at_ctx.command_buf;
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
		AT_print_status(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND);
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
		AT_print_status(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
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
	at_ctx.sigfox_rc = (sfx_rc_t) RC1;
	// Reset parser.
	AT_reset_parser();
	// Enable USART interrupt.
	USARTx_enable_interrupt();
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_task(void) {
	// Trigger decoding function if line end found.
	if (at_ctx.line_end_flag) {
		// Decode and execute command.
		USARTx_disable_interrupt();
		AT_decode();
		USARTx_enable_interrupt();
	}
	// Check RTC flag for wind measurements.
	if ((at_ctx.wind_measurement_flag != 0) && (RTC_get_alarm_b_flag() != 0)) {
		// Call WIND callback.
		WIND_measurement_period_callback();
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
