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
#include "flash_reg.h"
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

typedef enum {
	AT_SUCCESS = 0,
	AT_ERROR_BUSY,
	AT_ERROR_LAST
} AT_status_t;

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
 * @param error_code:	Error code to display.
 * @return:				None.
 */
static void AT_print_error(ERROR_source_t error_source, unsigned int error_code) {
	AT_response_add_string("ERROR ");
	AT_response_add_value(error_source, STRING_FORMAT_HEXADECIMAL, 1);
	AT_response_add_value(error_code, STRING_FORMAT_HEXADECIMAL, 0);
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

#ifdef AT_COMMANDS_SENSORS
/* AT$MCU? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_adc_callback(void) {
	// Local variables.
	unsigned int vmcu_mv = 0;
	signed char tmcu_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag == 0) {
		// Trigger internal ADC conversions.
		ADC1_init();
		ADC1_perform_measurements();
		ADC1_disable();
		ADC1_get_tmcu(&tmcu_degrees);
		ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &vmcu_mv);
		// Print results.
		AT_response_add_string("Vmcu=");
		AT_response_add_value((int) vmcu_mv, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("mV Tmcu=");
		AT_response_add_value((int) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string("dC");
		AT_response_add_string(AT_RESPONSE_END);
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
	AT_response_send();
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
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
		goto errors;
	}
	AT_print_ok();
	// Power on ADC.
#ifdef HW1_0
	spi_status = SPI1_power_on();
#endif
#ifdef HW2_0
	spi_status = SPI2_power_on();
#endif
	if (spi_status != SPI_SUCCESS) {
		AT_print_error(ERROR_SOURCE_SPI, spi_status);
		goto errors;
	}
	AT_response_add_string("MAX11136 running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	// Run external ADC conversions.
	max11136_status = MAX11136_perform_measurements();
	// Power off ADC.
	// Note: status is not checked since GPS data could be valid.
#ifdef HW1_0
	SPI1_power_off();
#endif
#ifdef HW2_0
	SPI2_power_off();
#endif
	// Check ADC status.
	if (max11136_status != MAX11136_SUCCESS) {
		AT_print_error(ERROR_SOURCE_MAX11136, max11136_status);
		goto errors;
	}
	// Vsrc.
	MAX11136_get_data(MAX11136_DATA_INDEX_VSRC_MV, &data);
	AT_response_add_string("Vsrc=");
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	// Vcap.
	AT_response_add_string("mV Vcap=");
	MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &data);
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	// LDR.
	AT_response_add_string("mV Light=");
	MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &data);
	AT_response_add_value((int) data, STRING_FORMAT_DECIMAL, 0);
	AT_response_add_string("%");
	// Response end.
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
errors:
	return;
}

/* AT$ITHS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_iths_callback(void) {
	// Local variables.
	signed char tamb_degrees = 0;
	unsigned char hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag == 0) {
		// Perform measurements.
		I2C1_power_on();
		SHT3X_perform_measurements(SHT3X_INTERNAL_I2C_ADDRESS);
		I2C1_power_off();
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
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
}

/* AT$ETHS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_eths_callback(void) {
	// Local variables.
	signed char tamb_degrees = 0;
	unsigned char hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag == 0) {
		// Perform measurements.
		I2C1_power_on();
		SHT3X_perform_measurements(SHT3X_EXTERNAL_I2C_ADDRESS);
		I2C1_power_off();
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
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
}

/* AT$EPTS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_epts_callback(void) {
	// Local variables.
	unsigned int pressure_pa = 0;
	signed char tamb_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag == 0) {
		// Perform measurements.
		I2C1_power_on();
		DPS310_perform_measurements(DPS310_EXTERNAL_I2C_ADDRESS);
		I2C1_power_off();
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
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
}

/* AT$EUVS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_euvs_callback(void) {
	// Local variables.
	unsigned char uv_index = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag == 0) {
		// Perform measurements.
		I2C1_power_on();
		SI1133_perform_measurements(SI1133_EXTERNAL_I2C_ADDRESS);
		I2C1_power_off();
		SI1133_get_uv_index(&uv_index);
		// Print result.
		AT_response_add_string("UVI=");
		AT_response_add_value(uv_index, STRING_FORMAT_DECIMAL, 0);
		AT_response_add_string(AT_RESPONSE_END);
		AT_response_send();
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
}

/* AT$WIND EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_wind_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	int enable = 0;
	unsigned int wind_speed_average = 0;
	unsigned int wind_speed_peak = 0;
	unsigned int wind_direction = 0;
	// Read enable parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
	if (parser_status == PARSER_SUCCESS) {
		// Start or stop wind continuous measurements.
		if (enable == 0) {
			RTC_disable_alarm_b_interrupt();
			RTC_clear_alarm_b_flag();
			WIND_stop_continuous_measure();
			// Get results.
			WIND_get_speed(&wind_speed_average, &wind_speed_peak);
			WIND_get_direction(&wind_direction);
			// Print results.
			AT_response_add_string("AverageSpeed=");
			AT_response_add_value((int) wind_speed_average, STRING_FORMAT_DECIMAL, 0);
			AT_response_add_string("m/h PeakSpeed=");
			AT_response_add_value((int) wind_speed_peak, STRING_FORMAT_DECIMAL, 0);
			// TODO check wind status for direction.
			AT_response_add_string("m/h AverageDirection=");
			AT_response_add_value((int) wind_direction, STRING_FORMAT_DECIMAL, 0);
			AT_response_add_string("d");
			AT_response_add_string(AT_RESPONSE_END);
			AT_response_send();
			// Reset data.
			WIND_reset_data();
			// Update flag.
			at_ctx.wind_measurement_flag = 0;
		}
		else {
			WIND_start_continuous_measure();
			RTC_enable_alarm_b_interrupt();
			// Update flag.
			at_ctx.wind_measurement_flag = 1;
			AT_print_ok();
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in enable parameter.
	}
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
	if (parser_status == PARSER_SUCCESS) {
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
	}
	else {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in enable parameter.
	}
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
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &timeout_seconds);
	if (parser_status != PARSER_SUCCESS) {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status);
		goto errors;
	}
	AT_print_ok();
	// Power on GPS.
	lpuart1_status = LPUART1_power_on();
	if (lpuart1_status != LPUART_SUCCESS) {
		AT_print_error(ERROR_SOURCE_LPUART, lpuart1_status);
		goto errors;
	}
	AT_response_add_string("GPS running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	// Start time aquisition.
	neom8n_status = NEOM8N_get_time(&gps_timestamp, (unsigned int) timeout_seconds, 0);
	// Power off GPS.
	// Note: status is not checked since GPS data could be valid.
	LPUART1_power_off();
	// Check GPS result.
	if (neom8n_status != NEOM8N_SUCCESS) {
		AT_print_error(ERROR_SOURCE_NEOM8N, neom8n_status);
		goto errors;
	}
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
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &timeout_seconds);
	if (parser_status != PARSER_SUCCESS) {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status);
		goto errors;
	}
	AT_print_ok();
	// Power on GPS.
	lpuart1_status = LPUART1_power_on();
	if (lpuart1_status != LPUART_SUCCESS) {
		AT_print_error(ERROR_SOURCE_LPUART, lpuart1_status);
		goto errors;
	}
	AT_response_add_string("GPS running...");
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
	// Start GPS fix.
	neom8n_status = NEOM8N_get_position(&gps_position, (unsigned int) timeout_seconds, &fix_duration_seconds);
	// Power off GPS.
	// Note: status is not checked since GPS data could be valid.
	LPUART1_power_off();
	// Check GPS result.
	if (neom8n_status != NEOM8N_SUCCESS) {
		AT_print_error(ERROR_SOURCE_NEOM8N, neom8n_status);
		goto errors;
	}
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
	return;
}
#endif

#ifdef AT_COMMANDS_NVM
/* AT$NVMR EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_nvmr_callback(void) {
	// Reset all NVM field to default value.
	NVM_enable();
	NVM_reset_default();
	NVM_disable();
	AT_print_ok();
}

/* AT$NVM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_nvm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	int address = 0;
	unsigned char nvm_data = 0;
	// Read address parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &address);
	if (parser_status == PARSER_SUCCESS) {
		// Check if address is reachable.
		if (((unsigned short) address) < EEPROM_SIZE) {
			// Read byte at requested address.
			NVM_enable();
			NVM_read_byte((unsigned short) address, &nvm_data);
			NVM_disable();
			// Print byte.
			AT_response_add_value(nvm_data, STRING_FORMAT_HEXADECIMAL, 1);
			AT_response_add_string(AT_RESPONSE_END);
			AT_response_send();
		}
		else {
			AT_print_error(ERROR_SOURCE_NVM, 0);
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in address parameter.
	}
}

/* AT$ID? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_get_id_callback(void) {
	// Local variables.
	unsigned char idx = 0;
	unsigned char id_byte = 0;
	// Retrieve device ID in NVM.
	NVM_enable();
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), &id_byte);
		AT_response_add_value(id_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	NVM_disable();
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* AT$ID EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_set_id_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	unsigned char device_id[ID_LENGTH];
	unsigned char extracted_length = 0;
	unsigned char idx = 0;
	// Read ID parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, ID_LENGTH, device_id, &extracted_length);
	if (parser_status == PARSER_SUCCESS) {
		// Check length.
		if (extracted_length == ID_LENGTH) {
			// Write device ID in NVM.
			NVM_enable();
			for (idx=0 ; idx<ID_LENGTH ; idx++) {
				NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), device_id[idx]);
			}
			NVM_disable();
			AT_print_ok();
		}
		else {
			AT_print_error(ERROR_SOURCE_PARSER, PARSER_ERROR_PARAMETER_BYTE_ARRAY_INVALID_LENGTH);
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in ID parameter.
	}
}

/* AT$KEY? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_get_key_callback(void) {
	// Local variables.
	unsigned char idx = 0;
	unsigned char key_byte = 0;
	// Retrieve device key in NVM.
	NVM_enable();
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), &key_byte);
		AT_response_add_value(key_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	NVM_disable();
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
}

/* AT$KEY EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_set_key_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	unsigned char device_key[AES_BLOCK_SIZE];
	unsigned char extracted_length = 0;
	unsigned char idx = 0;
	// Read key parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, AES_BLOCK_SIZE, device_key, &extracted_length);
	if (parser_status == PARSER_SUCCESS) {
		// Check length.
		if (extracted_length == AES_BLOCK_SIZE) {
			// Write device ID in NVM.
			NVM_enable();
			for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
				NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), device_key[idx]);
			}
			NVM_disable();
			AT_print_ok();
		}
		else {
			AT_print_error(ERROR_SOURCE_PARSER, PARSER_ERROR_PARAMETER_BYTE_ARRAY_INVALID_LENGTH);
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in key parameter.
	}
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
	if (at_ctx.wind_measurement_flag == 0) {
		// Send Sigfox OOB frame.
		sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		if (sfx_status == SFX_ERR_NONE) {
			sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
			sfx_status = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
		}
		SIGFOX_API_close();
		if (sfx_status == SFX_ERR_NONE) {
			AT_print_ok();
		}
		else {
			AT_print_error(ERROR_SOURCE_SIGFOX, sfx_status); // Error from Sigfox library.
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
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
	if (at_ctx.wind_measurement_flag == 0) {
		// First try with 2 parameters.
		parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 0, &data);
		if (parser_status == PARSER_SUCCESS) {
			// Try parsing downlink request parameter.
			parser_status =  PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &bidir_flag);
			if (parser_status == PARSER_SUCCESS) {
				// Send Sigfox bit with specified downlink request.
				sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
				if (sfx_status == SFX_ERR_NONE) {
					sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
					sfx_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, (sfx_bool) bidir_flag);
				}
				SIGFOX_API_close();
				if (sfx_status == SFX_ERR_NONE) {
					if (bidir_flag != 0) {
						AT_print_dl_payload(dl_payload);
					}
					AT_print_ok();
				}
				else {
					AT_print_error(ERROR_SOURCE_SIGFOX, sfx_status); // Error from Sigfox library.
				}
			}
			else {
				AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in bidir flag parameter.
			}
		}
		else {
			// Try with 1 parameter.
			parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &data);
			if (parser_status == PARSER_SUCCESS) {
				// Send Sigfox bit with no downlink request (by default).
				sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
				if (sfx_status == SFX_ERR_NONE) {
					sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
					sfx_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, 0);
				}
				SIGFOX_API_close();
				if (sfx_status == SFX_ERR_NONE) {
					AT_print_ok();
				}
				else {
					AT_print_error(ERROR_SOURCE_SIGFOX, sfx_status); // Error from Sigfox library.
				}
			}
			else {
				AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in data parameter.
			}
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
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
	if (at_ctx.wind_measurement_flag == 0) {
		// First try with 2 parameters.
		parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 0, 12, data, &extracted_length);
		if (parser_status == PARSER_SUCCESS) {
			// Try parsing downlink request parameter.
			parser_status =  PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &bidir_flag);
			if (parser_status == PARSER_SUCCESS) {
				// Send Sigfox frame with specified downlink request.
				sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
				if (sfx_status == SFX_ERR_NONE) {
					sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
					sfx_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, bidir_flag);
				}
				SIGFOX_API_close();
				if (sfx_status == SFX_ERR_NONE) {
					if (bidir_flag != 0) {
						AT_print_dl_payload(dl_payload);
					}
					AT_print_ok();
				}
				else {
					AT_print_error(ERROR_SOURCE_SIGFOX, sfx_status); // Error from Sigfox library.
				}
			}
			else {
				AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in bidir flag parameter.
			}
		}
		else {
			// Try with 1 parameter.
			parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 1, 12, data, &extracted_length);
			if (parser_status == PARSER_SUCCESS) {
				// Send Sigfox frame with no downlink request (by default).
				sfx_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
				if (sfx_status == SFX_ERR_NONE) {
					sfx_status = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
					sfx_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, 0);
				}
				SIGFOX_API_close();
				if (sfx_status == SFX_ERR_NONE) {
					AT_print_ok();
				}
				else {
					AT_print_error(ERROR_SOURCE_SIGFOX, sfx_status); // Error from Sigfox library.
				}
			}
			else {
				AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in data parameter.
			}
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
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
	sfx_error_t sfx_status = SFX_ERR_NONE;
	int enable = 0;
	int frequency_hz = 0;
	int power_dbm = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag == 0) {
		// Read frequency parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &frequency_hz);
		if (parser_status == PARSER_SUCCESS) {
			// First try with 3 parameters.
			parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 0, &enable);
			if (parser_status != PARSER_SUCCESS) {
				// Power is not given, try to parse enable as last parameter.
				parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &enable);
				if (parser_status == PARSER_SUCCESS) {
					// CW with default output power.
					SIGFOX_API_stop_continuous_transmission();
					if (enable != 0) {
						sfx_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
						if (sfx_status == SFX_ERR_NONE) {
							AT_print_ok();
						}
						else {
							AT_print_error(ERROR_SOURCE_SIGFOX, sfx_status); // Error from Sigfox library.
						}
					}
				}
				else {
					// Error in enable parameter.
					AT_print_error(ERROR_SOURCE_PARSER, parser_status);
				}
			}
			else if (parser_status == PARSER_SUCCESS) {
				// There is a third parameter, try to parse power.
				parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &power_dbm);
				if (parser_status == PARSER_SUCCESS) {
					// CW with given output power.
					SIGFOX_API_stop_continuous_transmission();
					if (enable != 0) {
						SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
						SX1232_set_rf_output_power((unsigned char) power_dbm);
					}
					AT_print_ok();
				}
				else {
					AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in power parameter.
				}
			}
			else {
				AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in enable parameter.
			}
		}
		else {
			AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in frequency parameter.
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
}

/* AT$RSSI EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void AT_rssi_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	int frequency_hz = 0;
	int duration_s = 0;
	signed short rssi_dbm = 0;
	unsigned int report_loop = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag == 0) {
		// Read frequency parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &frequency_hz);
		if (parser_status == PARSER_SUCCESS) {
			// Read duration parameters.
			parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &duration_s);
			if (parser_status == PARSER_SUCCESS) {
				RF_API_init(SFX_RF_MODE_RX);
				RF_API_change_frequency((sfx_u32) frequency_hz);
				// Start continuous listening.
				SX1232_set_mode(SX1232_MODE_FSRX);
				LPTIM1_delay_milliseconds(5, 0); // Wait TS_FS=60us typical.
				SX1232_set_mode(SX1232_MODE_RX);
				LPTIM1_delay_milliseconds(5, 0); // Wait TS_TR=120us typical.
				while (report_loop < ((duration_s * 1000) / AT_RSSI_REPORT_PERIOD_MS)) {
					SX1232_get_rssi(&rssi_dbm);
					AT_response_add_string("RSSI=");
					AT_response_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
					AT_response_add_string("dBm\n");
					AT_response_send();
					LPTIM1_delay_milliseconds(AT_RSSI_REPORT_PERIOD_MS, 0);
					report_loop++;
				}
				// Stop radio.
				RF_API_stop();
				AT_print_ok();
			}
			else {
				AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in duration parameter.
			}
		}
		else {
			AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in frequency parameter.
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
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
		AT_response_add_string("Unknown RC");
		break;
	}
	AT_response_add_string(AT_RESPONSE_END);
	AT_response_send();
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
	if (parser_status == PARSER_SUCCESS) {
		// Check value.
		if (((sfx_rc_enum_t) rc_index) < SFX_RC_LIST_MAX_SIZE) {
			// Update radio configuration.
			switch ((sfx_rc_enum_t) rc_index) {
			case SFX_RC1:
				at_ctx.sigfox_rc = (sfx_rc_t) RC1;
				at_ctx.sigfox_rc_idx = SFX_RC1;
				AT_print_ok();
				break;
			case SFX_RC2:
				at_ctx.sigfox_rc = (sfx_rc_t) RC1;
				for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc2_sm_config[idx];
				at_ctx.sigfox_rc_idx = SFX_RC2;
				AT_print_ok();
				break;
			case SFX_RC3A:
				at_ctx.sigfox_rc = (sfx_rc_t) RC3A;
				for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc3a_config[idx];
				at_ctx.sigfox_rc_idx = SFX_RC3A;
				AT_print_ok();
				break;
			case SFX_RC3C:
				at_ctx.sigfox_rc = (sfx_rc_t) RC3C;
				for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc3c_config[idx];
				at_ctx.sigfox_rc_idx = SFX_RC3C;
				AT_print_ok();
				break;
			case SFX_RC4:
				at_ctx.sigfox_rc = (sfx_rc_t) RC4;
				for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc4_sm_config[idx];
				at_ctx.sigfox_rc_idx = SFX_RC4;
				AT_print_ok();
				break;
			case SFX_RC5:
				at_ctx.sigfox_rc = (sfx_rc_t) RC5;
				for (idx=0 ; idx<SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc5_config[idx];
				at_ctx.sigfox_rc_idx = SFX_RC5;
				AT_print_ok();
				break;
			case SFX_RC6:
				at_ctx.sigfox_rc = (sfx_rc_t) RC6;
				at_ctx.sigfox_rc_idx = SFX_RC6;
				AT_print_ok();
				break;
			case SFX_RC7:
				at_ctx.sigfox_rc = (sfx_rc_t) RC7;
				at_ctx.sigfox_rc_idx = SFX_RC7;
				AT_print_ok();
				break;
			default:
				AT_print_error(ERROR_SOURCE_AT, 0); // Invalid RC.
				break;
			}
		}
		else {
			AT_print_error(ERROR_SOURCE_AT, 0); // Invalid RC.
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in RC parameter.
	}
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
	if (at_ctx.wind_measurement_flag == 0) {
		// Search RC parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &rc_index);
		if (parser_status == PARSER_SUCCESS) {
			// Check value.
			if (((sfx_rc_enum_t) rc_index) < SFX_RC_LIST_MAX_SIZE) {
				// Search test mode number.
				parser_status =  PARSER_get_parameter(&at_ctx.parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &test_mode);
				if (parser_status == PARSER_SUCCESS) {
					// Check parameters.
					if (((sfx_test_mode_t) test_mode) <= SFX_TEST_MODE_NVM) {
						// Call test mode function wth public key.
						sfx_status = ADDON_SIGFOX_RF_PROTOCOL_API_test_mode((sfx_rc_enum_t) rc_index, (sfx_test_mode_t) test_mode);
						if (sfx_status == SFX_ERR_NONE) {
							AT_print_ok();
						}
						else {
							AT_print_error(ERROR_SOURCE_SIGFOX, sfx_status); // Error from Sigfox library.
						}
					}
					else {
						AT_print_error(ERROR_SOURCE_AT, 0); // Invalid test mode.
					}
				}
				else {
					AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in test_mode parameter.
				}
			}
			else {
				AT_print_error(ERROR_SOURCE_AT, 0); // Invalid RC.
			}
		}
		else {
			AT_print_error(ERROR_SOURCE_PARSER, parser_status); // Error in RC parameter.
		}
	}
	else {
		AT_print_error(ERROR_SOURCE_AT, AT_ERROR_BUSY);
	}
}
#endif

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void AT_decode(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	unsigned int idx = 0;
	unsigned char decode_success = 0;
	// Empty or too short command.
	if (at_ctx.command_buf_idx < AT_COMMAND_LENGTH_MIN) {
		AT_print_error(ERROR_SOURCE_PARSER, PARSER_ERROR_UNKNOWN_COMMAND);
	}
	else {
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
			AT_print_error(ERROR_SOURCE_PARSER, PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
		}
	}
	// Reset AT parser.
	AT_init();
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
	at_ctx.command_buf_idx = 0;
	at_ctx.line_end_flag = 0;
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) at_ctx.response_buf[idx] = '\0';
	at_ctx.response_buf_idx = 0;
	at_ctx.wind_measurement_flag = 0;
	// Parsing variables.
	at_ctx.parser.rx_buf = (unsigned char*) at_ctx.command_buf;
	at_ctx.parser.rx_buf_length = 0;
	at_ctx.parser.separator_idx = 0;
	at_ctx.parser.start_idx = 0;
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
