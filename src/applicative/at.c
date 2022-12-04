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
#include "pwr.h"
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
#include "types.h"
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
// Commands
#define AT_COMMAND_BUFFER_SIZE			128
// Parameters separator.
#define AT_CHAR_SEPARATOR				','
// Replies.
#define AT_REPLY_BUFFER_SIZE			128
#define AT_REPLY_END					"\r\n"
#define AT_REPLY_TAB					"     "
#define AT_STRING_VALUE_BUFFER_SIZE		16
// Duration of RSSI command.
#define AT_RSSI_REPORT_PERIOD_MS		500

/*** AT callbacks declaration ***/

static void _AT_print_ok(void);
static void _AT_print_command_list(void);
static void _AT_print_sw_version(void);
static void _AT_print_error_stack(void);
#ifdef AT_COMMANDS_SENSORS
static void _AT_adc_callback(void);
static void _AT_max11136_callback(void);
static void _AT_iths_callback(void);
static void _AT_eths_callback(void);
static void _AT_epts_callback(void);
static void _AT_euvs_callback(void);
static void _AT_wind_callback(void);
static void _AT_rain_callback(void);
#endif
#ifdef AT_COMMANDS_GPS
static void _AT_time_callback(void);
static void _AT_gps_callback(void);
#endif
#ifdef AT_COMMANDS_NVM
static void _AT_nvmr_callback(void);
static void _AT_nvm_callback(void);
static void _AT_get_id_callback(void);
static void _AT_set_id_callback(void);
static void _AT_get_key_callback(void);
static void _AT_set_key_callback(void);
#endif
#ifdef AT_COMMANDS_SIGFOX
static void _AT_so_callback(void);
static void _AT_sb_callback(void);
static void _AT_sf_callback(void);
#endif
#ifdef AT_COMMANDS_TEST_MODES
static void _AT_tm_callback(void);
static void _AT_cw_callback(void);
static void _AT_dl_callback(void);
static void _AT_rssi_callback(void);
#endif

/*** AT local structures ***/

typedef struct {
	PARSER_mode_t mode;
	char_t* syntax;
	char_t* parameters;
	char_t* description;
	void (*callback)(void);
} AT_command_t;

typedef struct {
	// Command.
	volatile char_t command[AT_COMMAND_BUFFER_SIZE];
	volatile uint32_t command_size;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
	// Replies.
	char_t reply[AT_REPLY_BUFFER_SIZE];
	uint32_t reply_size;
	// Wind measurement flag.
	uint8_t wind_measurement_flag;
	// Sigfox RC.
	sfx_rc_t sigfox_rc;
} AT_context_t;

/*** AT local global variables ***/

static const AT_command_t AT_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", STRING_NULL, "Ping command", _AT_print_ok},
	{PARSER_MODE_COMMAND, "AT?", STRING_NULL, "List all available AT commands", _AT_print_command_list},
	{PARSER_MODE_COMMAND, "AT$V?", STRING_NULL, "Get SW version", _AT_print_sw_version},
	{PARSER_MODE_COMMAND, "AT$ERROR?", STRING_NULL, "Read error stack", _AT_print_error_stack},
	{PARSER_MODE_COMMAND, "AT$RST", STRING_NULL, "Reset MCU", PWR_software_reset},
#ifdef AT_COMMANDS_SENSORS
	{PARSER_MODE_COMMAND, "AT$ADC?", STRING_NULL, "Get internal ADC measurements", _AT_adc_callback},
	{PARSER_MODE_COMMAND, "AT$MAX11136?", STRING_NULL, "Get external ADC measurements (MAX11136)", _AT_max11136_callback},
#ifdef HW2_0
	{PARSER_MODE_COMMAND, "AT$ITHS?", STRING_NULL, "Get internal PCB temperature and humidity (SHT30)", _AT_iths_callback},
#endif
	{PARSER_MODE_COMMAND, "AT$ETHS?", STRING_NULL, "Get external temperature and humidity (SHT30)", _AT_eths_callback},
	{PARSER_MODE_COMMAND, "AT$EPTS?", STRING_NULL, "Get pressure and temperature (DPS310)", _AT_epts_callback},
	{PARSER_MODE_COMMAND, "AT$EUVS?", STRING_NULL, "Get UV (SI1133)", _AT_euvs_callback},
	{PARSER_MODE_HEADER,  "AT$WIND=", "enable[bit]", "Enable or disable wind measurements", _AT_wind_callback},
	{PARSER_MODE_HEADER,  "AT$RAIN=", "enable[bit]", "Enable or disable rain detection", _AT_rain_callback},
#endif
#ifdef AT_COMMANDS_GPS
	{PARSER_MODE_HEADER,  "AT$TIME=", "timeout[s]", "Get GPS time (NEOM8N)", _AT_time_callback},
	{PARSER_MODE_HEADER,  "AT$GPS=", "timeout[s]", "Get GPS position (NEOM8N)", _AT_gps_callback},
#endif
#ifdef AT_COMMANDS_NVM
	{PARSER_MODE_COMMAND, "AT$NVMR", STRING_NULL, "Reset NVM data", _AT_nvmr_callback},
	{PARSER_MODE_HEADER,  "AT$NVM=", "address[dec]", "Get NVM data", _AT_nvm_callback},
	{PARSER_MODE_COMMAND, "AT$ID?", STRING_NULL, "Get Sigfox device ID", _AT_get_id_callback},
	{PARSER_MODE_HEADER,  "AT$ID=", "id[hex]", "Set Sigfox device ID", _AT_set_id_callback},
	{PARSER_MODE_COMMAND, "AT$KEY?", STRING_NULL, "Get Sigfox device key", _AT_get_key_callback},
	{PARSER_MODE_HEADER,  "AT$KEY=", "key[hex]", "Set Sigfox device key", _AT_set_key_callback},
#endif
#ifdef AT_COMMANDS_SIGFOX
	{PARSER_MODE_COMMAND, "AT$SO", STRING_NULL, "Sigfox send control message", _AT_so_callback},
	{PARSER_MODE_HEADER,  "AT$SB=", "data[bit],(bidir_flag[bit])", "Sigfox send bit", _AT_sb_callback},
	{PARSER_MODE_HEADER,  "AT$SF=", "data[hex],(bidir_flag[bit])", "Sigfox send frame", _AT_sf_callback},
#endif
#ifdef AT_COMMANDS_TEST_MODES
	{PARSER_MODE_HEADER,  "AT$TM=", "rc_index[dec],test_mode[dec]", "Execute Sigfox test mode", _AT_tm_callback},
	{PARSER_MODE_HEADER,  "AT$CW=", "frequency[hz],enable[bit],(output_power[dbm])", "Start or stop continuous radio transmission", _AT_cw_callback},
	{PARSER_MODE_HEADER,  "AT$DL=", "frequency[hz]", "Continuous downlink frames decoding", _AT_dl_callback},
	{PARSER_MODE_HEADER,  "AT$RSSI=", "frequency[hz],duration[s]", "Start or stop continuous RSSI measurement", _AT_rssi_callback},
#endif
};

static AT_context_t at_ctx;

/*** AT local functions ***/

/* GENERIC MACRO TO ADD A CHARACTER TO THE REPLY BUFFER.
 * @param character:	Character to add.
 * @return:				None.
 */
#define _AT_reply_add_char(character) { \
	at_ctx.reply[at_ctx.reply_size] = character; \
	at_ctx.reply_size = (at_ctx.reply_size + 1) % AT_REPLY_BUFFER_SIZE; \
}

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void _AT_reply_add_string(char_t* tx_string) {
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		_AT_reply_add_char(*(tx_string++));
	}
}

/* APPEND A VALUE TO THE REPONSE BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void _AT_reply_add_value(int32_t tx_value, STRING_format_t format, uint8_t print_prefix) {
	// Local variables.
	STRING_status_t string_status = STRING_SUCCESS;
	char_t str_value[AT_STRING_VALUE_BUFFER_SIZE];
	uint8_t idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_STRING_VALUE_BUFFER_SIZE ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	string_status = STRING_value_to_string(tx_value, format, print_prefix, str_value);
	STRING_error_check();
	// Add string.
	_AT_reply_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_reply_send(void) {
	// Local variables.
	USART_status_t usart_status = USART_SUCCESS;
	// Add ending string.
	_AT_reply_add_string(AT_REPLY_END);
	_AT_reply_add_char(STRING_CHAR_NULL);
	// Send response over UART.
	usart_status = USARTx_send_string(at_ctx.reply);
	USART_error_check();
	// Flush reply buffer.
	at_ctx.reply_size = 0;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_ok(void) {
	_AT_reply_add_string("OK");
	_AT_reply_send();
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error:	Error code to print.
 * @return:			None.
 */
static void _AT_print_error(ERROR_t error) {
	// Add error to stack.
	ERROR_stack_add(error);
	// Print error.
	_AT_reply_add_string("ERROR_");
	if (error < 0x0100) {
		_AT_reply_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 0);
	}
	else {
		_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
	}
	_AT_reply_send();
}

/* PRINT ALL SUPPORTED AT COMMANDS.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_command_list(void) {
	// Local variables.
	uint32_t idx = 0;
	// Commands loop.
	for (idx=0 ; idx<(sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)) ; idx++) {
		// Print syntax.
		_AT_reply_add_string(AT_COMMAND_LIST[idx].syntax);
		// Print parameters.
		_AT_reply_add_string(AT_COMMAND_LIST[idx].parameters);
		_AT_reply_send();
		// Print description.
		_AT_reply_add_string(AT_REPLY_TAB);
		_AT_reply_add_string(AT_COMMAND_LIST[idx].description);
		_AT_reply_send();
	}
	_AT_print_ok();
}

/* PRINT SW VERSION.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_sw_version(void) {
	_AT_reply_add_string("SW");
	_AT_reply_add_value((int32_t) GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(".");
	_AT_reply_add_value((int32_t) GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(".");
	_AT_reply_add_value((int32_t) GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
	if (GIT_DIRTY_FLAG != 0) {
		_AT_reply_add_string(".d");
	}
	_AT_reply_add_string(" (");
	_AT_reply_add_value((int32_t) GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_reply_add_string(")");
	_AT_reply_send();
	_AT_print_ok();
}

/* PRINT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_error_stack(void) {
	// Local variables.
	ERROR_t error = SUCCESS;
	// Read stack.
	if (ERROR_stack_is_empty() != 0) {
		_AT_reply_add_string("Error stack empty");
	}
	else {
		// Unstack all errors.
		_AT_reply_add_string("[ ");
		do {
			error = ERROR_stack_read();
			if (error != SUCCESS) {
				_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
				_AT_reply_add_string(" ");
			}
		}
		while (error != SUCCESS);
		_AT_reply_add_string("]");
	}
	_AT_reply_send();
	_AT_print_ok();
}

#ifdef AT_COMMANDS_SENSORS
/* AT$ADC? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_adc_callback(void) {
	// Local variables.
	ADC_status_t adc1_status = ADC_SUCCESS;
	uint32_t voltage_mv = 0;
	int8_t tmcu_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Trigger internal ADC conversions.
	_AT_reply_add_string("ADC running...");
	_AT_reply_send();
	adc1_status = ADC1_perform_measurements();
	ADC1_error_check_print();
	// Read and print data.
	// MCU voltage.
	_AT_reply_add_string("Vmcu=");
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("mV");
	_AT_reply_send();
	// MCU temperature.
	_AT_reply_add_string("Tmcu=");
	adc1_status = ADC1_get_tmcu(&tmcu_degrees);
	ADC1_error_check_print();
	_AT_reply_add_value((int32_t) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dC");
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$ADC? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_max11136_callback(void) {
	// Local variables.
	MAX11136_status_t max11136_status = MAX11136_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	uint32_t data = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
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
	_AT_reply_add_string("MAX11136 running...");
	_AT_reply_send();
	max11136_status = MAX11136_perform_measurements();
	MAX11136_error_check_print();
	// Source voltage.
	_AT_reply_add_string("Vsrc=");
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VSRC_MV, &data);
	MAX11136_error_check_print();
	_AT_reply_add_value((int32_t) data, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("mV");
	_AT_reply_send();
	// Supercap voltage.
	_AT_reply_add_string("Vcap=");
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_VCAP_MV, &data);
	MAX11136_error_check_print();
	_AT_reply_add_value((int32_t) data, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("mV");
	_AT_reply_send();
	// Light.
	_AT_reply_add_string("Light=");
	max11136_status = MAX11136_get_data(MAX11136_DATA_INDEX_LDR_PERCENT, &data);
	MAX11136_error_check_print();
	_AT_reply_add_value((int32_t) data, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("%");
	_AT_reply_send();
	_AT_print_ok();
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
static void _AT_iths_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	int8_t tamb_degrees = 0;
	uint8_t hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	I2C1_error_check_print();
	_AT_reply_add_string("SHT3X running...");
	_AT_reply_send();
	sht3x_status = SHT3X_perform_measurements(SHT3X_INT_I2C_ADDRESS);
	SHT3X_INT_error_check_print();
	// Read and print data.
	// Temperature.
	_AT_reply_add_string("Tpcb=");
	sht3x_status = SHT3X_get_temperature(&tamb_degrees);
	SHT3X_INT_error_check_print();
	_AT_reply_add_value((int32_t) tamb_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dC");
	_AT_reply_send();
	// Humidity.
	_AT_reply_add_string("Hpcb=");
	sht3x_status = SHT3X_get_humidity(&hamb_percent);
	SHT3X_INT_error_check_print();
	_AT_reply_add_value((int32_t) hamb_percent, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("%");
	_AT_reply_send();
	_AT_print_ok();
errors:
	I2C1_power_off();
	return;
}

/* AT$ETHS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_eths_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	int8_t tamb_degrees = 0;
	uint8_t hamb_percent = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	I2C1_error_check_print();
	_AT_reply_add_string("SHT3X running...");
	_AT_reply_send();
	sht3x_status = SHT3X_perform_measurements(SHT3X_EXT_I2C_ADDRESS);
	SHT3X_EXT_error_check_print();
	// Read and print data.
	// Temperature.
	_AT_reply_add_string("Tamb=");
	sht3x_status = SHT3X_get_temperature(&tamb_degrees);
	SHT3X_EXT_error_check_print();
	_AT_reply_add_value((int32_t) tamb_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dC");
	_AT_reply_send();
	// Humidity.
	_AT_reply_add_string("Hamb=");
	sht3x_status = SHT3X_get_humidity(&hamb_percent);
	SHT3X_EXT_error_check_print();
	_AT_reply_add_value(hamb_percent, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("%");
	_AT_reply_send();
	_AT_print_ok();
errors:
	I2C1_power_off();
	return;
}

/* AT$EPTS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_epts_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	DPS310_status_t dps310_status = DPS310_SUCCESS;
	uint32_t pressure_pa = 0;
	int8_t tamb_degrees = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	I2C1_error_check_print();
	_AT_reply_add_string("DPS310 running...");
	_AT_reply_send();
	dps310_status = DPS310_perform_measurements(DPS310_EXTERNAL_I2C_ADDRESS);
	DPS310_error_check_print();
	// Read and print data.
	// Pressure.
	_AT_reply_add_string("Pabs=");
	dps310_status = DPS310_get_pressure(&pressure_pa);
	DPS310_error_check_print();
	_AT_reply_add_value((int32_t) pressure_pa, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("Pa T=");
	_AT_reply_send();
	// Temperature.
	_AT_reply_add_string("Tamb=");
	dps310_status = DPS310_get_temperature(&tamb_degrees);
	DPS310_error_check_print();
	_AT_reply_add_value((int32_t) tamb_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dC");
	_AT_reply_send();
	_AT_print_ok();
errors:
	I2C1_power_off();
	return;
}

/* AT$EUVS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_euvs_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SI1133_status_t si1133_status = SI1133_SUCCESS;
	uint8_t uv_index = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	I2C1_error_check_print();
	_AT_reply_add_string("SI1133 running...");
	_AT_reply_send();
	si1133_status = SI1133_perform_measurements(SI1133_EXTERNAL_I2C_ADDRESS);
	SI1133_error_check_print();
	// Read and print data.
	_AT_reply_add_string("UVI=");
	si1133_status = SI1133_get_uv_index(&uv_index);
	SI1133_error_check_print();
	_AT_reply_add_value(uv_index, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_send();
	_AT_print_ok();
errors:
	I2C1_power_off();
	return;
}

/* AT$WIND EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_wind_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_SUCCESS;
	WIND_status_t wind_status = WIND_SUCCESS;
	int32_t enable = 0;
	uint32_t wind_speed_average = 0;
	uint32_t wind_speed_peak = 0;
	uint32_t wind_direction = 0;
	// Read enable parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &enable);
	PARSER_error_check_print();
	// Start or stop wind continuous measurements.
	if (enable == 0) {
		WIND_stop_continuous_measure();
		// Update flag.
		at_ctx.wind_measurement_flag = 0;
		// Get speeds.
		wind_status = WIND_get_speed(&wind_speed_average, &wind_speed_peak);
		WIND_error_check_print();
		_AT_reply_add_string("wspd_avrg=");
		_AT_reply_add_value((int32_t) wind_speed_average, STRING_FORMAT_DECIMAL, 0);
		_AT_reply_add_string("m/h=");
		_AT_reply_send();
		_AT_reply_add_string("wpsd_peak=");
		_AT_reply_add_value((int32_t) wind_speed_peak, STRING_FORMAT_DECIMAL, 0);
		_AT_reply_add_string("m/h=");
		_AT_reply_send();
		// Get direction.
		_AT_reply_add_string("wdir_avrg=");
		wind_status = WIND_get_direction(&wind_direction);
		if (wind_status == (WIND_ERROR_BASE_MATH + MATH_ERROR_UNDEFINED)) {
			_AT_reply_add_string("N/A");
		}
		else {
			WIND_error_check_print();
			_AT_reply_add_value((int32_t) wind_direction, STRING_FORMAT_DECIMAL, 0);
			_AT_reply_add_string("d");
		}
		_AT_reply_send();
		// Reset data.
		WIND_reset_data();
	}
	else {
		WIND_start_continuous_measure();
		// Update flag.
		at_ctx.wind_measurement_flag = 1;
	}
	_AT_print_ok();
errors:
	return;
}

/* AT$RAIN EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_rain_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	RAIN_status_t rain_status = RAIN_SUCCESS;
	int32_t enable = 0;
	uint8_t rain_mm = 0;
	// Read enable parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &enable);
	PARSER_error_check_print();
	// Start or stop rain continuous measurements.
	if (enable == 0) {
		RAIN_stop_continuous_measure();
		// Read and print data.
		_AT_reply_add_string("rain=");
		rain_status = RAIN_get_pluviometry(&rain_mm);
		RAIN_error_check_print();
		_AT_reply_add_value((int32_t) rain_mm, STRING_FORMAT_DECIMAL,0);
		_AT_reply_add_string("mm");
		_AT_reply_send();
		// Reset data.
		RAIN_reset_data();
	}
	else {
		RAIN_start_continuous_measure();
	}
	_AT_print_ok();
errors:
	return;
}
#endif

#ifdef AT_COMMANDS_GPS
/* AT$TIME EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_time_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	int32_t timeout_seconds = 0;
	RTC_time_t gps_time;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &timeout_seconds);
	PARSER_error_check_print();
	// Power on GPS.
	lpuart1_status = LPUART1_power_on();
	LPUART1_error_check_print();
	// Start time aquisition.
	_AT_reply_add_string("GPS running...");
	_AT_reply_send();
	neom8n_status = NEOM8N_get_time(&gps_time, (uint32_t) timeout_seconds);
	NEOM8N_error_check_print();
	// Year.
	_AT_reply_add_value((gps_time.year), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("-");
	// Month.
	if ((gps_time.month) < 10) {
		_AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	_AT_reply_add_value((gps_time.month), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("-");
	// Day.
	if ((gps_time.date) < 10) {
		_AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	_AT_reply_add_value((gps_time.date), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(" ");
	// Hours.
	if ((gps_time.hours) < 10) {
		_AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	_AT_reply_add_value((gps_time.hours), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(":");
	// Minutes.
	if ((gps_time.minutes) < 10) {
		_AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	_AT_reply_add_value((gps_time.minutes), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(":");
	// Seconds.
	if ((gps_time.seconds) < 10) {
		_AT_reply_add_value(0, STRING_FORMAT_DECIMAL, 0);
	}
	_AT_reply_add_value((gps_time.seconds), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_send();
	_AT_print_ok();
errors:
	LPUART1_power_off();
	return;
}

/* AT$GPS EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_gps_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	int32_t timeout_seconds = 0;
	uint32_t fix_duration_seconds = 0;
	NEOM8N_position_t gps_position;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &timeout_seconds);
	PARSER_error_check_print();
	// Power on GPS.
	lpuart1_status = LPUART1_power_on();
	LPUART1_error_check_print();
	// Start GPS fix.
	_AT_reply_add_string("GPS running...");
	_AT_reply_send();
	neom8n_status = NEOM8N_get_position(&gps_position, (uint32_t) timeout_seconds, &fix_duration_seconds);
	NEOM8N_error_check_print();
	// Latitude.
	_AT_reply_add_string("Lat=");
	_AT_reply_add_value((gps_position.lat_degrees), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("d");
	_AT_reply_add_value((gps_position.lat_minutes), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("'");
	_AT_reply_add_value((gps_position.lat_seconds), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("''");
	_AT_reply_add_string(((gps_position.lat_north_flag) == 0) ? "S" : "N");
	// Longitude.
	_AT_reply_add_string(" Long=");
	_AT_reply_add_value((gps_position.long_degrees), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("d");
	_AT_reply_add_value((gps_position.long_minutes), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("'");
	_AT_reply_add_value((gps_position.long_seconds), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("''");
	_AT_reply_add_string(((gps_position.long_east_flag) == 0) ? "W" : "E");
	// Altitude.
	_AT_reply_add_string(" Alt=");
	_AT_reply_add_value((gps_position.altitude), STRING_FORMAT_DECIMAL, 0);
	// Fix duration.
	_AT_reply_add_string("m Fix=");
	_AT_reply_add_value(fix_duration_seconds, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("s");
	_AT_reply_send();
	_AT_print_ok();
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
static void _AT_nvmr_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Reset all NVM field to default value.
	nvm_status = NVM_reset_default();
	NVM_error_check_print();
	_AT_print_ok();
errors:
	return;
}

/* AT$NVM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_nvm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	int32_t address = 0;
	uint8_t nvm_data = 0;
	// Read address parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &address);
	PARSER_error_check_print();
	// Read byte at requested address.
	nvm_status = NVM_read_byte((uint16_t) address, &nvm_data);
	NVM_error_check_print();
	// Print data.
	_AT_reply_add_value(nvm_data, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$ID? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_get_id_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t idx = 0;
	uint8_t id_byte = 0;
	// Retrieve device ID in NVM.
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), &id_byte);
		NVM_error_check_print();
		_AT_reply_add_value(id_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$ID EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_set_id_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t device_id[ID_LENGTH];
	uint8_t extracted_length = 0;
	uint8_t idx = 0;
	// Read ID parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, ID_LENGTH, 1, device_id, &extracted_length);
	PARSER_error_check_print();
	// Write device ID in NVM.
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), device_id[idx]);
		NVM_error_check_print();
	}
	_AT_print_ok();
errors:
	return;
}

/* AT$KEY? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_get_key_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t idx = 0;
	uint8_t key_byte = 0;
	// Retrieve device key in NVM.
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), &key_byte);
		NVM_error_check_print();
		_AT_reply_add_value(key_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$KEY EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_set_key_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t device_key[AES_BLOCK_SIZE];
	uint8_t extracted_length = 0;
	uint8_t idx = 0;
	// Read key parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, AES_BLOCK_SIZE, 1, device_key, &extracted_length);
	PARSER_error_check_print();
	// Write device ID in NVM.
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), device_key[idx]);
		NVM_error_check_print();
	}
	_AT_print_ok();
errors:
	return;
}
#endif

#ifdef AT_COMMANDS_SIGFOX
/* PRINT SIGFOX DOWNLINK DATA ON AT INTERFACE.
 * @param dl_payload:	Downlink data to print.
 * @return:				None.
 */
static void _AT_print_dl_payload(sfx_u8* dl_payload) {
	_AT_reply_add_string("+RX=");
	uint8_t idx = 0;
	for (idx=0 ; idx<SIGFOX_DOWNLINK_DATA_SIZE_BYTES ; idx++) {
		_AT_reply_add_value(dl_payload[idx], STRING_FORMAT_HEXADECIMAL, 0);
	}
	_AT_reply_send();
}

/* AT$SO EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_so_callback(void) {
	// Local variables.
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Send Sigfox OOB frame.
	sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
	SIGFOX_API_error_check_print();
	_AT_reply_add_string("Sigfox library running...");
	_AT_reply_send();
	sigfox_api_status = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
	SIGFOX_API_error_check_print();
	_AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}

/* AT$SB EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_sb_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t data = 0;
	int32_t bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// First try with 2 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, &data);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
		PARSER_error_check_print();
		// Send Sigfox bit with specified downlink request.
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, (sfx_bool) bidir_flag);
		SIGFOX_API_error_check_print();
		if (bidir_flag != SFX_FALSE) {
			_AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &data);
		PARSER_error_check_print();
		// Send Sigfox bit with no downlink request (by default).
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, 0);
		SIGFOX_API_error_check_print();
	}
	_AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}

/* AT$SF EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_sf_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	sfx_u8 data[SIGFOX_UPLINK_DATA_MAX_SIZE_BYTES];
	uint8_t extracted_length = 0;
	int32_t bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// First try with 2 parameters.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 12, 0, data, &extracted_length);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
		PARSER_error_check_print();
		// Send Sigfox frame with specified downlink request.
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, bidir_flag);
		SIGFOX_API_error_check_print();
		if (bidir_flag != 0) {
			_AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, 12, 0, data, &extracted_length);
		PARSER_error_check_print();
		// Send Sigfox frame with no downlink request (by default).
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, 0);
		SIGFOX_API_error_check_print();
	}
	_AT_print_ok();
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
static void _AT_print_dl_phy_content(sfx_u8* dl_phy_content, int32_t rssi_dbm) {
	_AT_reply_add_string("+DL_PHY=");
	uint8_t idx = 0;
	for (idx=0 ; idx<SIGFOX_DOWNLINK_PHY_SIZE_BYTES ; idx++) {
		_AT_reply_add_value(dl_phy_content[idx], STRING_FORMAT_HEXADECIMAL, 0);
	}
	_AT_reply_add_string(" RSSI=");
	_AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dBm");
	_AT_reply_send();
}

/* AT$TM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_tm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t rc_index = 0;
	int32_t test_mode = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Read RC parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &rc_index);
	PARSER_error_check_print();
	// Read test mode parameter.
	parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &test_mode);
	PARSER_error_check_print();
	// Call test mode function wth public key.
	_AT_reply_add_string("Sigfox addon running...");
	_AT_reply_send();
	sigfox_api_status = ADDON_SIGFOX_RF_PROTOCOL_API_test_mode((sfx_rc_enum_t) rc_index, (sfx_test_mode_t) test_mode);
	SIGFOX_API_error_check_print();
	_AT_print_ok();
errors:
	return;
}

/* AT$CW EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_cw_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t enable = 0;
	int32_t frequency_hz = 0;
	int32_t power_dbm = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &frequency_hz);
	PARSER_error_check_print();
	// First try with 3 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, &enable);
	if (parser_status == PARSER_SUCCESS) {
		// There is a third parameter, try to parse power.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &power_dbm);
		PARSER_error_check_print();
		// CW with given output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sigfox_api_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			SIGFOX_API_error_check_print();
			sx1232_status = SX1232_set_rf_output_power((uint8_t) power_dbm);
			SX1232_error_check_print();
			_AT_reply_add_string("SX1232 running...");
			_AT_reply_send();
		}
	}
	else {
		// Power is not given, try to parse enable as last parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &enable);
		PARSER_error_check_print();
		// CW with last output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sigfox_api_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			SIGFOX_API_error_check_print();
			_AT_reply_add_string("SX1232 running...");
			_AT_reply_send();
		}
	}
	_AT_print_ok();
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
static void _AT_dl_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	sfx_u8 dl_phy_content[SIGFOX_DOWNLINK_PHY_SIZE_BYTES];
	sfx_s16 rssi_dbm = 0;
	sfx_rx_state_enum_t dl_status = DL_PASSED;
	int32_t frequency_hz = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &frequency_hz);
	PARSER_error_check_print();
	// Start radio.
	sigfox_api_status = RF_API_init(SFX_RF_MODE_RX);
	SIGFOX_API_error_check_print();
	sigfox_api_status = RF_API_change_frequency(frequency_hz);
	SIGFOX_API_error_check_print();
	_AT_reply_add_string("RX GFSK running...");
	_AT_reply_send();
	while (dl_status == DL_PASSED) {
		sigfox_api_status = RF_API_wait_frame(dl_phy_content, &rssi_dbm, &dl_status);
		SIGFOX_API_error_check_print();
		// Check result.
		if (dl_status == DL_PASSED) {
			_AT_print_dl_phy_content(dl_phy_content, rssi_dbm);
		}
		else {
			_AT_reply_add_string("RX timeout");
			_AT_reply_send();
		}
	}
	_AT_print_ok();
errors:
	sigfox_api_status = RF_API_stop();
	SIGFOX_API_error_check();
	return;
}

/* AT$RSSI EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_rssi_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	SX1232_status_t sx1232_status = SX1232_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t frequency_hz = 0;
	int32_t duration_s = 0;
	int16_t rssi_dbm = 0;
	uint32_t report_loop = 0;
	// Check if wind measurement is not running.
	if (at_ctx.wind_measurement_flag != 0) {
		_AT_print_error(ERROR_BUSY);
		goto errors;
	}
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &frequency_hz);
	PARSER_error_check_print();
	// Read duration parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &duration_s);
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
	lptim1_status = LPTIM1_delay_milliseconds(5, 0);
	LPTIM1_error_check_print();
	sx1232_status = SX1232_set_mode(SX1232_MODE_RX);
	SX1232_error_check_print();
	// Wait TS_TR=120us typical.
	lptim1_status = LPTIM1_delay_milliseconds(5, 0);
	LPTIM1_error_check_print();
	// Measurement loop.
	_AT_reply_add_string("SX1232 running...");
	_AT_reply_send();
	while (report_loop < ((duration_s * 1000) / AT_RSSI_REPORT_PERIOD_MS)) {
		// Read RSSI.
		sx1232_status = SX1232_get_rssi(&rssi_dbm);
		SX1232_error_check_print();
		// Print RSSI.
		_AT_reply_add_string("RSSI=");
		_AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		_AT_reply_add_string("dBm");
		_AT_reply_send();
		// Report delay.
		lptim1_status = LPTIM1_delay_milliseconds(AT_RSSI_REPORT_PERIOD_MS, 0);
		LPTIM1_error_check_print();
		report_loop++;
	}
	_AT_print_ok();
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
static void _AT_reset_parser(void) {
	// Flush buffers.
	at_ctx.command_size = 0;
	at_ctx.reply_size = 0;
	// Reset flag.
	at_ctx.line_end_flag = 0;
	// Reset parser.
	at_ctx.parser.buffer = (char_t*) at_ctx.command;
	at_ctx.parser.buffer_size = 0;
	at_ctx.parser.separator_idx = 0;
	at_ctx.parser.start_idx = 0;
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_decode(void) {
	// Local variables.
	uint8_t idx = 0;
	uint8_t decode_success = 0;
	// Update parser length.
	at_ctx.parser.buffer_size = at_ctx.command_size;
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
		_AT_print_error(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
		goto errors;
	}
errors:
	_AT_reset_parser();
	return;
}

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_init(void) {
	// Init context.
	_AT_reset_parser();
	at_ctx.wind_measurement_flag = 0;
	at_ctx.sigfox_rc = (sfx_rc_t) RC1;
	// Enable USART.
	USARTx_enable_interrupt();
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_task(void) {
	// Trigger decoding function if line end found.
	if (at_ctx.line_end_flag != 0) {
		// Decode and execute command.
		USARTx_disable_interrupt();
		_AT_decode();
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
void AT_fill_rx_buffer(uint8_t rx_byte) {
	// Append byte if line end flag is not allready set.
	if (at_ctx.line_end_flag == 0) {
		// Check ending characters.
		if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
			at_ctx.command[at_ctx.command_size] = STRING_CHAR_NULL;
			at_ctx.line_end_flag = 1;
		}
		else {
			// Store new byte.
			at_ctx.command[at_ctx.command_size] = rx_byte;
			// Manage index.
			at_ctx.command_size = (at_ctx.command_size + 1) % AT_COMMAND_BUFFER_SIZE;
		}
	}
}

/* PRINT SIGFOX LIBRARY RESULT.
 * @param test_result:	Test result.
 * @param rssi:			Downlink signal rssi in dBm.
 */
void AT_print_test_result(uint8_t test_result, int32_t rssi_dbm) {
	// Check result.
	if (test_result == 0) {
		_AT_reply_add_string("Test failed.");
	}
	else {
		_AT_reply_add_string("Test passed. RSSI=");
		_AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		_AT_reply_add_string("dBm");
	}
	_AT_reply_send();
}

/* PRINT PLUVIOMETRY DATA.
 * @param rain_edge_count:	Rain gauge edge counter.
 * @return:					None.
 */
void AT_print_rain(uint8_t rain_edge_count) {
	// Print data.
	_AT_reply_add_string("Rain_edge_count=");
	_AT_reply_add_value((int32_t) rain_edge_count, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_send();
}

/* PRINT WIND SPEED.
 * @param speed_mh:	Wind speed in m/h.
 * @return:					None.
 */
void AT_print_wind_speed(uint32_t speed_mh) {
	// Print data.
	_AT_reply_add_string("Wind_speed=");
	_AT_reply_add_value((int32_t) speed_mh, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("m/h");
	_AT_reply_send();
}

/* PRINT WIND DIRECTION.
 * @param direction_degrees:	Wind direction in degrees.
 * @param direction_x:			Current direction vector x value.
 * @param direction_y:			Current direction vector y value.
 * @return:							None.
 */
void AT_print_wind_direction(uint32_t direction_degrees, int32_t direction_x, int32_t direction_y) {
	// Print data.
	_AT_reply_add_string("Wind_direction=");
	_AT_reply_add_value((int32_t) direction_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("d x=");
	_AT_reply_add_value(direction_x, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(" y=");
	_AT_reply_add_value(direction_y, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_send();
}

#endif
