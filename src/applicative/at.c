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
#define AT_COMMANDS_GPS
#define AT_COMMANDS_SENSORS
#define AT_COMMANDS_NVM
#define AT_COMMANDS_SIGFOX
#define AT_COMMANDS_CW_RSSI
#define AT_COMMANDS_TEST_MODES
#define AT_COMMANDS_RC
// Common macros.
#define AT_COMMAND_LENGTH_MIN			2
#define AT_COMMAND_BUFFER_LENGTH		128
#define AT_RESPONSE_BUFFER_LENGTH		128
#define AT_STRING_VALUE_BUFFER_LENGTH	16
// Input commands without parameter.
#define AT_COMMAND_TEST					"AT"
#define AT_COMMAND_ADC					"AT$ADC?"
#define AT_COMMAND_MCU					"AT$MCU?"
#ifdef HW2_0
#define AT_COMMAND_ITHS					"AT$ITHS?"
#endif
#define AT_COMMAND_ETHS					"AT$ETHS?"
#define AT_COMMAND_EPTS					"AT$EPTS?"
#define AT_COMMAND_ELDR					"AT$ELDR?"
#define AT_COMMAND_EUVS					"AT$EUVS?"
#define AT_COMMAND_ID					"AT$ID?"
#define AT_COMMAND_KEY					"AT$KEY?"
#define AT_COMMAND_NVMR					"AT$NVMR"
#define AT_COMMAND_OOB					"AT$SO"
#define AT_COMMAND_RC					"AT$RC?"
// Input commands with parameters (headers).
#define AT_HEADER_TIME					"AT$TIME="
#define AT_HEADER_GPS					"AT$GPS="
#define AT_HEADER_WIND					"AT$WIND="
#define AT_HEADER_RAIN					"AT$RAIN="
#define AT_HEADER_NVM					"AT$NVM="
#define AT_HEADER_ID					"AT$ID="
#define AT_HEADER_KEY					"AT$KEY="
#define AT_HEADER_SF					"AT$SF="
#define AT_HEADER_SB					"AT$SB="
#define AT_HEADER_CW					"AT$CW="
#define AT_HEADER_RSSI					"AT$RSSI="
#define AT_HEADER_TM					"AT$TM="
#define AT_HEADER_RC					"AT$RC="
// Parameters separator.
#define AT_CHAR_SEPARATOR				','
// Responses.
#define AT_RESPONSE_OK					"OK"
#define AT_RESPONSE_END					"\r\n"
#define AT_RESPONSE_ERROR_PSR			"PSR_ERROR_"
#define AT_RESPONSE_ERROR_SFX			"SFX_ERROR_"
#define AT_RESPONSE_ERROR_APP			"APP_ERROR_"
// Duration of RSSI command.
#define AT_RSSI_REPORT_PERIOD_MS		500
#define AT_RSSI_REPORT_DURATION_MS		60000
// Sigfox RC standard config size.
#define AT_SIGFOX_RC_STD_CONFIG_SIZE	3

/*** AT local structures ***/

typedef enum {
	AT_ERROR_SOURCE_PSR,
	AT_ERROR_SOURCE_SFX,
	AT_ERROR_SOURCE_APP
} AT_ErrorSource;

typedef enum {
	APP_ERROR_NVM_ADDRESS_OVERFLOW,
	APP_ERROR_RF_FREQUENCY_UNDERFLOW,
	APP_ERROR_RF_FREQUENCY_OVERFLOW,
	APP_ERROR_RF_OUTPUT_POWER_OVERFLOW,
	APP_ERROR_INVALID_RC,
	APP_ERROR_INVALID_TEST_MODE,
	APP_ERROR_GPS_INVALID_TIMEOUT,
	APP_ERROR_GPS_TIMEOUT,
	APP_ERROR_FORBIDDEN_COMMAND
} AT_ApplicativeError;

typedef struct {
	// AT command buffer.
	volatile unsigned char at_command_buf[AT_COMMAND_BUFFER_LENGTH];
	volatile unsigned int at_command_buf_idx;
	volatile unsigned char at_line_end_flag;
	PARSER_Context at_parser;
	char at_response_buf[AT_RESPONSE_BUFFER_LENGTH];
	unsigned int at_response_buf_idx;
	// Wind measurement flag.
	unsigned char wind_measurement_flag;
	// Sigfox RC.
	sfx_rc_t sigfox_rc;
	sfx_u32 sigfox_rc_std_config[AT_SIGFOX_RC_STD_CONFIG_SIZE];
	unsigned char sigfox_rc_idx;
} AT_Context;

/*** AT local global variables ***/

static AT_Context at_ctx;
#ifdef AT_COMMANDS_RC
static const sfx_u32 rc2_sm_config[AT_SIGFOX_RC_STD_CONFIG_SIZE] = RC2_SM_CONFIG;
static const sfx_u32 rc4_sm_config[AT_SIGFOX_RC_STD_CONFIG_SIZE] = RC4_SM_CONFIG;
static const sfx_u32 rc3a_config[AT_SIGFOX_RC_STD_CONFIG_SIZE] = RC3A_CONFIG;
static const sfx_u32 rc3c_config[AT_SIGFOX_RC_STD_CONFIG_SIZE] = RC3C_CONFIG;
static const sfx_u32 rc5_config[AT_SIGFOX_RC_STD_CONFIG_SIZE] = RC5_CONFIG;
#endif

/*** AT local functions ***/

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void AT_ResponseAddString(char* tx_string) {
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		at_ctx.at_response_buf[at_ctx.at_response_buf_idx++] = *(tx_string++);
		// Manage rollover.
		if (at_ctx.at_response_buf_idx >= AT_RESPONSE_BUFFER_LENGTH) {
			at_ctx.at_response_buf_idx = 0;
		}
	}
}

/* APPEND A VALUE TO THE REPONSE BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void AT_ResponseAddValue(int tx_value, STRING_Format format, unsigned char print_prefix) {
	// Local variables.
	char str_value[AT_STRING_VALUE_BUFFER_LENGTH];
	unsigned char idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_STRING_VALUE_BUFFER_LENGTH ; idx++) str_value[idx] = '\0';
	// Convert value to string.
	STRING_ConvertValue(tx_value, format, print_prefix, str_value);
	// Add string.
	AT_ResponseAddString(str_value);
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void AT_ReplyOk(void) {
	AT_ResponseAddString(AT_RESPONSE_OK);
	AT_ResponseAddString(AT_RESPONSE_END);
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error_code:	Error code to display.
 * @return:				None.
 */
static void AT_ReplyError(AT_ErrorSource error_source, unsigned int error_code) {
	switch (error_source) {
	case AT_ERROR_SOURCE_PSR:
		AT_ResponseAddString(AT_RESPONSE_ERROR_PSR);
		break;
	case AT_ERROR_SOURCE_SFX:
		AT_ResponseAddString(AT_RESPONSE_ERROR_SFX);
		break;
	case AT_ERROR_SOURCE_APP:
		AT_ResponseAddString(AT_RESPONSE_ERROR_APP);
		break;
	default:
		break;
	}
	AT_ResponseAddValue(error_code, STRING_FORMAT_HEXADECIMAL, 1);
	AT_ResponseAddString(AT_RESPONSE_END);
}

/* PRINT ADC RESULTS.
 * @param:	None.
 * @return:	None.
 */
static void AT_PrintAdcData(void) {
	unsigned int result_12bits = 0;
	// AIN0.
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN0, &result_12bits);
	AT_ResponseAddString("AIN0=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	// AIN1.
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN1, &result_12bits);
	AT_ResponseAddString(" AIN1=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	// AIN2.
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN2, &result_12bits);
	AT_ResponseAddString(" AIN2=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	// AIN3.
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN3, &result_12bits);
	AT_ResponseAddString(" AIN3=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	// AIN4 (resistor divider with 6.8M and 1M -> Vin = 7.8 * Vout).
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN4, &result_12bits);
	AT_ResponseAddString(" AIN4=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	// AIN5.
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN5, &result_12bits);
	AT_ResponseAddString(" AIN5=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	// AIN6.
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN6, &result_12bits);
	AT_ResponseAddString(" AIN6=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	// AIN7.
	MAX11136_GetChannel(MAX11136_CHANNEL_AIN7, &result_12bits);
	AT_ResponseAddString(" AIN7=");
	AT_ResponseAddValue(result_12bits, STRING_FORMAT_HEXADECIMAL, 0);
	AT_ResponseAddString(AT_RESPONSE_END);
}

/* PRINT SIGFOX DOWNLINK DATA ON AT INTERFACE.
 * @param sfx_downlink_data:	Downlink data to print.
 * @return:						None.
 */
static void AT_PrintDownlinkData(sfx_u8* sfx_downlink_data) {
	AT_ResponseAddString("+RX=");
	unsigned char idx = 0;
	for (idx=0 ; idx<8 ; idx++) {
		AT_ResponseAddValue(sfx_downlink_data[idx], STRING_FORMAT_HEXADECIMAL, 0);
	}
	AT_ResponseAddString(AT_RESPONSE_END);
}

/* PRINT A TIMESTAMP ON USART.
 * @param timestamp_to_print:	Pointer to the timestamp to print.
 * @return:						None.
 */
static void AT_PrintTimestamp(Timestamp* timestamp_to_print) {
	// Year.
	AT_ResponseAddValue((timestamp_to_print -> year), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("-");
	// Month.
	if ((timestamp_to_print -> month) < 10) {
		AT_ResponseAddValue(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_ResponseAddValue((timestamp_to_print -> month), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("-");
	// Day.
	if ((timestamp_to_print -> date) < 10) {
		AT_ResponseAddValue(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_ResponseAddValue((timestamp_to_print -> date), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString(" ");
	// Hours.
	if ((timestamp_to_print -> hours) < 10) {
		AT_ResponseAddValue(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_ResponseAddValue((timestamp_to_print -> hours), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString(":");
	// Minutes.
	if ((timestamp_to_print -> minutes) < 10) {
		AT_ResponseAddValue(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_ResponseAddValue((timestamp_to_print -> minutes), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString(":");
	// Seconds.
	if ((timestamp_to_print -> seconds) < 10) {
		AT_ResponseAddValue(0, STRING_FORMAT_DECIMAL, 0);
	}
	AT_ResponseAddValue((timestamp_to_print -> seconds), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString(AT_RESPONSE_END);
}

/* PRINT GPS POSITION ON USART.
 * @param gps_position:	Pointer to GPS position to print.
 * @return:				None.
 */
static void AT_PrintPosition(Position* gps_position, unsigned int gps_fix_duration) {
	// Latitude.
	AT_ResponseAddString("Lat=");
	AT_ResponseAddValue((gps_position -> lat_degrees), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("d");
	AT_ResponseAddValue((gps_position -> lat_minutes), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("'");
	AT_ResponseAddValue((gps_position -> lat_seconds), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("''-");
	AT_ResponseAddString(((gps_position -> lat_north_flag) == 0) ? "S" : "N");
	// Longitude.
	AT_ResponseAddString(" Long=");
	AT_ResponseAddValue((gps_position -> long_degrees), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("d");
	AT_ResponseAddValue((gps_position -> long_minutes), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("'");
	AT_ResponseAddValue((gps_position -> long_seconds), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("''-");
	AT_ResponseAddString(((gps_position -> long_east_flag) == 0) ? "W" : "E");
	// Altitude.
	AT_ResponseAddString(" Alt=");
	AT_ResponseAddValue((gps_position -> altitude), STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("m Fix=");
	AT_ResponseAddValue(gps_fix_duration, STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("s");
	AT_ResponseAddString(AT_RESPONSE_END);
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void AT_DecodeRxBuffer(void) {
	// Local variables.
	PARSER_Status parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	int generic_int_1 = 0;
	int generic_int_2 = 0;
	int generic_int_3 = 0;
	unsigned int generic_uint_1 = 0;
	unsigned int generic_uint_2 = 0;
	unsigned char generic_uchar = 0;
	unsigned char generic_byte_array_1[AES_BLOCK_SIZE];
	unsigned char idx = 0;
	unsigned char extracted_length = 0;
#ifdef AT_COMMANDS_GPS
	Position gps_position;
	Timestamp gps_timestamp;
	NEOM8N_ReturnCode gps_status;
#endif
#ifdef AT_COMMANDS_SENSORS
	signed char generic_signed_byte = 0;
#endif
#ifdef AT_COMMANDS_SIGFOX
	sfx_error_t sfx_error = 0;
	unsigned char generic_byte_array_2[AES_BLOCK_SIZE];
#endif
	// Empty or too short command.
	if (at_ctx.at_command_buf_idx < AT_COMMAND_LENGTH_MIN) {
		AT_ReplyError(AT_ERROR_SOURCE_PSR, PARSER_ERROR_UNKNOWN_COMMAND);
	}
	else {
		// Update parser length.
		at_ctx.at_parser.rx_buf_length = (at_ctx.at_command_buf_idx - 1); // To ignore line end.
		// Test command AT<CR>.
		if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_TEST) == PARSER_SUCCESS) {
			// Nothing to do, only reply OK to acknowledge serial link.
			AT_ReplyOk();
		}
#ifdef AT_COMMANDS_GPS
		// TIME command AT$TIME=<timeout_seconds><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_TIME) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Search timeout parameter.
				parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &generic_int_1);
				if (parser_status == PARSER_SUCCESS) {
					// Start GPS fix.
					LPUART1_PowerOn();
					gps_status = NEOM8N_GetTimestamp(&gps_timestamp, generic_int_1, 0);
					LPUART1_PowerOff();
					switch (gps_status) {
					case NEOM8N_SUCCESS:
						AT_PrintTimestamp(&gps_timestamp);
						break;
					case NEOM8N_TIMEOUT:
						AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_GPS_TIMEOUT);
						break;
					default:
						break;
					}
				}
				else {
					// Error in timeout parameter.
					AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// GPS command AT$GPS=<timeout_seconds><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_GPS) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Search timeout parameter.
				parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &generic_int_1);
				if (parser_status == PARSER_SUCCESS) {
					// Start GPS fix.
					LPUART1_PowerOn();
					gps_status = NEOM8N_GetPosition(&gps_position, generic_int_1, 0, &generic_int_2);
					LPUART1_PowerOff();
					switch (gps_status) {
					case NEOM8N_SUCCESS:
						AT_PrintPosition(&gps_position, generic_int_2);
						break;
					case NEOM8N_TIMEOUT:
						AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_GPS_TIMEOUT);
						break;
					default:
						break;
					}
				}
				else {
					// Error in timeout parameter.
					AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
#endif
#ifdef AT_COMMANDS_SENSORS
		// ADC command AT$ADC?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_ADC) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Trigger external ADC convertions.
#ifdef HW1_0
				SPI1_PowerOn();
#endif
#ifdef HW2_0
				SPI2_PowerOn();
#endif
				// Run external ADC conversions.
				MAX11136_PerformMeasurements();
#ifdef HW1_0
				SPI1_PowerOff();
#endif
#ifdef HW2_0
				SPI2_PowerOff();
#endif
				// Print results.
				AT_PrintAdcData();
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// MCU command AT$MCU?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_MCU) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Trigger internal ADC conversions.
				ADC1_PerformMeasurements();
				ADC1_GetTmcuComp2(&generic_signed_byte);
				ADC1_GetData(ADC_DATA_IDX_VMCU_MV, &generic_int_1);
				// Print results.
				AT_ResponseAddString("Vcc=");
				AT_ResponseAddValue(generic_int_1, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("mV T=");
				AT_ResponseAddValue((int) generic_signed_byte, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("dC");
				AT_ResponseAddString(AT_RESPONSE_END);
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
#ifdef HW2_0
		// Internal temperature and humidity sensor command AT$ITHS?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_ITHS) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Perform measurements.
				I2C1_PowerOn();
				SHT3X_PerformMeasurements(SHT3X_INTERNAL_I2C_ADDRESS);
				I2C1_PowerOff();
				SHT3X_GetTemperatureComp2(&generic_signed_byte);
				SHT3X_GetHumidity(&generic_uchar);
				// Print results.
				AT_ResponseAddString("T=");
				AT_ResponseAddValue((int) generic_signed_byte, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("dC H=");
				AT_ResponseAddValue(generic_uchar, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("%");
				AT_ResponseAddString(AT_RESPONSE_END);
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
#endif
		// External temperature and humidity sensor command AT$ETHS?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_ETHS) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Perform measurements.
				I2C1_PowerOn();
				SHT3X_PerformMeasurements(SHT3X_EXTERNAL_I2C_ADDRESS);
				I2C1_PowerOff();
				SHT3X_GetTemperatureComp2(&generic_signed_byte);
				SHT3X_GetHumidity(&generic_uchar);
				// Print results.
				AT_ResponseAddString("T=");
				AT_ResponseAddValue((int) generic_signed_byte, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("dC H=");
				AT_ResponseAddValue(generic_uchar, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("%");
				AT_ResponseAddString(AT_RESPONSE_END);
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// External pressure and temperature sensor command AT$PTS?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_EPTS) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Perform measurements.
				I2C1_PowerOn();
				DPS310_PerformMeasurements(DPS310_EXTERNAL_I2C_ADDRESS);
				I2C1_PowerOff();
				DPS310_GetPressure(&generic_int_1);
				DPS310_GetTemperature(&generic_signed_byte);
				// Print results.
				AT_ResponseAddString("P=");
				AT_ResponseAddValue(generic_int_1, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("Pa T=");
				AT_ResponseAddValue((int) generic_signed_byte, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("dC");
				AT_ResponseAddString(AT_RESPONSE_END);
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// External LDR command AT$LDR?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_ELDR) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Perform measurements.
				I2C1_PowerOn();
#ifdef HW1_0
				SPI1_PowerOn();
#endif
#ifdef HW2_0
				SPI2_PowerOn();
#endif
				// Run external ADC conversions.
				MAX11136_PerformMeasurements();
#ifdef HW1_0
				SPI1_PowerOff();
#endif
#ifdef HW2_0
				SPI2_PowerOff();
#endif
				I2C1_PowerOff();
				ADC1_PerformMeasurements();
				// Get LDR and supply voltage.
				MAX11136_GetChannel(MAX11136_CHANNEL_LDR, &generic_int_1);
				ADC1_GetData(ADC_DATA_IDX_VMCU_MV, &generic_int_2);
				// Print result.
				AT_ResponseAddString("Light=");
				AT_ResponseAddValue((100 * generic_int_1) / (generic_int_2), STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString("%");
				AT_ResponseAddString(AT_RESPONSE_END);
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// Externam UV index sensor command AT$UVS?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_EUVS) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Perform measurements.
				I2C1_PowerOn();
				SI1133_PerformMeasurements(SI1133_EXTERNAL_I2C_ADDRESS);
				I2C1_PowerOff();
				SI1133_GetUvIndex(&generic_uchar);
				// Print result.
				AT_ResponseAddString("UVI=");
				AT_ResponseAddValue(generic_uchar, STRING_FORMAT_DECIMAL, 0);
				AT_ResponseAddString(AT_RESPONSE_END);
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// Wind measurements command AT$WIND=<enable><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_WIND) == PARSER_SUCCESS) {
			parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &generic_int_1);
			if (parser_status == PARSER_SUCCESS) {
				// Start or stop wind continuous measurements.
				if (generic_int_1 == 0) {
					RTC_DisableAlarmBInterrupt();
					RTC_ClearAlarmBFlag();
					WIND_StopContinuousMeasure();
					// Print results.
					WIND_GetSpeed(&generic_uint_1, &generic_uint_2);
					AT_ResponseAddString("AverageSpeed=");
					AT_ResponseAddValue(generic_uint_1, STRING_FORMAT_DECIMAL, 0);
					AT_ResponseAddString("m/h PeakSpeed=");
					AT_ResponseAddValue(generic_uint_2, STRING_FORMAT_DECIMAL, 0);
					WIND_GetDirection(&generic_uint_1);
					if (generic_uint_1 != WIND_DIRECTION_ERROR_VALUE) {
						AT_ResponseAddString("m/h AverageDirection=");
						AT_ResponseAddValue(generic_uint_1, STRING_FORMAT_DECIMAL, 0);
						AT_ResponseAddString("d");
					}
					AT_ResponseAddString(AT_RESPONSE_END);
					// Reset data.
					WIND_ResetData();
					// Update flag.
					at_ctx.wind_measurement_flag = 0;
				}
				else {
					WIND_StartContinuousMeasure();
					RTC_EnableAlarmBInterrupt();
					// Update flag.
					at_ctx.wind_measurement_flag = 1;
				}
				AT_ReplyOk();
			}
			else {
				// Error in enable parameter.
				AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
			}
		}
		// Rain measurements command AT$RAIN=<enable><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_RAIN) == PARSER_SUCCESS) {
			parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &generic_int_1);
			if (parser_status == PARSER_SUCCESS) {
				// Start or stop rain continuous measurements.
				if (generic_int_1 == 0) {
					RAIN_StopContinuousMeasure();
					// Get result.
					RAIN_GetPluviometry(&generic_uchar);
					// Print data.
					AT_ResponseAddString("Rain=");
					AT_ResponseAddValue(generic_uchar, STRING_FORMAT_DECIMAL,0);
					AT_ResponseAddString("mm");
					AT_ResponseAddString(AT_RESPONSE_END);
					// Reset data.
					RAIN_ResetData();
				}
				else {
					RAIN_StartContinuousMeasure();
				}
				AT_ReplyOk();
			}
			else {
				// Error in enable parameter.
				AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
			}
		}
#endif
#ifdef AT_COMMANDS_NVM
		// NVM reset command AT$NVMR<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_NVMR) == PARSER_SUCCESS) {
			// Reset all NVM field to default value.
			NVM_Enable();
			NVM_ResetDefault();
			NVM_Disable();
			AT_ReplyOk();
		}
		// NVM read command AT$NVM=<address_offset><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_NVM) == PARSER_SUCCESS) {
			parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &generic_int_1);
			if (parser_status == PARSER_SUCCESS) {
				// Check if address is reachable.
				if (((unsigned short) generic_int_1) < EEPROM_SIZE) {
					// Read byte at requested address.
					NVM_Enable();
					NVM_ReadByte((unsigned short) generic_int_1, &generic_uchar);
					NVM_Disable();
					// Print byte.
					AT_ResponseAddValue(generic_uchar, STRING_FORMAT_HEXADECIMAL, 1);
					AT_ResponseAddString(AT_RESPONSE_END);
				}
				else {
					AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_NVM_ADDRESS_OVERFLOW);
				}
			}
			else {
				// Error in address parameter.
				AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
			}
		}
		// Get ID command AT$ID?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_ID) == PARSER_SUCCESS) {
			// Retrieve device ID in NVM.
			NVM_Enable();
			for (idx=0 ; idx<ID_LENGTH ; idx++) {
				NVM_ReadByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + ID_LENGTH - idx - 1), &generic_uchar);
				AT_ResponseAddValue(generic_uchar, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
			}
			NVM_Disable();
			AT_ResponseAddString(AT_RESPONSE_END);
		}
		// Set ID command AT$ID=<id><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_ID) == PARSER_SUCCESS) {
			parser_status = PARSER_GetByteArray(&at_ctx.at_parser, AT_CHAR_SEPARATOR, 1, ID_LENGTH, generic_byte_array_1, &extracted_length);
			if (parser_status == PARSER_SUCCESS) {
				// Check length.
				if (extracted_length == ID_LENGTH) {
					// Write device ID in NVM.
					NVM_Enable();
					for (idx=0 ; idx<ID_LENGTH ; idx++) {
						NVM_WriteByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + ID_LENGTH - idx - 1), generic_byte_array_1[idx]);
					}
					NVM_Disable();
					AT_ReplyOk();
				}
				else {
					AT_ReplyError(AT_ERROR_SOURCE_PSR, PARSER_ERROR_PARAMETER_BYTE_ARRAY_INVALID_LENGTH);
				}
			}
			else {
				// Error in ID parameter.
				AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
			}
		}
		// Get key command AT$KEY?<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_KEY) == PARSER_SUCCESS) {
			// Retrieve device key in NVM.
			NVM_Enable();
			for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
				NVM_ReadByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + idx), &generic_uchar);
				AT_ResponseAddValue(generic_uchar, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
			}
			NVM_Disable();
			AT_ResponseAddString(AT_RESPONSE_END);
		}
		// Set key command AT$KEY=<id><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_KEY) == PARSER_SUCCESS) {
			parser_status = PARSER_GetByteArray(&at_ctx.at_parser, AT_CHAR_SEPARATOR, 1, AES_BLOCK_SIZE, generic_byte_array_1, &extracted_length);
			if (parser_status == PARSER_SUCCESS) {
				// Check length.
				if (extracted_length == AES_BLOCK_SIZE) {
					// Write device ID in NVM.
					NVM_Enable();
					for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
						NVM_WriteByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + idx), generic_byte_array_1[idx]);
					}
					NVM_Disable();
					AT_ReplyOk();
				}
				else {
					AT_ReplyError(AT_ERROR_SOURCE_PSR, PARSER_ERROR_PARAMETER_BYTE_ARRAY_INVALID_LENGTH);
				}
			}
			else {
				// Error in ID parameter.
				AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
			}
		}
#endif
#ifdef AT_COMMANDS_SIGFOX
		// Sigfox send OOB command AT$SO<CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_OOB) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Send Sigfox OOB frame.
				sfx_error = SIGFOX_API_open(&at_ctx.sigfox_rc);
				if (sfx_error == SFX_ERR_NONE) {
					sfx_error = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
					sfx_error = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
				}
				SIGFOX_API_close();
				if (sfx_error == SFX_ERR_NONE) {
					AT_ReplyOk();
				}
				else {
					// Error from Sigfox library.
					AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// Sigfox send bit command AT$SB=<bit>,<downlink_request><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_SB) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// First try with 2 parameters.
				parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 0, &generic_int_1);
				if (parser_status == PARSER_SUCCESS) {
					// Try parsing downlink request parameter.
					parser_status =  PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &generic_int_2);
					if (parser_status == PARSER_SUCCESS) {
						// Send Sigfox bit with specified downlink request.
						sfx_error = SIGFOX_API_open(&at_ctx.sigfox_rc);
						if (sfx_error == SFX_ERR_NONE) {
							sfx_error = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
							sfx_error = SIGFOX_API_send_bit((sfx_bool) generic_int_1, generic_byte_array_2, 2, (sfx_bool) generic_int_2);
						}
						SIGFOX_API_close();
						if (sfx_error == SFX_ERR_NONE) {
							if (generic_int_2 != 0) {
								AT_PrintDownlinkData(generic_byte_array_2);
							}
							AT_ReplyOk();
						}
						else {
							// Error from Sigfox library.
							AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
						}
					}
					else {
						// Error in downlink request parameter.
						AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
					}
				}
				else {
					// Try with 1 parameter.
					parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &generic_int_1);
					if (parser_status == PARSER_SUCCESS) {
						// Send Sigfox bit with no downlink request (by default).
						sfx_error = SIGFOX_API_open(&at_ctx.sigfox_rc);
						if (sfx_error == SFX_ERR_NONE) {
							sfx_error = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
							sfx_error = SIGFOX_API_send_bit((sfx_bool) generic_int_1, generic_byte_array_2, 2, 0);
						}
						SIGFOX_API_close();
						if (sfx_error == SFX_ERR_NONE) {
							AT_ReplyOk();
						}
						else {
							// Error from Sigfox library.
							AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
						}
					}
					else {
						// Error in data parameter.
						AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
					}
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// Sigfox send frame command AT$SF=<data>,<downlink_request><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_SF) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// First try with 2 parameters.
				parser_status = PARSER_GetByteArray(&at_ctx.at_parser, AT_CHAR_SEPARATOR, 0, 12, generic_byte_array_1, &extracted_length);
				if (parser_status == PARSER_SUCCESS) {
					// Try parsing downlink request parameter.
					parser_status =  PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &generic_int_2);
					if (parser_status == PARSER_SUCCESS) {
						// Send Sigfox frame with specified downlink request.
						sfx_error = SIGFOX_API_open(&at_ctx.sigfox_rc);
						if (sfx_error == SFX_ERR_NONE) {
							sfx_error = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
							sfx_error = SIGFOX_API_send_frame(generic_byte_array_1, extracted_length, generic_byte_array_2, 2, generic_int_2);
						}
						SIGFOX_API_close();
						if (sfx_error == SFX_ERR_NONE) {
							if (generic_int_2 != 0) {
								AT_PrintDownlinkData(generic_byte_array_2);
							}
							AT_ReplyOk();
						}
						else {
							// Error from Sigfox library.
							AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
						}
					}
					else {
						// Error in downlink request parameter.
						AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
					}
				}
				else {
					// Try with 1 parameter.
					parser_status = PARSER_GetByteArray(&at_ctx.at_parser, AT_CHAR_SEPARATOR, 1, 12, generic_byte_array_1, &extracted_length);
					if (parser_status == PARSER_SUCCESS) {
						// Send Sigfox frame with no downlink request (by default).
						sfx_error = SIGFOX_API_open(&at_ctx.sigfox_rc);
						if (sfx_error == SFX_ERR_NONE) {
							sfx_error = SIGFOX_API_set_std_config(at_ctx.sigfox_rc_std_config, SFX_FALSE);
							sfx_error = SIGFOX_API_send_frame(generic_byte_array_1, extracted_length, generic_byte_array_2, 2, 0);
						}
						SIGFOX_API_close();
						if (sfx_error == SFX_ERR_NONE) {
							AT_ReplyOk();
						}
						else {
							// Error from Sigfox library.
							AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
						}
					}
					else {
						// Error in data parameter.
						AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
					}
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
#endif
#ifdef AT_COMMANDS_CW_RSSI
		// CW command AT$CW=<frequency_hz>,<enable>,<output_power_dbm><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_CW) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Search frequency parameter.
				parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &generic_int_1);
				if (parser_status == PARSER_SUCCESS) {
					// First try with 3 parameters.
					parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 0, &generic_int_2);
					if (parser_status != PARSER_SUCCESS) {
						// Power is not given, try to parse enable as last parameter.
						parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_BOOLEAN, AT_CHAR_SEPARATOR, 1, &generic_int_2);
						if (parser_status == PARSER_SUCCESS) {
							// CW with default output power.
							SIGFOX_API_stop_continuous_transmission();
							if (generic_int_2 != 0) {
								SIGFOX_API_start_continuous_transmission((sfx_u32) generic_int_1, SFX_NO_MODULATION);
							}
							AT_ReplyOk();
						}
						else {
							// Error in enable parameter.
							AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
						}
					}
					else if (parser_status == PARSER_SUCCESS) {
						// There is a third parameter, try to parse power.
						parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &generic_int_3);
						if (parser_status == PARSER_SUCCESS) {
							// CW with given output power.
							SIGFOX_API_stop_continuous_transmission();
							if (generic_int_2 != 0) {
								SIGFOX_API_start_continuous_transmission((sfx_u32) generic_int_1, SFX_NO_MODULATION);
								SX1232_SetRfOutputPower((unsigned char) generic_int_3);
							}
							AT_ReplyOk();
						}
						else {
							// Error in power parameter.
							AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
						}
					}
					else {
						// Error in enable parameter.
						AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
					}
				}
				else {
					// Error in frequency parameter.
					AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
		// RSSI report command AT$RSSI=<frequency_hz><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_RSSI) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Parse frequency parameter.
				parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &generic_int_1);
				if (parser_status == PARSER_SUCCESS) {
					RF_API_init(SFX_RF_MODE_RX);
					RF_API_change_frequency((sfx_u32) generic_int_1);
					// Start continuous listening.
					SX1232_SetMode(SX1232_MODE_FSRX);
					LPTIM1_DelayMilliseconds(5, 0); // Wait TS_FS=60us typical.
					SX1232_SetMode(SX1232_MODE_RX);
					LPTIM1_DelayMilliseconds(5, 0); // Wait TS_TR=120us typical.
					generic_uint_1 = 0;
					while (generic_uint_1 < (AT_RSSI_REPORT_DURATION_MS / AT_RSSI_REPORT_PERIOD_MS)) {
						generic_uchar = SX1232_GetRssi();
						AT_ResponseAddString("RSSI = -");
						AT_ResponseAddValue(generic_uchar, STRING_FORMAT_DECIMAL, 0);
						AT_ResponseAddString("dBm\n");
						LPTIM1_DelayMilliseconds(AT_RSSI_REPORT_PERIOD_MS, 0);
						generic_uint_1++;
					}
					// Stop radio.
					RF_API_stop();
					AT_ReplyOk();
				}
				else {
					// Error in frequency parameter.
					AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
#endif
#ifdef AT_COMMANDS_TEST_MODES
		// Sigfox test mode command AT$TM=<rc>,<test_mode><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_TM) == PARSER_SUCCESS) {
			// Check if wind measurement is not running.
			if (at_ctx.wind_measurement_flag == 0) {
				// Search RC parameter.
				parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 0, &generic_int_1);
				if (parser_status == PARSER_SUCCESS) {
					// Check value.
					if (((sfx_rc_enum_t) generic_int_1) < SFX_RC_LIST_MAX_SIZE) {
						// Search test mode number.
						parser_status =  PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &generic_int_2);
						if (parser_status == PARSER_SUCCESS) {
							// Check parameters.
							if (((sfx_test_mode_t) generic_int_2) <= SFX_TEST_MODE_NVM) {
								// Call test mode function wth public key.
								sfx_error = ADDON_SIGFOX_RF_PROTOCOL_API_test_mode((sfx_rc_enum_t) generic_int_1, (sfx_test_mode_t) generic_int_2);
								if (sfx_error == SFX_ERR_NONE) {
									AT_ReplyOk();
								}
								else {
									// Error from Sigfox library.
									AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
								}
							}
							else {
								// Invalid test mode.
								AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_INVALID_TEST_MODE);
							}
						}
						else {
							// Error in test_mode parameter.
							AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
						}
					}
					else {
						// Invalid RC.
						AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_INVALID_RC);
					}
				}
				else {
					// Error in RC parameter.
					AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
				}
			}
			else {
				AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_FORBIDDEN_COMMAND);
			}
		}
#endif
#ifdef AT_COMMANDS_RC
		// Get Sigfox RC command AT$RC=<rc><CR>.
		else if (PARSER_CompareCommand(&at_ctx.at_parser, AT_COMMAND_RC) == PARSER_SUCCESS) {
			switch (at_ctx.sigfox_rc_idx) {
			case SFX_RC1:
				AT_ResponseAddString("RC1");
				break;
			case SFX_RC2:
				AT_ResponseAddString("RC2");
				break;
			case SFX_RC3A:
				AT_ResponseAddString("RC3A");
				break;
			case SFX_RC3C:
				AT_ResponseAddString("RC3C");
				break;
			case SFX_RC4:
				AT_ResponseAddString("RC4");
				break;
			case SFX_RC5:
				AT_ResponseAddString("RC5");
				break;
			case SFX_RC6:
				AT_ResponseAddString("RC6");
				break;
			case SFX_RC7:
				AT_ResponseAddString("RC7");
				break;
			default:
				AT_ResponseAddString("Unknown RC");
				break;
			}
			AT_ResponseAddString(AT_RESPONSE_END);
		}
		// Set Sigfox RC command AT$RC=<rc><CR>.
		else if (PARSER_CompareHeader(&at_ctx.at_parser, AT_HEADER_RC) == PARSER_SUCCESS) {
			// Search RC parameter.
			parser_status = PARSER_GetParameter(&at_ctx.at_parser, PARSER_PARAMETER_TYPE_DECIMAL, AT_CHAR_SEPARATOR, 1, &generic_int_1);
			if (parser_status == PARSER_SUCCESS) {
				// Check value.
				if (((sfx_rc_enum_t) generic_int_1) < SFX_RC_LIST_MAX_SIZE) {
					// Update radio configuration.
					switch ((sfx_rc_enum_t) generic_int_1) {
					case SFX_RC1:
						at_ctx.sigfox_rc = (sfx_rc_t) RC1;
						at_ctx.sigfox_rc_idx = SFX_RC1;
						AT_ReplyOk();
						break;
					case SFX_RC2:
						at_ctx.sigfox_rc = (sfx_rc_t) RC1;
						for (idx=0 ; idx<AT_SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc2_sm_config[idx];
						at_ctx.sigfox_rc_idx = SFX_RC2;
						AT_ReplyOk();
						break;
					case SFX_RC3A:
						at_ctx.sigfox_rc = (sfx_rc_t) RC3A;
						for (idx=0 ; idx<AT_SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc3a_config[idx];
						at_ctx.sigfox_rc_idx = SFX_RC3A;
						AT_ReplyOk();
						break;
					case SFX_RC3C:
						at_ctx.sigfox_rc = (sfx_rc_t) RC3C;
						for (idx=0 ; idx<AT_SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc3c_config[idx];
						at_ctx.sigfox_rc_idx = SFX_RC3C;
						AT_ReplyOk();
						break;
					case SFX_RC4:
						at_ctx.sigfox_rc = (sfx_rc_t) RC4;
						for (idx=0 ; idx<AT_SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc4_sm_config[idx];
						at_ctx.sigfox_rc_idx = SFX_RC4;
						AT_ReplyOk();
						break;
					case SFX_RC5:
						at_ctx.sigfox_rc = (sfx_rc_t) RC5;
						for (idx=0 ; idx<AT_SIGFOX_RC_STD_CONFIG_SIZE ; idx++) at_ctx.sigfox_rc_std_config[idx] = rc5_config[idx];
						at_ctx.sigfox_rc_idx = SFX_RC5;
						AT_ReplyOk();
						break;
					case SFX_RC6:
						at_ctx.sigfox_rc = (sfx_rc_t) RC6;
						at_ctx.sigfox_rc_idx = SFX_RC6;
						AT_ReplyOk();
						break;
					case SFX_RC7:
						at_ctx.sigfox_rc = (sfx_rc_t) RC7;
						at_ctx.sigfox_rc_idx = SFX_RC7;
						AT_ReplyOk();
						break;
					default:
						// Invalid RC.
						AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_INVALID_RC);
						break;
					}
				}
				else {
					// Invalid RC.
					AT_ReplyError(AT_ERROR_SOURCE_APP, APP_ERROR_INVALID_RC);
				}
			}
			else {
				// Error in RC parameter.
				AT_ReplyError(AT_ERROR_SOURCE_PSR, parser_status);
			}
		}
#endif
		// Unknown command.
		else {
			AT_ReplyError(AT_ERROR_SOURCE_PSR, PARSER_ERROR_UNKNOWN_COMMAND);
		}
	}
	// Send response.
	USARTx_SendString(at_ctx.at_response_buf);
	// Reset AT parser.
	AT_Init();
}

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_Init(void) {
	// Init context.
	unsigned int idx = 0;
	for (idx=0 ; idx<AT_COMMAND_BUFFER_LENGTH ; idx++) at_ctx.at_command_buf[idx] = '\0';
	at_ctx.at_command_buf_idx = 0;
	at_ctx.at_line_end_flag = 0;
	for (idx=0 ; idx<AT_RESPONSE_BUFFER_LENGTH ; idx++) at_ctx.at_response_buf[idx] = '\0';
	at_ctx.at_response_buf_idx = 0;
	at_ctx.wind_measurement_flag = 0;
	// Default Sigfox RC set to RC1.
	at_ctx.sigfox_rc = (sfx_rc_t) RC1;
	at_ctx.sigfox_rc_idx = SFX_RC1;
	// Parsing variables.
	at_ctx.at_parser.rx_buf = (unsigned char*) at_ctx.at_command_buf;
	at_ctx.at_parser.rx_buf_length = 0;
	at_ctx.at_parser.separator_idx = 0;
	at_ctx.at_parser.start_idx = 0;
	// Enable USART interrupt.
#ifdef HW1_0
	NVIC_EnableInterrupt(NVIC_IT_USART2);
#endif
#ifdef HW2_0
	NVIC_EnableInterrupt(NVIC_IT_USART1);
#endif
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_Task(void) {
	// Trigger decoding function if line end found.
	if (at_ctx.at_line_end_flag) {
		AT_DecodeRxBuffer();
	}
	// Check RTC flag for wind measurements.
	if (RTC_GetAlarmBFlag() != 0) {
		// Call WIND callback.
		WIND_MeasurementPeriodCallback();
		RTC_ClearAlarmBFlag();
	}
}

/* FILL AT COMMAND BUFFER WITH A NEW BYTE (CALLED BY USART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void AT_FillRxBuffer(unsigned char rx_byte) {
	// Append byte if LF flag is not allready set.
	if (at_ctx.at_line_end_flag == 0) {
		// Store new byte.
		at_ctx.at_command_buf[at_ctx.at_command_buf_idx] = rx_byte;
		// Manage index.
		at_ctx.at_command_buf_idx++;
		if (at_ctx.at_command_buf_idx >= AT_COMMAND_BUFFER_LENGTH) {
			at_ctx.at_command_buf_idx = 0;
		}
	}
	// Set LF flag to trigger decoding.
	if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
		at_ctx.at_line_end_flag = 1;
	}
}

/* PRINT SIGFOX LIBRARY RESULT.
 * @param test_result:	Test result.
 * @param rssi:			Downlink signal rssi in dBm.
 */
void AT_PrintTestResult(unsigned char test_result, int rssi_dbm) {
	// Check result.
	if (test_result == 0) {
		AT_ResponseAddString("Test failed.");
	}
	else {
		AT_ResponseAddString("Test passed. RSSI=");
		AT_ResponseAddValue(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		AT_ResponseAddString("dBm");
	}
	AT_ResponseAddString(AT_RESPONSE_END);
	// Send response.
	USARTx_SendString(at_ctx.at_response_buf);
	// Reset AT parser.
	AT_Init();
}

/* PRINT PLUVIOMETRY DATA.
 * @param rain_edge_count:	Rain gauge edge counter.
 * @return:					None.
 */
void AT_PrintRain(unsigned char rain_edge_count) {
	// Print data.
	AT_ResponseAddString("RainEdgeCount=");
	AT_ResponseAddValue((int) rain_edge_count, STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString(AT_RESPONSE_END);
	// Send response.
	USARTx_SendString(at_ctx.at_response_buf);
	// Reset AT parser.
	AT_Init();
}

/* PRINT WIND SPEED.
 * @param wind_speed_mh:	Wind speed in m/h.
 * @return:					None.
 */
void AT_PrintWindSpeed(unsigned int wind_speed_mh) {
	// Print data.
	AT_ResponseAddString("Speed=");
	AT_ResponseAddValue((int) wind_speed_mh, STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("m/h");
	AT_ResponseAddString(AT_RESPONSE_END);
	// Send response.
	USARTx_SendString(at_ctx.at_response_buf);
	// Reset AT parser.
	AT_Init();
}

/* PRINT WIND DIRECTION.
 * @param wind_direction_degrees:	Wind direction in degrees.
 * @param wind_direction_x:			Current direction vector x value.
 * @param wind_direction_y:			Current direction vector y value.
 * @return:							None.
 */
void AT_PrintWindDirection(unsigned int wind_direction_degrees, int wind_direction_x, int wind_direction_y) {
	// Print data.
	AT_ResponseAddString("Direction=");
	AT_ResponseAddValue((int) wind_direction_degrees, STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString("d x=");
	AT_ResponseAddValue(wind_direction_x, STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString(" y=");
	AT_ResponseAddValue(wind_direction_y, STRING_FORMAT_DECIMAL, 0);
	AT_ResponseAddString(AT_RESPONSE_END);
	// Send response.
	USARTx_SendString(at_ctx.at_response_buf);
	// Reset AT parser.
	AT_Init();
}

#endif
