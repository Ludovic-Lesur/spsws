/*
 * at.c
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludovic
 */

#include "at.h"

#include "aes.h"
#include "dps310.h"
#include "i2c.h"
#include "lpuart.h"
#include "max11136.h"
#include "neom8n.h"
#include "nvm.h"
#include "sht3x.h"
#include "si1133.h"
#include "sigfox_api.h"
#include "spi.h"
#include "sx1232.h"
#include "usart.h"

/*** AT local macros ***/

#define AT_BUFFER_SIZE									64

#define AT_NULL_CHAR									'\0'
#define AT_SEPARATOR_CHAR								','
#define AT_CR_CHAR										'\r'
#define AT_LF_CHAR										'\n'

#define AT_COMMAND_MIN_SIZE								2
#define AT_HEXA_MAX_DIGITS								8
#define AT_DECIMAL_MAX_DIGITS							9

#define AT_CW_DEFAULT_OUTPUT_POWER_DBM					14

// Input commands without parameter.
#define AT_IN_COMMAND_TEST								"AT"
#define AT_IN_COMMAND_ADC								"AT$ADC"
#define AT_IN_COMMAND_MCU								"AT$MCU"
#define AT_IN_COMMAND_ID								"AT$ID?"
#define AT_IN_COMMAND_KEY								"AT$KEY?"
#define AT_IN_COMMAND_SF								"AT$SF"
#define AT_IN_COMMAND_OOB								"AT$OOB"

// Input commands with parameters (headers).
#define AT_IN_HEADER_GPS								"AT$GPS=" 		// AT$GPS=<timeout_seconds><CR>.
#define AT_IN_HEADER_TIME								"AT$TIME="		// AT$TIME=<timeout_seconds><CR>.
#define AT_IN_HEADER_CW									"AT$CW="		// AT$CW=<frequency_hz>,<enable>,<output_power_dbm><CR>.
#define AT_IN_HEADER_ID									"AT$ID="		// AT$ID=<id><CR>.
#define AT_IN_HEADER_KEY								"AT$KEY="		// AT$KEY=<key><CR>.
#define AT_IN_HEADER_SF									"AT$SF="		// AT$SF=<uplink_data>,<downlink_request><CR>.
#define AT_IN_HEADER_SB									"AT$SB="		// AT$SB=<bit><CR>.

// Output commands without data.
#define AT_OUT_COMMAND_OK								"OK"

// Output commands with data (headers).
#define AT_OUT_HEADER_AT_ERROR							"AT_ERROR "		// AT_ERROR <error_code><CR>
#define AT_OUT_HEADER_SFX_ERROR							"SFX_ERROR "	// SFX_ERROR <error_code><CR>

// Syntax errors.
#define AT_NO_ERROR						  				0x00 			// For internal processing ("OK" is returned in this case).
#define AT_OUT_ERROR_UNKNOWN_COMMAND					0x01 			// Unknown command or header.
#define AT_OUT_ERROR_NO_PARAM_FOUND						0x02 			// No parameter found after header.
#define AT_OUT_ERROR_NO_SEP_FOUND						0x03 			// No separator found.
#define AT_OUT_ERROR_PARAM_BIT_INVALID_CHAR				0x04 			// Parameter is not a bit (0/1).
#define AT_OUT_ERROR_PARAM_BIT_OVERFLOW	  				0x05 			// Parameter length overflow (> 1 digit).
#define AT_OUT_ERROR_PARAM_HEXA_ODD_SIZE	  			0x06 			// Odd number of character(s) to code an hexadecimal parameter.
#define AT_OUT_ERROR_PARAM_HEXA_INVALID_CHAR			0x07 			// Invalid character found in hexadecimal parameter.
#define AT_OUT_ERROR_PARAM_HEXA_OVERFLOW				0x08 			// Parameter value overflow (> 32 bits).
#define AT_OUT_ERROR_PARAM_DEC_INVALID_CHAR				0x09 			// Invalid character found in decimal parameter.
#define AT_OUT_ERROR_PARAM_DEC_OVERFLOW					0x0A 			// Invalid length when parsing byte array.
#define AT_OUT_ERROR_PARAM_BYTE_ARRAY_INVALID_LENGTH	0x0B

// Parameters errors
#define AT_OUT_ERROR_TIMEOUT_OVERFLOW					0x80			// Timeout is too large.
#define AT_OUT_ERROR_RF_FREQUENCY_UNDERFLOW				0x81			// RF frequency is too low.
#define AT_OUT_ERROR_RF_FREQUENCY_OVERFLOW				0x82			// RF frequency is too high.
#define AT_OUT_ERROR_RF_OUTPUT_POWER_OVERFLOW			0x83			// RF output power is too high.

// Components errors
#define AT_OUT_ERROR_NEOM8N_INVALID_DATA				0x86		// Invalid data retrieved by GPS.
#define AT_OUT_ERROR_NEOM8N_TIMEOUT						0x87		// GPS timeout.

/*** AT local structures ***/

typedef enum {
	AT_ERROR_SOURCE_AT,
	AT_ERROR_SOURCE_SFX
} AT_ErrorSource;

typedef enum at_param_type {
	AT_PARAM_TYPE_BOOLEAN,
	AT_PARAM_TYPE_HEXADECIMAL,
	AT_PARAM_TYPE_DECIMAL
} AT_ParameterType;

typedef struct {
	// AT command buffer.
	unsigned char at_rx_buf[AT_BUFFER_SIZE];
	unsigned int at_rx_buf_idx;
	unsigned char at_line_end_flag; // Set to '1' as soon as a <CR> or <LF> is received.
	// Parsing variables.
	unsigned int start_idx;
	unsigned int end_idx;
	unsigned int separator_idx;
} AT_Context;

/*** AT local global variables ***/

volatile AT_Context at_ctx;

/*** AT local functions ***/

/* CONVERTS THE ASCII CODE OF AN HEXADECIMAL CHARACTER TO THE CORRESPONDING A 4-BIT WORD.
 * @param c:	The hexadecimal character to convert.
 * @return:		The results of conversion.
 */
unsigned char AT_AsciiToHexa(char c) {
	unsigned char hexa_value = 0;
	if ((c >= 'A') && (c <= 'F')) {
		hexa_value = c - 'A' + 10;
	}
	if ((c >= '0') && (c <= '9')) {
		hexa_value = c - '0';
	}
	return hexa_value;
}

/* CONVERTS A 4-BITS VARIABLE TO THE ASCII CODE OF THE CORRESPONDING HEXADECIMAL CHARACTER IN ASCII.
 * @param n:	The char to converts.
 * @return:		The results of conversion.
 */
char AT_HexaToAscii(unsigned char n) {
	char hexa_char = 0;
	if (n <= 15) {
		hexa_char = (n <= 9 ? (char) (n + '0') : (char) (n + ('A' - 10)));
	}
	return hexa_char;
}

/* CHECK IF A GIVEN ASCII CODE CORRESPONDS TO AN HEXADECIMAL CHARACTER.
 * @param ascii_code:	The byte to analyse.
 * @return:				1 if the byte is the ASCII code of an hexadecimal character, 0 otherwise.
 */
unsigned char AT_IsHexaChar(unsigned char ascii_code) {
	return (((ascii_code >= '0') && (ascii_code <= '9')) || ((ascii_code >= 'A') && (ascii_code <= 'F')));
}

/* CHECK IF A GIVEN ASCII CODE CORRESPONDS TO A DECIMAL CHARACTER.
 * @param ascii_code:	The byte to analyse.
 * @return:				1 if the byte is the ASCII code of a decimal character, 0 otherwise.
 */
unsigned char AT_IsDecimalChar(unsigned char ascii_code) {
	return ((ascii_code >= '0') && (ascii_code <= '9'));
}

/* COMPUTE A POWER A 10.
 * @param power:	The desired power.
 * @return result:	Result of computation.
 */
unsigned int AT_Pow10(unsigned char power) {
	unsigned int result = 0;
	unsigned int pow10_buf[10] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
	if (power <= 9) {
		result = pow10_buf[power];
	}
	return result;
}

/* CHECK EQUALITY BETWEEN A GIVEN COMMAND AND THE CURRENT AT COMMAND BUFFER.
 * @param command:			String to compare.
 * @return return_code:		'AT_NO_ERROR' if strings are identical.
 * 							'AT_OUT_ERROR_UNKNOWN_COMMAND' otherwise.
 */
unsigned short AT_CompareCommand(char* command) {
	unsigned short return_code = AT_OUT_ERROR_UNKNOWN_COMMAND;
	unsigned int idx = 0;
	// 'command' ends with a NULL character (standard in C).
	// 'at_rx_buf' ends at 'at_rx_buf_idx'.
	while ((command[idx] != AT_NULL_CHAR) && (idx < at_ctx.at_rx_buf_idx)) {
		if (command[idx] != at_ctx.at_rx_buf[idx]) {
			// Difference found, exit loop.
			break;
		}
		else {
			// Increment index to check next character.
			idx++;
		}
	}
	// Strings are identical if 'idx' reached 'at_rx_buf_idx'-1 (to ignore the single <CR> or <LF>) and 'command[idx]' = NULL.
	if ((idx == (at_ctx.at_rx_buf_idx-1)) && (command[idx] == AT_NULL_CHAR)) {
		return_code = AT_NO_ERROR;
	}
	return return_code;
}

/* CHECK EQUALITY BETWEEN A GIVEN HEADER AND THE BEGINNING OF THE CURRENT AT COMMAND BUFFER.
 * @param header:			String to compare.
 * @return return_code:		'AT_NO_ERROR' if headers are identical.
 * 							'AT_OUT_ERROR_UNKNOWN_COMMAND' otherwise.
 */
unsigned short AT_CompareHeader(char* header) {
	unsigned short return_code = AT_OUT_ERROR_UNKNOWN_COMMAND;
	unsigned int idx = 0;
	// 'header' ends with a NULL character (standard in C).
	// 'at_rx_buf' ends at 'at_buf_idx'.
	while ((header[idx] != AT_NULL_CHAR) && (idx < at_ctx.at_rx_buf_idx)) {
		if (header[idx] != at_ctx.at_rx_buf[idx]) {
			// Difference found, exit loop.
			break;
		}
		else {
			// Increment index to check next character.
			idx++;
		}
	}
	// Header are identical if 'header[idx]' = NULL.
	if (header[idx] == AT_NULL_CHAR) {
		at_ctx.start_idx = idx; // At this step, idx is on the NULL character (after the header).
		return_code = AT_NO_ERROR;
	}
	else {
		at_ctx.start_idx = 0;
	}
	return return_code;
}

/* SEARCH SEPARATOR IN THE CURRENT AT COMMAND BUFFER.
 * @param:					None.
 * @return separator_found:	Boolean indicating if separator was found.
 */
unsigned char AT_SearchSeparator(void) {
	unsigned char separator_found = 0;
	unsigned int i = 0;
	// Starting from char following the current separator (which is the start of buffer in case of first call).
	for (i=(at_ctx.separator_idx+1) ; i<at_ctx.at_rx_buf_idx ; i++) {
		if (at_ctx.at_rx_buf[i] == AT_SEPARATOR_CHAR) {
			at_ctx.separator_idx = i;
			separator_found = 1;
			break;
		}
	}
	return separator_found;
}

/* RETRIEVE A PARAMETER IN THE CURRENT AT BUFFER.
 * @param param-type:	Format of parameter to get.
 * @param last_param:	Indicates if the parameter to scan is the last in AT command.
 * @param param_value:	Pointer to the parameter value.
 * @return return_code:	AT error code.
 */
unsigned short AT_GetParameter(AT_ParameterType param_type, unsigned char last_param, unsigned int* param_value) {
	unsigned short return_code = AT_OUT_ERROR_UNKNOWN_COMMAND;
	// Local variables.
	unsigned int i = 0; // Generic index used in for loops.
	// Bit parsing.
	unsigned char bit_digit = 0;
	// Hexadecimal parsing.
	unsigned char hexa_number_of_bytes = 0;
	unsigned char hexa_byte_buf[AT_HEXA_MAX_DIGITS/2] = {0};
	unsigned char hexa_digit_idx = 0; // Used instead of i to ignore offset.
	// Decimal parsing.
	unsigned char dec_number_of_digits = 0;
	unsigned char dec_digit_buf[AT_DECIMAL_MAX_DIGITS] = {0};
	unsigned char dec_digit_idx = 0; // Used instead of i to ignore offset.
	// Search separator if required.
	if (last_param == 1) {
		at_ctx.end_idx = at_ctx.at_rx_buf_idx - 2; // -2 to ignore current position + <CR>/<LF>.
	}
	else {
		if (AT_SearchSeparator() == 1) {
			at_ctx.end_idx = at_ctx.separator_idx - 1; // -2 to separator.
		}
		else {
			// Error -> none separator found.
			return AT_OUT_ERROR_NO_SEP_FOUND;
		}
	}
	// Check if parameter is not empty.
	if (at_ctx.end_idx < at_ctx.start_idx) {
		// Error in parameter -> none parameter found.
		return AT_OUT_ERROR_NO_PARAM_FOUND;
	}
	switch (param_type) {
	case AT_PARAM_TYPE_BOOLEAN:
		// Check if there is only 1 digit (start and end index are equal).
		if ((at_ctx.end_idx-at_ctx.start_idx) == 0) {
			// Get digit and check if it is a bit.
			bit_digit = at_ctx.at_rx_buf[at_ctx.start_idx];
			if ((bit_digit == AT_HexaToAscii(0)) || (bit_digit == AT_HexaToAscii(1))) {
				(*param_value) = AT_AsciiToHexa(bit_digit);
				return_code = AT_NO_ERROR;
			}
			else {
				// Error in parameter -> the parameter is not a bit.
				return_code = AT_OUT_ERROR_PARAM_BIT_INVALID_CHAR;
			}
		}
		else {
			// Error in parameter -> more than 1 digit for a boolean parameter.
			return_code = AT_OUT_ERROR_PARAM_BIT_OVERFLOW;
		}
		break;
	case AT_PARAM_TYPE_HEXADECIMAL:
		// Check if parameter has an even number of digits (two hexadecimal characters are required to code a byte).
		// Warning: in this case index delta is an odd number !
		if (((at_ctx.end_idx-at_ctx.start_idx) % 2) != 0) {
			// Get the number of byte (= number of digits / 2).
			hexa_number_of_bytes = ((at_ctx.end_idx - at_ctx.start_idx) + 1) / 2;
			// Check if parameter can be binary coded on 32 bits = 4 bytes.
			if (hexa_number_of_bytes > 4) {
				// Error in parameter -> value is too large.
				return AT_OUT_ERROR_PARAM_HEXA_OVERFLOW;
			}
			// Scan parameter.
			for (i=at_ctx.start_idx ; i<=at_ctx.end_idx ; i++) {
				// Increment digit_idx.
				hexa_digit_idx++;
				// Check if buffer content are hexadecimal characters.
				if (AT_IsHexaChar(at_ctx.at_rx_buf[i]) == 0) {
					return AT_OUT_ERROR_PARAM_HEXA_INVALID_CHAR;
				}
				// Get byte every two digits.
				if ((hexa_digit_idx % 2) == 0) {
					// Current byte = (previous digit << 4) + (current digit).
					hexa_byte_buf[(hexa_digit_idx/2)-1] = ((AT_AsciiToHexa(at_ctx.at_rx_buf[i-1])) << 4) + AT_AsciiToHexa(at_ctx.at_rx_buf[i]);
				}
			}
			// The loop didn't return, parameter is valid -> retrieve the number.
			(*param_value) = 0;
			for (i=0 ; i<hexa_number_of_bytes ; i++) {
				(*param_value) |= hexa_byte_buf[i] << (8 * (hexa_number_of_bytes-i-1)); // MSB is first in 'byte_buf'.
			}
			return_code = AT_NO_ERROR;
		}
		else {
			// Error in parameter -> odd number of digits while using hexadecimal format.
			return_code = AT_OUT_ERROR_PARAM_HEXA_ODD_SIZE;
		}
		break;
	case AT_PARAM_TYPE_DECIMAL:
		// Get number of digits.
		dec_number_of_digits = (at_ctx.end_idx - at_ctx.start_idx) + 1;
		// Check if parameter exists and can be binary coded on 32 bits = 9 digits max.
		if ((dec_number_of_digits) > 9) {
			return AT_OUT_ERROR_PARAM_DEC_OVERFLOW;
		}
		// Scan parameter.
		for (i=at_ctx.start_idx ; i<=at_ctx.end_idx ; i++) {
			// Check if buffer content are decimal characters.
			if (AT_IsDecimalChar(at_ctx.at_rx_buf[i]) == 0) {
				return AT_OUT_ERROR_PARAM_DEC_INVALID_CHAR;
			}
			// Store digit and increment index.
			dec_digit_buf[dec_digit_idx] = AT_AsciiToHexa(at_ctx.at_rx_buf[i]);
			dec_digit_idx++;
		}
		// The loop didn't return, parameter is valid -> retrieve the number.
		(*param_value) = 0;
		for (i=0 ; i<dec_number_of_digits ; i++) {
			(*param_value) = (*param_value) + dec_digit_buf[i] * (AT_Pow10((dec_number_of_digits-i-1))); // Most significant digit is first in 'digit_buf'.
		}
		return_code = AT_NO_ERROR;
		break;
	default:
		// Unknown parameter format.
		break;
	}
	// Update start index after decoding parameter.
	if (at_ctx.separator_idx > 0) {
		at_ctx.start_idx = at_ctx.separator_idx+1;
	}
	return return_code;
}

/* RETRIEVE A HEXADECIMAL BYTE ARRAY IN THE CURRENT AT BUFFER.
 * @param last_param:		Indicates if the parameter to scan is the last in AT command.
 * @param byte_array:		Pointer to the extracted byte array.
 * @param expected_length:	Length of buffer to extract.
 * @return return_code:		AT error code.
 */
unsigned short AT_GetByteArray(unsigned char last_param, unsigned char* byte_array, unsigned char max_length, unsigned char* extracted_length) {
	unsigned short return_code = AT_OUT_ERROR_UNKNOWN_COMMAND;
	// Local variables.
	unsigned int i = 0; // Generic index used in for loops.
	unsigned char hexa_number_of_bytes = 0;
	unsigned char hexa_digit_idx = 0; // Used instead of i to ignore offset.
	(*extracted_length) = 0;
	// Search separator if required.
	if (last_param == 1) {
		at_ctx.end_idx = at_ctx.at_rx_buf_idx - 2; // -2 to ignore current position + <CR>/<LF>.
	}
	else {
		if (AT_SearchSeparator() != 0) {
			at_ctx.end_idx = at_ctx.separator_idx - 1; // -1 to ignore separator.
		}
		else {
			// Error -> none separator found.
			return AT_OUT_ERROR_NO_SEP_FOUND;
		}
	}
	// Check if parameter is not empty.
	if (at_ctx.end_idx < at_ctx.start_idx) {
		// Error in parameter -> none parameter found.
		return AT_OUT_ERROR_NO_PARAM_FOUND;
	}
	// Check if parameter has an even number of digits (two hexadecimal characters are required to code a byte).
	// Warning: in this case index delta is an odd number !
	if (((at_ctx.end_idx - at_ctx.start_idx) % 2) != 0) {
		// Get the number of byte (= number of digits / 2).
		hexa_number_of_bytes = ((at_ctx.end_idx - at_ctx.start_idx) + 1) / 2;
		// Check if byte array does not exceed given length.
		if (hexa_number_of_bytes > max_length) {
			// Error in parameter -> array is too large.
			return AT_OUT_ERROR_PARAM_BYTE_ARRAY_INVALID_LENGTH;
		}
		// Scan each byte.
		for (i=at_ctx.start_idx ; i<=at_ctx.end_idx ; i++) {
			// Increment digit_idx.
			hexa_digit_idx++;
			// Check if buffer content are hexadecimal characters.
			if (AT_IsHexaChar(at_ctx.at_rx_buf[i]) == 0) {
				return AT_OUT_ERROR_PARAM_HEXA_INVALID_CHAR;
			}
			// Get byte every two digits.
			if ((hexa_digit_idx % 2) == 0) {
				// Current byte = (previous digit << 4) + (current digit).
				byte_array[(hexa_digit_idx/2)-1] = ((AT_AsciiToHexa(at_ctx.at_rx_buf[i-1])) << 4) + AT_AsciiToHexa(at_ctx.at_rx_buf[i]);
				(*extracted_length)++;
			}
		}
		// The loop didn't return, byte array is valid
		return_code = AT_NO_ERROR;
	}
	else {
		// Error in parameter -> odd number of digits while using hexadecimal format.
		return_code = AT_OUT_ERROR_PARAM_HEXA_ODD_SIZE;
	}
	// Update start index after decoding parameter.
	if (at_ctx.separator_idx > 0) {
		at_ctx.start_idx = at_ctx.separator_idx+1;
	}
	return return_code;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void AT_ReplyOk(void) {
	USART_SendString(AT_OUT_COMMAND_OK);
	USART_SendValue(AT_CR_CHAR, USART_FORMAT_ASCII, 0);
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error_code:	Error code to display.
 * @return:				None.
 */
void AT_ReplyError(AT_ErrorSource error_source, unsigned short error_code) {
	switch (error_source) {
	case AT_ERROR_SOURCE_AT:
		USART_SendString(AT_OUT_HEADER_AT_ERROR);
		break;
	case AT_ERROR_SOURCE_SFX:
		USART_SendString(AT_OUT_HEADER_SFX_ERROR);
		break;
	default:
		break;
	}
	USART_SendValue(error_code, USART_FORMAT_HEXADECIMAL, 1);
	USART_SendValue(AT_CR_CHAR, USART_FORMAT_ASCII, 0);
}

/* PRINT ADC RESULTS.
 * @param:	None.
 * @return:	None.
 */
void AT_PrintAdcResults(void) {
	unsigned short result_12bits = 0;
	// Compute effective supply voltage.
	MAX11136_GetChannelVoltage12bits(7, &result_12bits);
	unsigned int vcc_mv= (MAX11136_BANDGAP_VOLTAGE_MV * MAX11136_FULL_SCALE) / (result_12bits);
	// AIN0.
	MAX11136_GetChannelVoltage12bits(0, &result_12bits);
	USART_SendString("AIN0=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	// AIN1.
	MAX11136_GetChannelVoltage12bits(1, &result_12bits);
	USART_SendString("mV AIN1=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	// AIN2.
	MAX11136_GetChannelVoltage12bits(2, &result_12bits);
	USART_SendString("mV AIN2=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	// AIN3.
	MAX11136_GetChannelVoltage12bits(3, &result_12bits);
	USART_SendString("mV AIN3=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	// AIN4.
	MAX11136_GetChannelVoltage12bits(4, &result_12bits);
	USART_SendString("mV AIN4=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	// AIN5.
	MAX11136_GetChannelVoltage12bits(5, &result_12bits);
	USART_SendString("mV AIN5=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	// AIN6.
	MAX11136_GetChannelVoltage12bits(6, &result_12bits);
	USART_SendString("mV AIN6=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	// AIN7.
	MAX11136_GetChannelVoltage12bits(7, &result_12bits);
	USART_SendString("mV AIN7=");
	USART_SendValue(((result_12bits * vcc_mv) / (MAX11136_FULL_SCALE)), USART_FORMAT_DECIMAL, 0);
	USART_SendString("mV\n");
}

/* PRINT A TIMESTAMP ON USART.
 * @param timestamp_to_print:	Pointer to the timestamp to print.
 * @return:						None.
 */
void AT_PrintTimestamp(Timestamp* timestamp_to_print) {
	// Year.
	USART_SendValue((timestamp_to_print -> year), USART_FORMAT_DECIMAL, 0);
	USART_SendString("-");
	// Month.
	if ((timestamp_to_print -> month) < 10) {
		USART_SendValue(0, USART_FORMAT_DECIMAL, 0);
	}
	USART_SendValue((timestamp_to_print -> month), USART_FORMAT_DECIMAL, 0);
	USART_SendString("-");
	// Day.
	if ((timestamp_to_print -> date) < 10) {
		USART_SendValue(0, USART_FORMAT_DECIMAL, 0);
	}
	USART_SendValue((timestamp_to_print -> date), USART_FORMAT_DECIMAL, 0);
	USART_SendString(" ");
	// Hours.
	if ((timestamp_to_print -> hours) < 10) {
		USART_SendValue(0, USART_FORMAT_DECIMAL, 0);
	}
	USART_SendValue((timestamp_to_print -> hours), USART_FORMAT_DECIMAL, 0);
	USART_SendString(":");
	// Minutes.
	if ((timestamp_to_print -> minutes) < 10) {
		USART_SendValue(0, USART_FORMAT_DECIMAL, 0);
	}
	USART_SendValue((timestamp_to_print -> minutes), USART_FORMAT_DECIMAL, 0);
	USART_SendString(":");
	// Seconds.
	if ((timestamp_to_print -> seconds) < 10) {
		USART_SendValue(0, USART_FORMAT_DECIMAL, 0);
	}
	USART_SendValue((timestamp_to_print -> seconds), USART_FORMAT_DECIMAL, 0);
	USART_SendString("\n");
}

/* PRINT GPS POSITION ON USART.
 * @param gps_position:	Pointer to GPS position to print.
 * @return:				None.
 */
void AT_PrintPosition(Position* gps_position) {
	// Header.
	// Latitude.
	USART_SendString("Lat=");
	USART_SendValue((gps_position -> lat_degrees), USART_FORMAT_DECIMAL, 0);
	USART_SendString("°");
	USART_SendValue((gps_position -> lat_minutes), USART_FORMAT_DECIMAL, 0);
	USART_SendString("'");
	USART_SendValue((gps_position -> lat_seconds), USART_FORMAT_DECIMAL, 0);
	USART_SendString("''-");
	USART_SendString(((gps_position -> lat_north)==1)? "N" : "S");
	// Longitude.
	USART_SendString(" Long=");
	USART_SendValue((gps_position -> long_degrees), USART_FORMAT_DECIMAL, 0);
	USART_SendString("°");
	USART_SendValue((gps_position -> long_minutes), USART_FORMAT_DECIMAL, 0);
	USART_SendString("'");
	USART_SendValue((gps_position -> long_seconds), USART_FORMAT_DECIMAL, 0);
	USART_SendString("''-");
	USART_SendString(((gps_position -> long_east)==1)? "E" : "W");
	// Altitude.
	USART_SendString(" Alt=");
	USART_SendValue((gps_position -> altitude), USART_FORMAT_DECIMAL, 0);
	USART_SendString("m\n");
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
void AT_DecodeRxBuffer(void) {

	// At this step, 'at_buf_idx' is 1 character after the first line end character (<CR> or <LF>).
	unsigned short get_param_result = 0;
	unsigned char byte_idx = 0;
	unsigned char extracted_length = 0;
	sfx_u8 sfx_uplink_data[12] = {0x00};
	sfx_u8 sfx_downlink_data[8] = {0x00};
	sfx_error_t sfx_error = 0;
	sfx_rc_t rc1 = RC1;

	/* Empty or too short command */
	if (at_ctx.at_rx_buf_idx < AT_COMMAND_MIN_SIZE) {
		// Reply error.
		AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_UNKNOWN_COMMAND);
	}
	else {

		/* Test command AT<CR> */
		if (AT_CompareCommand(AT_IN_COMMAND_TEST) == AT_NO_ERROR) {
			// Nothing to do, only reply OK to acknowledge serial link.
			AT_ReplyOk();
		}

		/* ADC command AT$ADC<CR> */
		else if (AT_CompareCommand(AT_IN_COMMAND_ADC) == AT_NO_ERROR) {
			// Trigger ADC convertion.
			SPI_PowerOn();
			MAX11136_ConvertAllChannels();
			SPI_PowerOff();
			// Print results.
			AT_PrintAdcResults();
		}

		/* GPS command AT$GPS=<timeout_seconds><CR> */
		else if (AT_CompareHeader(AT_IN_HEADER_GPS) == AT_NO_ERROR) {
			unsigned int timeout_seconds = 0;
			// Search timeout parameter.
			get_param_result = AT_GetParameter(AT_PARAM_TYPE_DECIMAL, 1, &timeout_seconds);
			if (get_param_result == AT_NO_ERROR) {
				// Start GPS fix.
				Position gps_position;
				LPUART_PowerOn();
				//NEOM8N_ReturnCode get_position_result = NEOM8N_GetPosition(&gps_position, timeout_seconds);
				NEOM8N_ReturnCode get_position_result = NEOM8N_TIMEOUT;
				LPUART_PowerOff();
				switch (get_position_result) {
				case NEOM8N_SUCCESS:
					AT_PrintPosition(&gps_position);
					break;
				case NEOM8N_INVALID_DATA:
					AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_NEOM8N_INVALID_DATA);
					break;
				case NEOM8N_TIMEOUT:
					AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_NEOM8N_TIMEOUT);
					break;
				default:
					break;
				}
			}
			else {
				// Error in timeout parameter.
				AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
			}
		}

		/* TIME command AT$TIME=<timeout_seconds><CR> */
		else if (AT_CompareHeader(AT_IN_HEADER_TIME) == AT_NO_ERROR) {
			unsigned int timeout_seconds = 0;
			// Search timeout parameter.
			get_param_result = AT_GetParameter(AT_PARAM_TYPE_DECIMAL, 1, &timeout_seconds);
			if (get_param_result == AT_NO_ERROR) {
				// Start GPS fix.
				Timestamp gps_timestamp;
				LPUART_PowerOn();
				//NEOM8N_ReturnCode get_timestamp_result = NEOM8N_GetTimestamp(&gps_timestamp, timeout_seconds);
				NEOM8N_ReturnCode get_timestamp_result = NEOM8N_TIMEOUT;
				LPUART_PowerOff();
				switch (get_timestamp_result) {
				case NEOM8N_SUCCESS:
					AT_PrintTimestamp(&gps_timestamp);
					break;
				case NEOM8N_INVALID_DATA:
					AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_NEOM8N_INVALID_DATA);
					break;
				case NEOM8N_TIMEOUT:
					AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_NEOM8N_TIMEOUT);
					break;
				default:
					break;
				}
			}
			else {
				// Error in timeout parameter.
				AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
			}
		}

		/* CW command AT$CW=<frequency_hz>,<enable>,<output_power_dbm><CR> */
		else if (AT_CompareHeader(AT_IN_HEADER_CW) == AT_NO_ERROR) {
			unsigned int frequency_hz = 0;
			// Search frequency parameter.
			get_param_result = AT_GetParameter(AT_PARAM_TYPE_DECIMAL, 0, &frequency_hz);
			if (get_param_result == AT_NO_ERROR) {
				unsigned char enable = 0;
				// First try with 3 parameters.
				get_param_result = AT_GetParameter(AT_PARAM_TYPE_BOOLEAN, 0, &frequency_hz);
				if (get_param_result == AT_OUT_ERROR_NO_SEP_FOUND) {
					// Power is not given, try to parse enable as last parameter.
					get_param_result = AT_GetParameter(AT_PARAM_TYPE_BOOLEAN, 1, &frequency_hz);
					if (get_param_result == AT_NO_ERROR) {
						// CW with default output power.
						if (enable == 0) {
							SX1232_StopCw();
							SPI_PowerOff();
						}
						else {
							SPI_PowerOn();
							SX1232_StartCw(frequency_hz, AT_CW_DEFAULT_OUTPUT_POWER_DBM);
						}
					}
					else {
						// Error in enable parameter.
						AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
					}
				}
				else if (get_param_result == AT_NO_ERROR) {
					// There is a third parameter, try to parse power.
					unsigned int output_power_dbm = 0;
					get_param_result = AT_GetParameter(AT_PARAM_TYPE_DECIMAL, 1, &output_power_dbm);
					if (get_param_result == AT_NO_ERROR) {
						// CW with given output power.
						if (enable == 0) {
							SX1232_StopCw();
							SPI_PowerOff();
						}
						else {
							SPI_PowerOn();
							SX1232_StartCw(frequency_hz, output_power_dbm);
						}
					}
					else {
						// Error in power parameter.
						AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
					}
				}
				else {
					// Error in enable parameter.
					AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
				}
			}
			else {
				// Error in frequency parameter.
				AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
			}
		}

		/* Get ID command AT$ID?<CR> */
		else if (AT_CompareCommand(AT_IN_COMMAND_ID) == AT_NO_ERROR) {
			// Retrieve device ID in NVM.
			unsigned char id_byte = 0;
			for (byte_idx=0 ; byte_idx<ID_LENGTH ; byte_idx++) {
				NVM_ReadByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + ID_LENGTH - byte_idx - 1), &id_byte);
				USART_SendValue(id_byte, USART_FORMAT_HEXADECIMAL, (byte_idx==0 ? 1 : 0));
			}
			USART_SendString("\n");
		}

		/* Set ID command AT$ID=<id><CR> */
		else if (AT_CompareHeader(AT_IN_HEADER_ID) == AT_NO_ERROR) {
			unsigned char param_id[ID_LENGTH] = {0};
			get_param_result = AT_GetByteArray(1, param_id, ID_LENGTH, &extracted_length);
			if (get_param_result == AT_NO_ERROR) {
				// Check length.
				if (extracted_length == ID_LENGTH) {
					// Write device ID in NVM.
					for (byte_idx=0 ; byte_idx<ID_LENGTH ; byte_idx++) {
						NVM_WriteByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + ID_LENGTH - byte_idx - 1), param_id[byte_idx]);
					}
					AT_ReplyOk();
				}
				else {
					AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_PARAM_BYTE_ARRAY_INVALID_LENGTH);
				}
			}
			else {
				// Error in ID parameter.
				AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
			}
		}

		/* Get key command AT$KEY?<CR> */
		else if (AT_CompareCommand(AT_IN_COMMAND_KEY) == AT_NO_ERROR) {
			// Retrieve device key in NVM.
			unsigned char id_byte = 0;
			unsigned char byte_idx = 0;
			for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE ; byte_idx++) {
				NVM_ReadByte((NVM_SIGFOX_KEY_ADDRESS_OFFSET + byte_idx), &id_byte);
				USART_SendValue(id_byte, USART_FORMAT_HEXADECIMAL, (byte_idx==0 ? 1 : 0));
			}
			USART_SendString("\n");
		}

		/* Set key command AT$KEY=<id><CR> */
		else if (AT_CompareHeader(AT_IN_HEADER_KEY) == AT_NO_ERROR) {
			unsigned char param_key[ID_LENGTH] = {0};
			get_param_result = AT_GetByteArray(1, param_key, AES_BLOCK_SIZE, &extracted_length);
			if (get_param_result == AT_NO_ERROR) {
				// Check length.
				if (extracted_length == AES_BLOCK_SIZE) {
					// Write device ID in NVM.
					for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE ; byte_idx++) {
						NVM_WriteByte((NVM_SIGFOX_ID_ADDRESS_OFFSET + byte_idx), param_key[byte_idx]);
					}
					AT_ReplyOk();
				}
				else {
					AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_PARAM_BYTE_ARRAY_INVALID_LENGTH);
				}
			}
			else {
				// Error in ID parameter.
				AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
			}
		}

		/* Sigfox send empty frame command AT$SF<CR> */
		else if (AT_CompareCommand(AT_IN_COMMAND_SF) == AT_NO_ERROR) {
			// Send Sigfox empty frame.
			SIGFOX_API_open(&rc1);
			sfx_error = SIGFOX_API_send_frame(sfx_uplink_data, 0, sfx_downlink_data, 2, 0);
			SIGFOX_API_close();
			if (sfx_error == SFX_ERR_NONE) {
				AT_ReplyOk();
			}
			else {
				// Error from Sigfox library.
				AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
			}
		}

		/* Sigfox send frame command AT$SF=<data>,<downlink_request><CR> */
		else if (AT_CompareHeader(AT_IN_HEADER_SF) == AT_NO_ERROR) {
			// First try with 2 parameters.
			get_param_result = AT_GetByteArray(0, sfx_uplink_data, 12, &extracted_length);
			if (get_param_result == AT_NO_ERROR) {
				// Try parsing downlink request parameter.
				unsigned int downlink_request = 0;
				get_param_result =  AT_GetParameter(AT_PARAM_TYPE_BOOLEAN, 1, &downlink_request);
				if (get_param_result == AT_NO_ERROR) {
					// Send Sigfox frame with specified downlink request.
					SIGFOX_API_open(&rc1);
					sfx_error = SIGFOX_API_send_frame(sfx_uplink_data, extracted_length, sfx_downlink_data, 2, downlink_request);
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
					// Error in downlink request parameter.
					AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
				}
			}
			else {
				// Try with 1 parameter.
				get_param_result = AT_GetByteArray(1, sfx_uplink_data, 12, &extracted_length);
				if (get_param_result == AT_NO_ERROR) {
					// Send Sigfox frame with no downlink request (by default).
					SIGFOX_API_open(&rc1);
					sfx_error = SIGFOX_API_send_frame(sfx_uplink_data, extracted_length, sfx_downlink_data, 2, 0);
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
					AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
				}
			}
		}

		/* Sigfox send bit command */
		else if (AT_CompareHeader(AT_IN_HEADER_SB) == AT_NO_ERROR) {
			unsigned int data_bit = 0;
			// First try with 2 parameters.
			get_param_result = AT_GetParameter(AT_PARAM_TYPE_BOOLEAN, 0, &data_bit);
			if (get_param_result == AT_NO_ERROR) {
				// Try parsing downlink request parameter.
				unsigned int downlink_request = 0;
				get_param_result =  AT_GetParameter(AT_PARAM_TYPE_BOOLEAN, 1, &downlink_request);
				if (get_param_result == AT_NO_ERROR) {
					// Send Sigfox bit with specified downlink request.
					SIGFOX_API_open(&rc1);
					sfx_error = SIGFOX_API_send_bit(data_bit, sfx_downlink_data, 2, downlink_request);
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
					// Error in downlink request parameter.
					AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
				}
			}
			else {
				// Try with 1 parameter.
				get_param_result = AT_GetParameter(AT_PARAM_TYPE_BOOLEAN, 1, &data_bit);
				if (get_param_result == AT_NO_ERROR) {
					// Send Sigfox bit with no downlink request (by default).
					SIGFOX_API_open(&rc1);
					sfx_error = SIGFOX_API_send_bit(data_bit, sfx_downlink_data, 2, 0);
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
					AT_ReplyError(AT_ERROR_SOURCE_AT, get_param_result);
				}
			}
		}

		else if (AT_CompareCommand(AT_IN_COMMAND_OOB) == AT_NO_ERROR) {
			// Send Sigfox OOB frame.
			SIGFOX_API_open(&rc1);
			sfx_error = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
			SIGFOX_API_close();
			if (sfx_error == SFX_ERR_NONE) {
				AT_ReplyOk();
			}
			else {
				// Error from Sigfox library.
				AT_ReplyError(AT_ERROR_SOURCE_SFX, sfx_error);
			}
		}

		/* Unknown command */
		else {
			AT_ReplyError(AT_ERROR_SOURCE_AT, AT_OUT_ERROR_UNKNOWN_COMMAND);
		}
	}
}

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_Init(void) {

	/* Init context */
	// AT command buffer.
	unsigned int idx = 0;
	for (idx=0 ; idx<AT_BUFFER_SIZE ; idx++) at_ctx.at_rx_buf[idx] = 0;
	at_ctx.at_rx_buf_idx = 0;
	at_ctx.at_line_end_flag = 0;
	// Parsing variables.
	at_ctx.start_idx = 0;
	at_ctx.end_idx = 0;
	at_ctx.separator_idx = 0;
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_Task(void) {
	// Trigger decode function if line end found.
	if (at_ctx.at_line_end_flag) {
		AT_DecodeRxBuffer();
		AT_Init();
	}
}

/* FILL AT COMMAND BUFFER WITH A NEW BYTE FROM USART.
 * @param rx_byte:	New byte to store.
 * @return:			None.
 */
void AT_FillRxBuffer(unsigned char rx_byte) {
	unsigned char increment_idx = 1;

	/* Append incoming byte to buffer */
	if ((rx_byte == AT_CR_CHAR) || (rx_byte == AT_LF_CHAR)) {
		// Append line end only if the previous byte was not allready a line end and if other characters are allready presents.
		if (((at_ctx.at_rx_buf[at_ctx.at_rx_buf_idx-1] != AT_LF_CHAR) && (at_ctx.at_rx_buf[at_ctx.at_rx_buf_idx-1] != AT_CR_CHAR)) && (at_ctx.at_rx_buf_idx > 0)) {
			at_ctx.at_rx_buf[at_ctx.at_rx_buf_idx] = rx_byte;
			// Set line end flag to trigger decoding.
			at_ctx.at_line_end_flag = 1;
		}
		else {
			// No byte stored, do not increment buffer index.
			increment_idx = 0;
		}
	}
	else {
		// Append byte in all cases.
		at_ctx.at_rx_buf[at_ctx.at_rx_buf_idx] = rx_byte;
	}

	/* Increment index and manage rollover */
	if (increment_idx != 0) {
		at_ctx.at_rx_buf_idx++;
		if (at_ctx.at_rx_buf_idx >= AT_BUFFER_SIZE) {
			at_ctx.at_rx_buf_idx = 0;
		}
	}
}
