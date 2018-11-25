/*
 * at.c
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludovic
 */

#include "at.h"

#include "usart.h"

/*** AT local macros ***/

#define AT_BUFFER_SIZE			32

#define AT_NULL_CHAR			'\0'
#define AT_SEPARATOR_CHAR		','
#define AT_CR_CHAR				'\r'
#define AT_LF_CHAR				'\n'

#define AT_COMMAND_MIN_SIZE		2
#define AT_HEXA_MAX_DIGITS		8
#define AT_DECIMAL_MAX_DIGITS	9

/*** AT local structures ***/

typedef enum at_param_type {
	AT_Boolean,
	AT_Hexadecimal,
	AT_Decimal
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
		at_ctx.end_idx = at_ctx.at_rx_buf_idx-1; // -1 to ignore <CR>.
	}
	else {
		if (AT_SearchSeparator() == 1) {
			at_ctx.end_idx = at_ctx.separator_idx-1; // -1 to ignore separator.
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
	case AT_Boolean:
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
	case AT_Hexadecimal:
		// Check if parameter has an even number of digits (two hexadecimal characters are required to code a byte).
		// Warning: in this case index delta is an odd number !
		if (((at_ctx.end_idx-at_ctx.start_idx) % 2) != 0) {
			// Get the number of byte (= number of digits / 2).
			hexa_number_of_bytes = ((at_ctx.end_idx-at_ctx.start_idx)+1)/2;
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
				(*param_value) |= hexa_byte_buf[i] << (8*(hexa_number_of_bytes-i-1)); // MSB is first in 'byte_buf'.
			}
			return_code = AT_NO_ERROR;
		}
		else {
			// Error in parameter -> odd number of digits while using hexadecimal format.
			return_code = AT_OUT_ERROR_PARAM_HEXA_ODD_SIZE;
		}
		break;
	case AT_Decimal:
		// Get number of digits.
		dec_number_of_digits = (at_ctx.end_idx-at_ctx.start_idx)+1;
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
			(*param_value) = (*param_value) + dec_digit_buf[i]*(AT_Pow10((dec_number_of_digits-i-1))); // Most significant digit is first in 'digit_buf'.
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

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void AT_ReplyOk(void) {
	USART_SendString(AT_OUT_COMMAND_OK);
	USART_SendValue(AT_CR_CHAR, USART_ASCII);
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error_code:	Error code to display.
 * @return:				None.
 */
void AT_ReplyError(unsigned short error_code) {
	USART_SendString(AT_OUT_HEADER_ERROR);
	USART_SendValue(error_code, USART_Hexadecimal);
	USART_SendValue(AT_CR_CHAR, USART_ASCII);
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
void AT_DecodeRxBuffer(void) {
	// At this step, 'at_buf_idx' is on the <CR> character.

	/* Empty or too short command */
	if (at_ctx.at_rx_buf_idx < AT_COMMAND_MIN_SIZE) {
		// Reply error.
		AT_ReplyError(AT_OUT_ERROR_UNKNOWN_COMMAND);
	}
	else {
		/* Test command */
		/* AT<CR> */
		if (AT_CompareCommand(AT_IN_COMMAND_TEST) == AT_NO_ERROR) {
			// Nothing to do, only reply OK to acknowledge serial link.
			AT_ReplyOk();
		}

		/* Unknown command */
		else {
			AT_ReplyError(AT_OUT_ERROR_UNKNOWN_COMMAND);
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

/* PRINT GPS TIMESTAMP ON USART.
 * @param gps_timestamp:	Pointer to GPS timestamp to print.
 * @return:					None.
 */
void AT_PrintGpsTimestamp(GPS_TimestampData* gps_timestamp) {
	// Header.
	USART_SendString("---> GPS Timestamp ");
	// Year.
	USART_SendValue((gps_timestamp -> date_year), USART_Decimal);
	USART_SendString("-");
	// Month.
	if ((gps_timestamp -> date_month) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> date_month), USART_Decimal);
	USART_SendString("-");
	// Day.
	if ((gps_timestamp -> date_day) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> date_day), USART_Decimal);
	USART_SendString(" ");
	// Hours.
	if ((gps_timestamp -> time_hours) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> time_hours), USART_Decimal);
	USART_SendString(":");
	// Minutes.
	if ((gps_timestamp -> time_minutes) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> time_minutes), USART_Decimal);
	USART_SendString(":");
	// Seconds.
	if ((gps_timestamp -> time_seconds) < 10) {
		USART_SendValue(0, USART_Decimal);
	}
	USART_SendValue((gps_timestamp -> time_seconds), USART_Decimal);
	USART_SendString("\n");
}

/* PRINT GPS POSITION ON USART.
 * @param gps_position:	Pointer to GPS position to print.
 * @return:				None.
 */
void AT_PrintGpsPosition(GPS_PositionData* gps_position) {
	// Header.
	USART_SendString("---> GPS position ");
	// Latitude.
	USART_SendString("Lat=");
	USART_SendValue((gps_position -> lat_degrees), USART_Decimal);
	USART_SendString("°");
	USART_SendValue((gps_position -> lat_minutes), USART_Decimal);
	USART_SendString("'");
	USART_SendValue((gps_position -> lat_seconds), USART_Decimal);
	USART_SendString("''-");
	USART_SendString(((gps_position -> lat_north)==1)? "N" : "S");
	// Longitude.
	USART_SendString(" Long=");
	USART_SendValue((gps_position -> long_degrees), USART_Decimal);
	USART_SendString("°");
	USART_SendValue((gps_position -> long_minutes), USART_Decimal);
	USART_SendString("'");
	USART_SendValue((gps_position -> long_seconds), USART_Decimal);
	USART_SendString("''-");
	USART_SendString(((gps_position -> long_east)==1)? "E" : "W");
	// Altitude.
	USART_SendString(" Alt=");
	USART_SendValue((gps_position -> altitude), USART_Decimal);
	USART_SendString("m\n");
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
