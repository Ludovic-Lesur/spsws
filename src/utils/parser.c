/*
 * parser.c
 *
 *  Created on: 10 dec. 2021
 *      Author: Ludo
 */

#include "parser.h"

#include "string.h"
#include "math.h"

/*** PARSER local macros ***/

#define PARSER_PARAMETER_BINARY_MAX_DIGITS		1
#define PARSER_PARAMETER_HEXADECIMAL_MAX_BYTES	4
#define PARSER_PARAMETER_DECIMAL_MAX_DIGITS		10

/*** PARSER local functions ***/

/* SEARCH SEPARATOR IN THE CURRENT AT COMMAND BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param separator:    Reference separator.
 * @return status:      Comparison result.
 */
static PARSER_Status PARSER_SearchSeparator(PARSER_Context* parser_ctx, char separator) {
	// Local variables.
	PARSER_Status status = PARSER_ERROR_SEPARATOR_NOT_FOUND;
	unsigned char idx = 0;
	// Starting from char following the current separator (which is the start of buffer in case of first call).
	for (idx=(parser_ctx -> start_idx) ; idx<(parser_ctx -> rx_buf_length) ; idx++) {
		if ((parser_ctx -> rx_buf)[idx] == separator) {
			(parser_ctx -> separator_idx) = idx;
			status = PARSER_SUCCESS;
			break;
		}
	}
	return status;
}

/*** PARSER functions ***/

/* CHECK EQUALITY BETWEEN A GIVEN COMMAND AND THE CURRENT AT COMMAND BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param command:      Reference command.
 * @return status:      Comparison result.
 */
PARSER_Status PARSER_CompareCommand(PARSER_Context* parser_ctx, char* command) {
	// Local variables.
	PARSER_Status status = PARSER_SUCCESS;
	unsigned int idx = 0;
	// Compare all characters.
	while (command[idx] != STRING_CHAR_NULL) {
		if ((parser_ctx -> rx_buf)[(parser_ctx -> start_idx) + idx] != command[idx]) {
			// Difference found or end of command, exit loop.
			status = PARSER_ERROR_UNKNOWN_COMMAND;
			goto errors;
		}
		idx++;
	}
	// Check length equality.
	if ((parser_ctx -> rx_buf_length) != idx) {
		status = PARSER_ERROR_UNKNOWN_COMMAND;
		goto errors;
	}
errors:
	return status;
}

/* CHECK EQUALITY BETWEEN A GIVEN HEADER AND THE BEGINNING OF THE CURRENT AT COMMAND BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param header:       Reference header.
 * @return status:      Comparison result.
 */
PARSER_Status PARSER_CompareHeader(PARSER_Context* parser_ctx, char* header) {
	// Local variables.
	PARSER_Status status = PARSER_SUCCESS;
	unsigned int idx = 0;
	// Compare all characters.
	while (header[idx] != STRING_CHAR_NULL) {
		if ((parser_ctx -> rx_buf)[(parser_ctx -> start_idx) + idx] != header[idx]) {
			// Difference found or end of command, exit loop.
			status = PARSER_ERROR_UNKNOWN_COMMAND;
			goto errors;
		}
		idx++;
	}
	// Update start index in case of success.
	(parser_ctx -> start_idx) = idx;
errors:
	return status;
}

/* RETRIEVE A PARAMETER IN THE CURRENT AT BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param param_type:   Format of parameter to get.
 * @param separator:    Parameter separator character.
 * @param last_param:   Indicates if the parameter to scan is the last in AT command.
 * @param param_value:  Pointer hat will contain extracted parameter value.
 * @return status:      Searching result.
 */
PARSER_Status PARSER_GetParameter(PARSER_Context* parser_ctx, PARSER_ParameterType param_type, char separator, unsigned char last_param, int* param) {
    // Local variables.
	PARSER_Status status = PARSER_ERROR_UNKNOWN_COMMAND;
	unsigned char param_length_char = 0;
	unsigned char param_negative_flag = 0;
	unsigned char idx = 0; // Generic index used in for loops.
    unsigned char end_idx = 0;
	unsigned char bit_digit = 0;
	unsigned char hexa_number_of_bytes = 0;
	unsigned char hexa_byte_buf[PARSER_PARAMETER_HEXADECIMAL_MAX_BYTES] = {0};
	unsigned char hexa_digit_idx = 0; // Used instead of idx to ignore offset.
	unsigned char dec_digit_buf[PARSER_PARAMETER_DECIMAL_MAX_DIGITS] = {0};
	unsigned char dec_digit_idx = 0; // Used instead of idx to ignore offset.
	// Search separator if required.
	if (last_param != 0) {
		end_idx = (parser_ctx -> rx_buf_length) - 1;
	}
	else {
		if (PARSER_SearchSeparator(parser_ctx, separator) == PARSER_SUCCESS) {
			end_idx = (parser_ctx -> separator_idx) - 1;
		}
		else {
			status = PARSER_ERROR_SEPARATOR_NOT_FOUND;
			goto errors;
		}
	}
	// Manage negative numbers.
	if ((parser_ctx -> rx_buf)[parser_ctx -> start_idx] == STRING_CHAR_MINUS) {
		// Set flag and increment start index to skip minus symbol.
		param_negative_flag = 1;
		(parser_ctx -> start_idx)++;
	}
	// Compute parameter length.
	param_length_char = (end_idx - (parser_ctx -> start_idx) + 1);
	// Check if parameter is not empty.
	if (param_length_char == 0) {
		status = PARSER_ERROR_PARAMETER_NOT_FOUND;
		goto errors;
	}
	switch (param_type) {
	case PARSER_PARAMETER_TYPE_BOOLEAN:
		// Check if there is only 1 digit (start and end index are equal).
		if (param_length_char == PARSER_PARAMETER_BINARY_MAX_DIGITS) {
			// Get digit and check if it is a bit.
			bit_digit = (parser_ctx -> rx_buf)[parser_ctx -> start_idx];
			if ((bit_digit == STRING_HexaToAscii(0)) || (bit_digit == STRING_HexaToAscii(1))) {
				(*param) = STRING_AsciiToHexa(bit_digit);
				status = PARSER_SUCCESS;
			}
			else {
				status = PARSER_ERROR_PARAMETER_BIT_INVALID;
				goto errors;
			}
		}
		else {
			status = PARSER_ERROR_PARAMETER_BIT_OVERFLOW;
			goto errors;
		}
		break;
	case PARSER_PARAMETER_TYPE_HEXADECIMAL:
		// Check if parameter has an even number of digits (two hexadecimal characters are required to code a byte).
		if ((param_length_char % 2) == 0) {
			// Get the number of byte (= number of digits / 2).
			hexa_number_of_bytes = (param_length_char / 2);
			// Check if parameter can be binary coded on 32 bits = 4 bytes.
			if (hexa_number_of_bytes > PARSER_PARAMETER_HEXADECIMAL_MAX_BYTES) {
				// Error in parameter -> value is too large.
				status = PARSER_ERROR_PARAMETER_HEXA_OVERFLOW;
				goto errors;
			}
			// Scan parameter.
			for (idx=(parser_ctx -> start_idx) ; idx<=end_idx ; idx++) {
				// Increment digit_idx.
				hexa_digit_idx++;
				// Check if buffer content are hexadecimal characters.
				if (STRING_IsHexaChar((parser_ctx -> rx_buf)[idx]) == 0) {
					status = PARSER_ERROR_PARAMETER_HEXA_INVALID;
					goto errors;
				}
				// Get byte every two digits.
				if ((hexa_digit_idx % 2) == 0) {
					// Current byte = (previous digit << 4) + (current digit).
					hexa_byte_buf[(hexa_digit_idx / 2) - 1] = ((STRING_AsciiToHexa((parser_ctx -> rx_buf)[idx - 1])) << 4) + STRING_AsciiToHexa((parser_ctx -> rx_buf)[idx]);
				}
			}
			// The loop didn't return, parameter is valid -> retrieve the number.
			(*param) = 0;
			for (idx=0 ; idx<hexa_number_of_bytes ; idx++) {
				(*param) |= hexa_byte_buf[idx] << (8 * (hexa_number_of_bytes - idx - 1)); // MSB is first in 'byte_buf'.
			}
			// Add sign.
			if (param_negative_flag != 0) {
				(*param) = (-1) * (*param);
			}
			status = PARSER_SUCCESS;
		}
		else {
			// Error in parameter -> odd number of digits while using hexadecimal format.
			status = PARSER_ERROR_PARAMETER_HEXA_ODD_SIZE;
			goto errors;
		}
		break;
	case PARSER_PARAMETER_TYPE_DECIMAL:
		// Check if parameter exists and can be binary coded on 32 bits = 9 digits max.
		if (param_length_char > PARSER_PARAMETER_DECIMAL_MAX_DIGITS) {
			status = PARSER_ERROR_PARAMETER_DEC_OVERFLOW;
			goto errors;
		}
		// Scan parameter.
		for (idx=(parser_ctx -> start_idx) ; idx<=end_idx ; idx++) {
			// Check if buffer content are decimal characters.
			if (STRING_IsDecimalChar((parser_ctx -> rx_buf)[idx]) == 0) {
				status = PARSER_ERROR_PARAMETER_DEC_INVALID;
				goto errors;
			}
			// Store digit and increment index.
			dec_digit_buf[dec_digit_idx] = STRING_AsciiToHexa((parser_ctx -> rx_buf)[idx]);
			dec_digit_idx++;
		}
		// The loop didn't return, parameter is valid -> retrieve the number.
		(*param) = 0;
		for (idx=0 ; idx<param_length_char ; idx++) {
			(*param) = (*param) + dec_digit_buf[idx] * (MATH_Pow10((param_length_char - idx - 1))); // Most significant digit is first in dec_digit_buf.
		}
		// Add sign.
		if (param_negative_flag != 0) {
			(*param) = (-1) * (*param);
		}
		status = PARSER_SUCCESS;
		break;
	default:
		// Unknown parameter format.
		break;
	}
	// Update start index after decoding parameter.
	if ((parser_ctx -> separator_idx) > 0) {
		(parser_ctx -> start_idx) = (parser_ctx -> separator_idx) + 1;
	}
errors:
	return status;
}

/* RETRIEVE A HEXADECIMAL BYTE ARRAY IN THE CURRENT AT BUFFER.
 * @param parser_ctx:       Parser structure.
 * @param separator:        Parameter separator character.
 * @param last_param:		Indicates if the parameter to scan is the last in AT command.
 * @param max_length:       Maximum length of the byte array.
 * @param param:            Pointer to the extracted byte array.
 * @param extracted_length:	Length of the extracted buffer.
 * @return status:          Searching result.
 */
PARSER_Status PARSER_GetByteArray(PARSER_Context* parser_ctx, char separator, unsigned char last_param, unsigned char max_length, unsigned char* param, unsigned char* extracted_length) {
    // Local variables.
	PARSER_Status status = PARSER_ERROR_UNKNOWN_COMMAND;
	unsigned char param_length_char = 0;
	unsigned char idx = 0; // Generic index used in for loops.
    unsigned char end_idx = 0;
	unsigned char hexa_number_of_bytes = 0;
	unsigned char hexa_digit_idx = 0; // Used instead of i to ignore offset.
	(*extracted_length) = 0;
	// Search separator if required.
	if (last_param != 0) {
		end_idx = (parser_ctx -> rx_buf_length) - 1;
	}
	else {
		if (PARSER_SearchSeparator(parser_ctx, separator) == PARSER_SUCCESS) {
			end_idx = (parser_ctx -> separator_idx) - 1;
		}
		else {
			status = PARSER_ERROR_SEPARATOR_NOT_FOUND;
			goto errors;
		}
	}
	// Compute parameter length.
	param_length_char = (end_idx - (parser_ctx -> start_idx) + 1);
	// Check if parameter is not empty.
	if (param_length_char == 0) {
		// Error in parameter -> none parameter found.
		status = PARSER_ERROR_PARAMETER_NOT_FOUND;
		goto errors;
	}
	// Check if parameter has an even number of digits (two hexadecimal characters are required to code a byte).
	if ((param_length_char % 2) == 0) {
		// Get the number of byte (= number of digits / 2).
		hexa_number_of_bytes = (param_length_char / 2);
		// Check if byte array does not exceed given length.
		if (hexa_number_of_bytes > max_length) {
			// Error in parameter -> array is too large.
			status = PARSER_ERROR_PARAMETER_BYTE_ARRAY_INVALID_LENGTH;
			goto errors;
		}
		// Scan each byte.
		for (idx=(parser_ctx -> start_idx) ; idx<=end_idx ; idx++) {
			// Increment digit_idx.
			hexa_digit_idx++;
			// Check if buffer content are hexadecimal characters.
			if (STRING_IsHexaChar((parser_ctx -> rx_buf)[idx]) == 0) {
				status = PARSER_ERROR_PARAMETER_HEXA_INVALID;
				goto errors;
			}
			// Get byte every two digits.
			if ((hexa_digit_idx % 2) == 0) {
				// Current byte = (previous digit << 4) + (current digit).
				param[(hexa_digit_idx / 2) - 1] = ((STRING_AsciiToHexa((parser_ctx -> rx_buf)[idx - 1])) << 4) + STRING_AsciiToHexa((parser_ctx -> rx_buf)[idx]);
				(*extracted_length)++;
			}
		}
		// The loop didn't return, byte array is valid
		status = PARSER_SUCCESS;
	}
	else {
		// Error in parameter -> odd number of digits while using hexadecimal format.
		status = PARSER_ERROR_PARAMETER_HEXA_ODD_SIZE;
		goto errors;
	}
	// Update start index after decoding parameter.
	if ((parser_ctx -> separator_idx) > 0) {
		(parser_ctx -> start_idx) = (parser_ctx -> separator_idx) + 1;
	}
errors:
	return status;
}
