/*
 * parser.c
 *
 *  Created on: 10 dec. 2021
 *      Author: Ludo
 */

#include "parser.h"

#include "string.h"
#include "math.h"

/*** PARSER local functions ***/

/* SEARCH SEPARATOR IN THE CURRENT AT COMMAND BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param separator:    Reference separator.
 * @return status:      Comparison result.
 */
static PARSER_status_t PARSER_search_separator(PARSER_context_t* parser_ctx, char separator) {
	// Local variables.
	PARSER_status_t status = PARSER_ERROR_SEPARATOR_NOT_FOUND;
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

/* CHECK EQUALITY BETWEEN A GIVEN COMMAND OR HEADER AND THE CURRENT AT COMMAND BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param mode:			Comparison mode.
 * @param str:			Input string.
 * @return status:      Comparison result.
 */
PARSER_status_t PARSER_compare(PARSER_context_t* parser_ctx, PARSER_mode_t mode, char* command) {
	// Local variables.
	PARSER_status_t status = PARSER_SUCCESS;
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
	switch (mode) {
	case PARSER_MODE_COMMAND:
		// Check length equality.
		if ((parser_ctx -> rx_buf_length) != idx) {
			status = PARSER_ERROR_UNKNOWN_COMMAND;
			goto errors;
		}
		break;
	case PARSER_MODE_HEADER:
		// Update start index.
		(parser_ctx -> start_idx) = idx;
		break;
	default:
		// Unknown mode.
		status = PARSER_ERROR_MODE;
		goto errors;
		break;
	}
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
PARSER_status_t PARSER_get_parameter(PARSER_context_t* parser_ctx, STRING_format_t param_type, char separator, unsigned char last_param, int* param) {
    // Local variables.
	PARSER_status_t status = PARSER_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	unsigned char end_idx = 0;
	unsigned char param_length_char = 0;
	// Compute end index.
	if (last_param != 0) {
		end_idx = (parser_ctx -> rx_buf_length) - 1;
	}
	else {
		// Search separator.
		status = PARSER_search_separator(parser_ctx, separator);
		if (status != PARSER_SUCCESS) goto errors;
		// Update end index.
		end_idx = (parser_ctx -> separator_idx) - 1;
	}
	// Compute parameter length.
	param_length_char = (end_idx - (parser_ctx -> start_idx) + 1);
	// Check if parameter is not empty.
	if (param_length_char == 0) {
		status = PARSER_ERROR_PARAMETER_NOT_FOUND;
		goto errors;
	}
	// Convert string.
	string_status = STRING_string_to_value(&((parser_ctx -> rx_buf)[parser_ctx -> start_idx]), param_type, param_length_char, param);
	STRING_status_check(PARSER_ERROR_BASE_STRING);
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
PARSER_status_t PARSER_get_byte_array(PARSER_context_t* parser_ctx, char separator, unsigned char last_param, unsigned char max_length, unsigned char exact_length, unsigned char* param, unsigned char* extracted_length) {
    // Local variables.
	PARSER_status_t status = PARSER_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	unsigned char param_length_char = 0;
    unsigned char end_idx = 0;
	// Compute end index.
	if (last_param != 0) {
		end_idx = (parser_ctx -> rx_buf_length) - 1;
	}
	else {
		status = PARSER_search_separator(parser_ctx, separator);
		if (status != PARSER_SUCCESS) goto errors;
		// Update end index.
		end_idx = (parser_ctx -> separator_idx) - 1;
	}
	// Compute parameter length.
	param_length_char = (end_idx - (parser_ctx -> start_idx) + 1);
	// Check if parameter is not empty.
	if (param_length_char == 0) {
		// Error in parameter -> none parameter found.
		status = PARSER_ERROR_PARAMETER_NOT_FOUND;
		goto errors;
	}
	// Convert string.
	string_status = STRING_hexadecimal_string_to_byte_array(&((parser_ctx -> rx_buf)[parser_ctx -> start_idx]), separator, param, extracted_length);
	STRING_status_check(PARSER_ERROR_BASE_STRING);
	// Update start index after decoding parameter.
	if ((parser_ctx -> separator_idx) > 0) {
		(parser_ctx -> start_idx) = (parser_ctx -> separator_idx) + 1;
	}
	// Check length if required.
	if (((exact_length != 0) && ((*extracted_length) != max_length)) || ((*extracted_length) > max_length)) {
		status = PARSER_ERROR_BYTE_ARRAY_LENGTH;
		goto errors;
	}
errors:
	return status;
}
