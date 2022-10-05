/*
 * parser.c
 *
 *  Created on: 10 dec. 2021
 *      Author: Ludo
 */

#include "parser.h"

#include "string.h"
#include "math.h"
#include "types.h"

/*** PARSER local functions ***/

/* GENERIC MACRO TO CHECK RESULT INPUT POINTER.
 * @param ptr:	Pointer to check.
 * @return:		None.
 */
#define _PARSER_check_pointer(ptr) { \
	if (ptr == NULL) { \
		status = PARSER_ERROR_NULL_PARAMETER; \
		goto errors; \
	} \
}

/* SEARCH SEPARATOR IN THE CURRENT AT COMMAND BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param separator:    Reference separator.
 * @return status:      Comparison result.
 */
static PARSER_status_t _PARSER_search_separator(PARSER_context_t* parser_ctx, char_t separator) {
	// Local variables.
	PARSER_status_t status = PARSER_ERROR_SEPARATOR_NOT_FOUND;
	uint8_t idx = 0;
	// Check parameters.
	_PARSER_check_pointer(parser_ctx);
	// Starting from int8_t following the current separator (which is the start of buffer in case of first call).
	for (idx=(parser_ctx -> start_idx) ; idx<(parser_ctx -> rx_buf_length) ; idx++) {
		if ((parser_ctx -> rx_buf)[idx] == separator) {
			(parser_ctx -> separator_idx) = idx;
			status = PARSER_SUCCESS;
			break;
		}
	}
errors:
	return status;
}

/*** PARSER functions ***/

/* CHECK EQUALITY BETWEEN A GIVEN COMMAND OR HEADER AND THE CURRENT AT COMMAND BUFFER.
 * @param parser_ctx:   Parser structure.
 * @param mode:			Comparison mode.
 * @param ref:			Reference string to compare with the buffer.
 * @return status:      Comparison result.
 */
PARSER_status_t PARSER_compare(PARSER_context_t* parser_ctx, PARSER_mode_t mode, char_t* ref) {
	// Local variables.
	PARSER_status_t status = PARSER_SUCCESS;
	uint32_t idx = 0;
	// Check parameters.
	_PARSER_check_pointer(parser_ctx);
	_PARSER_check_pointer(ref);
	// Compare all characters.
	while (ref[idx] != STRING_CHAR_NULL) {
		// Compare current character.
		if ((parser_ctx -> rx_buf)[(parser_ctx -> start_idx) + idx] != ref[idx]) {
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
 * @param param_value:  Pointer hat will contain extracted parameter value.
 * @return status:      Searching result.
 */
PARSER_status_t PARSER_get_parameter(PARSER_context_t* parser_ctx, STRING_format_t param_type, char_t separator, int32_t* param) {
    // Local variables.
	PARSER_status_t status = PARSER_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	uint8_t end_idx = 0;
	uint8_t param_length_char = 0;
	// Check parameters.
	_PARSER_check_pointer(parser_ctx);
	_PARSER_check_pointer(param);
	// Compute end index.
	if (separator != STRING_CHAR_NULL) {
		// Search separator.
		status = _PARSER_search_separator(parser_ctx, separator);
		if (status != PARSER_SUCCESS) goto errors;
		end_idx = (parser_ctx -> separator_idx) - 1;
	}
	else {
		end_idx = (parser_ctx -> rx_buf_length) - 1;
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
 * @param max_length:       Maximum length of the byte array.
 * @param param:            Pointer to the extracted byte array.
 * @param extracted_length:	Length of the extracted buffer.
 * @return status:          Searching result.
 */
PARSER_status_t PARSER_get_byte_array(PARSER_context_t* parser_ctx, char_t separator, uint8_t max_length, uint8_t exact_length, uint8_t* param, uint8_t* extracted_length) {
    // Local variables.
	PARSER_status_t status = PARSER_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	uint8_t param_length_char = 0;
    uint8_t end_idx = 0;
    // Check parameters.
    _PARSER_check_pointer(parser_ctx);
    _PARSER_check_pointer(param);
    _PARSER_check_pointer(extracted_length);
    // Compute end index.
	if (separator != STRING_CHAR_NULL) {
		// Search separator.
		status = _PARSER_search_separator(parser_ctx, separator);
		if (status != PARSER_SUCCESS) goto errors;
		end_idx = (parser_ctx -> separator_idx) - 1;
	}
	else {
		end_idx = (parser_ctx -> rx_buf_length) - 1;
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
