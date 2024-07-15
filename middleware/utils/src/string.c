/*
 * string.c
 *
 *  Created on: 05 dec. 2021
 *      Author: Ludo
 */

#include "string.h"

#include "math.h"
#include "types.h"

/*** STRING local macros ***/

#define STRING_VALUE_BUFFER_SIZE	16
#define STRING_SIZE_MAX				100

/*** STRING local functions ***/

/*******************************************************************/
#define _STRING_check_pointer(ptr) { \
	if (ptr == NULL) { \
		status = STRING_ERROR_NULL_PARAMETER; \
		goto errors; \
	} \
}

/*******************************************************************/
static STRING_status_t _STRING_is_decimal_char(char_t chr) {
	// Local variables.
	STRING_status_t status = ((chr >= '0') && (chr <= '9')) ? STRING_SUCCESS : STRING_ERROR_DECIMAL_INVALID;
	return status;
}

/*******************************************************************/
static STRING_status_t _STRING_decimal_char_to_value(char_t chr, uint8_t* value) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	// Check parameters.
	status = _STRING_is_decimal_char(chr);
	if (status != STRING_SUCCESS) goto errors;
	_STRING_check_pointer(value);
	// Perform conversion.
	(*value) = (chr - '0') & 0x0F;
errors:
	return status;
}

/*******************************************************************/
static STRING_status_t _STRING_decimal_value_to_char(uint8_t value, char_t* chr) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	// Check parameters.
	if (value > MATH_DECIMAL_DIGIT_MAX_VALUE) {
		status = STRING_ERROR_DECIMAL_OVERFLOW;
		goto errors;
	}
	_STRING_check_pointer(chr);
	// Perform conversion.
	(*chr) = (value + '0');
errors:
	return status;
}

/*******************************************************************/
static STRING_status_t _STRING_is_hexadecimal_char(char_t chr) {
	// Local variables.
	STRING_status_t status = (((chr >= '0') && (chr <= '9')) || ((chr >= 'A') && (chr <= 'F')) || ((chr >= 'a') && (chr <= 'f'))) ? STRING_SUCCESS : STRING_ERROR_HEXADECIMAL_INVALID;
	return status;
}

/*******************************************************************/
static STRING_status_t _STRING_hexadecimal_char_to_value(char_t chr, uint8_t* value) {
	// Local variables.
	STRING_status_t status = STRING_ERROR_HEXADECIMAL_INVALID;
	// Check parameters.
	_STRING_check_pointer(value);
	// Check ranges.
	if ((chr >= 'A') && (chr <= 'F')) {
		(*value) = (chr - 'A' + 10 ) & 0x0F;
		status = STRING_SUCCESS;
	}
	if ((chr >= 'a') && (chr <= 'f')) {
		(*value) = (chr - 'a' + 10) & 0x0F;
		status = STRING_SUCCESS;
	}
	if ((chr >= '0') && (chr <= '9')) {
		(*value) = (chr - '0') & 0x0F;
		status = STRING_SUCCESS;
	}
errors:
	return status;
}

/*******************************************************************/
static STRING_status_t _STRING_hexadecimal_value_to_char(uint8_t value, char_t* chr) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	// Check parameters.
	if (value > MATH_HEXADECIMAL_DIGIT_MAX_VALUE) {
		status = STRING_ERROR_HEXADECIMAL_OVERFLOW;
		goto errors;
	}
	_STRING_check_pointer(chr);
	// Perform conversion.
	(*chr) = (value <= 9 ? (value + '0') : (value + ('A' - 10)));
errors:
	return status;
}

/*** STRING functions ***/

/*******************************************************************/
STRING_status_t STRING_value_to_string(int32_t value, STRING_format_t format, uint8_t print_prefix, char_t* str) {
    // Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t first_non_zero_found = 0;
	uint32_t str_idx = 0;
	uint8_t generic_byte = 0;
	uint32_t previous_decade = 0;
	uint32_t abs_value = 0;
	int32_t idx = 0;
	// Check parameters.
	_STRING_check_pointer(str);
	// Manage negative numbers.
	if (value < 0) {
		str[str_idx++] = STRING_CHAR_MINUS;
	}
	// Get absolute value.
	MATH_abs(value, abs_value);
	// Build string according to format.
	switch (format) {
	case STRING_FORMAT_BOOLEAN:
		if (print_prefix != 0) {
			// Print "0b" prefix.
            str[str_idx++] = '0';
            str[str_idx++] = 'b';
		}
		for (idx=(MATH_BINARY_DIGIT_MAX_NUMBER - 1) ; idx>=0 ; idx--) {
			if (abs_value & (0b1 << idx)) {
				str[str_idx++] = '1';
				first_non_zero_found = 1;
			}
			else {
				if ((first_non_zero_found != 0) || (idx == 0)) {
					str[str_idx++] = '0';
				}
			}
		}
		break;
	case STRING_FORMAT_HEXADECIMAL:
		if (print_prefix != 0) {
			// Print "0x" prefix.
			str[str_idx++] = '0';
			str[str_idx++] = 'x';
		}
		for (idx=((MATH_HEXADECIMAL_DIGIT_MAX_NUMBER / 2) - 1) ; idx>=0 ; idx--) {
			generic_byte = (abs_value >> (8 * idx)) & 0xFF;
			if (generic_byte != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				// Convert to character.
				status = _STRING_hexadecimal_value_to_char(((generic_byte & 0xF0) >> 4), &(str[str_idx++]));
				if (status != STRING_SUCCESS) goto errors;
				status = _STRING_hexadecimal_value_to_char(((generic_byte & 0x0F) >> 0), &(str[str_idx++]));
				if (status != STRING_SUCCESS) goto errors;
			}
		}
		break;
	case STRING_FORMAT_DECIMAL:
		if (print_prefix != 0) {
			// Print "0d" prefix.
			str[str_idx++] = '0';
			str[str_idx++] = 'd';
		}
		for (idx=(MATH_DECIMAL_DIGIT_MAX_NUMBER - 1) ; idx>=0 ; idx--) {
			generic_byte = (abs_value - previous_decade) / (MATH_POWER_10[idx]);
			previous_decade += (generic_byte * MATH_POWER_10[idx]);
			if (generic_byte != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				status = _STRING_decimal_value_to_char(generic_byte, &(str[str_idx++]));
				if (status != STRING_SUCCESS) goto errors;
			}
		}
		break;
	default:
		status = STRING_ERROR_FORMAT;
		goto errors;
	}
errors:
	str[str_idx++] = STRING_CHAR_NULL; // End string.
	return status;
}

/*******************************************************************/
STRING_status_t STRING_byte_array_to_hexadecimal_string(uint8_t* data, uint8_t data_size, uint8_t print_prefix, char_t* str) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t idx = 0;
	// Check parameters.
	_STRING_check_pointer(data);
	_STRING_check_pointer(str);
	// Build string.
	for (idx=0 ; idx<data_size ; idx++) {
		status = STRING_value_to_string((int32_t) data[idx], STRING_FORMAT_HEXADECIMAL, print_prefix, &(str[2 * idx]));
		if (status != STRING_SUCCESS) goto errors;
	}
errors:
	str[2 * idx] = STRING_CHAR_NULL; // End string.
	return status;
}

/*******************************************************************/
STRING_status_t STRING_string_to_value(char_t* str, STRING_format_t format, uint8_t number_of_digits, int32_t* value) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t char_idx = 0;
	uint8_t start_idx = 0;
	uint8_t negative_flag = 0;
	uint8_t digit_value = 0;
	// Check parameters.
	_STRING_check_pointer(str);
	_STRING_check_pointer(value);
	// Reset result.
	(*value) = 0;
	// Manage negative numbers.
	if (str[0] == STRING_CHAR_MINUS) {
		// Set flag and increment start index to skip minus symbol.
		negative_flag = 1;
		start_idx++;
	}
	// Decode string according to format.
	switch (format) {
	case STRING_FORMAT_BOOLEAN:
		// Check if there is only 1 digit (start and end index are equal).
		if (number_of_digits != MATH_BOOLEAN_DIGIT_MAX_NUMBER) {
			status = STRING_ERROR_BOOLEAN_SIZE;
			goto errors;
		}
		// Get digit and check if it is a bit.
		switch (str[start_idx]) {
		case '0':
			(*value) = 0;
			break;
		case '1':
			(*value) = 1;
			break;
		default:
			status = STRING_ERROR_BOOLEAN_INVALID;
			goto errors;
		}
		break;
	case STRING_FORMAT_HEXADECIMAL:
		// Check if parameter has an even number of digits (two hexadecimal characters are required to code a byte).
		if ((number_of_digits % 2) != 0) {
			status = STRING_ERROR_HEXADECIMAL_ODD_SIZE;
			goto errors;
		}
		// Check if parameter can be binary coded on 32 bits = 4 bytes.
		if (number_of_digits > MATH_HEXADECIMAL_DIGIT_MAX_NUMBER) {
			// Error in parameter -> value is too large.
			status = STRING_ERROR_HEXADECIMAL_OVERFLOW;
			goto errors;
		}
		// Hexadecimal digits loop.
		for (char_idx=0 ; char_idx<number_of_digits ; char_idx++) {
			// Convert digit to value.
			status = _STRING_hexadecimal_char_to_value((str[start_idx + char_idx]), &digit_value);
			if (status != STRING_SUCCESS) goto errors;
			// Add digit to result.
			(*value) |= (digit_value << ((number_of_digits - char_idx - 1) * 4));
		}
		break;
	case STRING_FORMAT_DECIMAL:
		// Check if parameter can be binary coded on 32 bits.
		if (number_of_digits > MATH_DECIMAL_DIGIT_MAX_NUMBER) {
			// Error in parameter -> value is too large.
			status = STRING_ERROR_DECIMAL_OVERFLOW;
			goto errors;
		}
		// Decimal digits loop.
		for (char_idx=0 ; char_idx<number_of_digits ; char_idx++) {
			// Convert digit to value.
			status = _STRING_decimal_char_to_value(str[start_idx + char_idx], &digit_value);
			if (status != STRING_SUCCESS) goto errors;
			// Add digit to result.
			(*value) += (digit_value * MATH_POWER_10[number_of_digits - char_idx - 1]);
		}
		break;
	default:
		status = STRING_ERROR_FORMAT;
		goto errors;
	}
	// Add sign.
	if (negative_flag != 0) {
		(*value) = (-1) * (*value);
	}
errors:
	return status;
}

/*******************************************************************/
STRING_status_t STRING_hexadecimal_string_to_byte_array(char_t* str, char_t end_character, uint8_t* data, uint8_t* extracted_length) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t char_idx = 0;
	int32_t value = 0;
	// Check parameters.
	_STRING_check_pointer(str);
	_STRING_check_pointer(data);
	_STRING_check_pointer(extracted_length);
	// Reset extracted length.
	(*extracted_length) = 0;
	// Char loop.
	while ((str[char_idx] != end_character) && (str[char_idx] != STRING_CHAR_NULL)) {
		// Check character.
		if ((char_idx % 2) == 0) {
			if (_STRING_is_hexadecimal_char(str[char_idx]) != STRING_SUCCESS) {
				// Hexadecimal string end reached before ending character.
				status = STRING_ERROR_HEXADECIMAL_INVALID;
				goto errors;
			}
		}
		else {
			// Check character.
			if (_STRING_is_hexadecimal_char(str[char_idx]) != STRING_SUCCESS) {
				status = STRING_ERROR_HEXADECIMAL_ODD_SIZE;
				goto errors;
			}
			// Convert byte.
			status = STRING_string_to_value(&(str[char_idx - 1]), STRING_FORMAT_HEXADECIMAL, 2, &value);
			if (status !=  STRING_SUCCESS) goto errors;
			// Append byte.
			data[char_idx / 2] = (uint8_t) value;
			(*extracted_length)++;
		}
		char_idx++;
	}
	// Check that the number of analyzed characters is even.
	if ((char_idx % 2) != 0) {
		status = STRING_ERROR_HEXADECIMAL_ODD_SIZE;
		goto errors;
	}
errors:
	return status;
}

/*******************************************************************/
STRING_status_t STRING_get_size(char_t* str, uint8_t* size) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	// Check parameters.
	_STRING_check_pointer(str);
	_STRING_check_pointer(size);
	// Reset result.
	(*size) = 0;
	// Compute source buffer size.
	while (str[(*size)] != STRING_CHAR_NULL) {
		(*size)++;
		// Check overflow.
		if ((*size) > STRING_SIZE_MAX) {
			status = STRING_ERROR_SIZE_OVERFLOW;
			goto errors;
		}
	}
errors:
	return status;
}

/*******************************************************************/
STRING_status_t STRING_copy(STRING_copy_t* copy) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t idx = 0;
	uint8_t source_size = 0;
	uint8_t start_idx = 0;
	uint8_t destination_idx = 0;
	// Reset destination buffer if required.
	if ((copy -> flush_flag) != 0) {
		for (idx=0 ; idx<(copy -> destination_size) ; idx++) (copy -> destination)[idx] = (copy -> flush_char);
	}
	// Compute source buffer size.
	status = STRING_get_size((copy -> source), &source_size);
	if (status != STRING_SUCCESS) goto errors;
	// Check size.
	if (source_size > (copy -> destination_size)) {
		status = STRING_ERROR_COPY_OVERFLOW;
		goto errors;
	}
	// Compute column according to justification.
	switch (copy -> justification) {
	case STRING_JUSTIFICATION_LEFT:
		start_idx = 0;
		break;
	case STRING_JUSTIFICATION_CENTER:
		start_idx = ((copy -> destination_size) - source_size) / (2);
		break;
	case STRING_JUSTIFICATION_RIGHT:
		start_idx = ((copy -> destination_size) - source_size);
		break;
	default:
		status = STRING_ERROR_TEXT_JUSTIFICATION;
		goto errors;
	}
	// Char loop.
	idx = 0;
	while ((copy -> source)[idx] != STRING_CHAR_NULL) {
		// Check index.
		if (destination_idx >= ((copy -> destination_size) - 1)) {
			status = STRING_ERROR_COPY_OVERFLOW;
			goto errors;
		}
		(copy -> destination)[start_idx + idx] = (copy -> source)[idx];
		idx++;
	}
errors:
	return status;
}

/*******************************************************************/
STRING_status_t STRING_append_string(char_t* str, uint8_t str_size_max, char_t* new_str, uint8_t* str_size) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t idx = 0;
	// Fill buffer.
	while (new_str[idx] != STRING_CHAR_NULL) {
		// Check index.
		if ((*str_size) >= str_size_max) {
			status = STRING_ERROR_APPEND_OVERFLOW;
			goto errors;
		}
		str[(*str_size)] = new_str[idx];
		// Increment size and index..
		(*str_size)++;
		idx++;
	}
errors:
	return status;
}

/*******************************************************************/
STRING_status_t STRING_append_value(char_t* str, uint8_t str_size_max, int32_t value, STRING_format_t format, uint8_t print_prefix, uint8_t* str_size) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	char_t str_value[STRING_VALUE_BUFFER_SIZE];
	uint8_t idx = 0;
	// Reset string.
	for (idx=0 ; idx<STRING_VALUE_BUFFER_SIZE ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	status = STRING_value_to_string(value, format, print_prefix, str_value);
	if (status != STRING_SUCCESS) goto errors;
	// Add string.
	status = STRING_append_string(str, str_size_max, str_value, str_size);
errors:
	return status;
}

/*******************************************************************/
STRING_status_t STRING_value_to_5_digits_string(int32_t value, char_t* str) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t u1, u2, u3, u4, u5 = 0;
	uint8_t d1, d2, d3 = 0;
	// Convert value to message.
	if (value < 10000) {
		// Format = u.ddd
		u1 = (value) / (1000);
		d1 = (value - (u1 * 1000)) / (100);
		d2 = (value - (u1 * 1000) - (d1 * 100)) / (10);
		d3 = value - (u1 * 1000) - (d1 * 100) - (d2 * 10);
		status = STRING_value_to_string(u1, STRING_FORMAT_DECIMAL, 0, &(str[0]));
		if (status != STRING_SUCCESS) goto errors;
		str[1] = STRING_CHAR_DOT;
		status = STRING_value_to_string(d1, STRING_FORMAT_DECIMAL, 0, &(str[2]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(d2, STRING_FORMAT_DECIMAL, 0, &(str[3]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(d3, STRING_FORMAT_DECIMAL, 0, &(str[4]));
		if (status != STRING_SUCCESS) goto errors;
	}
	else if (value < 100000) {
		// Format = uu.dd
		u1 = (value) / (10000);
		u2 = (value - (u1 * 10000)) / (1000);
		d1 = (value - (u1 * 10000) - (u2 * 1000)) / (100);
		d2 = (value - (u1 * 10000) - (u2 * 1000) - (d1 * 100)) / (10);
		status = STRING_value_to_string(u1, STRING_FORMAT_DECIMAL, 0, &(str[0]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u2, STRING_FORMAT_DECIMAL, 0, &(str[1]));
		if (status != STRING_SUCCESS) goto errors;
		str[2] = STRING_CHAR_DOT;
		status = STRING_value_to_string(d1, STRING_FORMAT_DECIMAL, 0, &(str[3]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(d2, STRING_FORMAT_DECIMAL, 0, &(str[4]));
		if (status != STRING_SUCCESS) goto errors;
	}
	else if (value < 1000000) {
		// Format = uuu.d
		u1 = (value) / (100000);
		u2 = (value - (u1 * 100000)) / (10000);
		u3 = (value - (u1 * 100000) - (u2 * 10000)) / (1000);
		d1 = (value - (u1 * 100000) - (u2 * 10000) - (u3 * 1000)) / (100);
		status = STRING_value_to_string(u1, STRING_FORMAT_DECIMAL, 0, &(str[0]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u2, STRING_FORMAT_DECIMAL, 0, &(str[1]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u3, STRING_FORMAT_DECIMAL, 0, &(str[2]));
		if (status != STRING_SUCCESS) goto errors;
		str[3] = STRING_CHAR_DOT;
		status = STRING_value_to_string(d1, STRING_FORMAT_DECIMAL, 0, &(str[4]));
		if (status != STRING_SUCCESS) goto errors;
	}
	else if (value < 10000000) {
		// Format = uuuu
		u1 = (value) / (1000000);
		u2 = (value - (u1 * 1000000)) / (100000);
		u3 = (value - (u1 * 1000000) - (u2 * 100000)) / (10000);
		u4 = (value - (u1 * 1000000) - (u2 * 100000) - (u3 * 10000)) / (1000);
		status = STRING_value_to_string(u1, STRING_FORMAT_DECIMAL, 0, &(str[0]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u2, STRING_FORMAT_DECIMAL, 0, &(str[1]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u3, STRING_FORMAT_DECIMAL, 0, &(str[2]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u4, STRING_FORMAT_DECIMAL, 0, &(str[3]));
		if (status != STRING_SUCCESS) goto errors;
	}
	else {
		// Format = uuuuu
		u1 = (value) / (10000000);
		u2 = (value - (u1 * 10000000)) / (1000000);
		u3 = (value - (u1 * 10000000) - (u2 * 1000000)) / (100000);
		u4 = (value - (u1 * 10000000) - (u2 * 1000000) - (u3 * 100000)) / (10000);
		u5 = (value - (u1 * 10000000) - (u2 * 1000000) - (u3 * 100000) - (u4 * 10000)) / (1000);
		status = STRING_value_to_string(u1, STRING_FORMAT_DECIMAL, 0, &(str[0]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u2, STRING_FORMAT_DECIMAL, 0, &(str[1]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u3, STRING_FORMAT_DECIMAL, 0, &(str[2]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u4, STRING_FORMAT_DECIMAL, 0, &(str[3]));
		if (status != STRING_SUCCESS) goto errors;
		status = STRING_value_to_string(u5, STRING_FORMAT_DECIMAL, 0, &(str[4]));
		if (status != STRING_SUCCESS) goto errors;
	}
errors:
	return status;
}
