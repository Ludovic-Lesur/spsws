/*
 * string.c
 *
 *  Created on: 05 dec. 2021
 *      Author: Ludo
 */

#include "string.h"

#include "math.h"

/*** STRING local macros ***/

#define STRING_DIGIT_BOOLEAN_LENGTH			1
#define STRING_DIGIT_DECIMAL_MAX			9
#define STRING_DIGIT_HEXADECIMAL_MAX		0x0F
#define STRING_HEXADECICMAL_DIGIT_PER_BYTE	2

/*** STRING local functions ***/

/* CHECK IF A GIVEN ASCII CODE CORRESPONDS TO A DECIMAL CHARACTER.
 * @param chr:		Character to analyse.
 * @return status:	Function execution status.
 */
static STRING_status_t STRING_is_decimal_char(char chr) {
	// Local variables.
	STRING_status_t status = ((chr >= '0') && (chr <= '9')) ? STRING_SUCCESS : STRING_ERROR_DECIMAL_INVALID;
	return status;
}

/* CONVERTS THE ASCII CODE OF A DECIMAL CHARACTER TO THE CORRESPONDING 4-BIT VALUE.
 * @param chr:		Decimal ASCII char to convert.
 * @param value:	Pointer to the corresponding value.
 * @return status:	Function executions status.
 */
static STRING_status_t STRING_decimal_char_to_value(char chr, unsigned char* value) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	// Check ranges.
	status = STRING_is_decimal_char(chr);
	if (status != STRING_SUCCESS) goto errors;
	// Convert to value.
	(*value) = (chr - '0') & 0x0F;
errors:
	return status;
}

/* RETURN CORRESPONDING ASCII CHARACTER OF A GIVEN DECIMAL VALUE.
 * @param value:	Decimal digit.
 * @return chr:		Corresponding ASCII code.
 */
static STRING_status_t STRING_decimal_value_to_char(unsigned char value, char* chr) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	if (value > STRING_DIGIT_DECIMAL_MAX) {
		status = STRING_ERROR_DECIMAL_OVERFLOW;
		goto errors;
	}
	// Convert to char.
	(*chr) = (value + '0');
errors:
	return status;
}

/* CHECK IF A GIVEN ASCII CODE CORRESPONDS TO AN HEXADECIMAL CHARACTER.
 * @param chr:		Character to analyse.
 * @return status:	Function execution status.
 */
static STRING_status_t STRING_is_hexadecimal_char(char chr) {
	// Local variables.
	STRING_status_t status = (((chr >= '0') && (chr <= '9')) || ((chr >= 'A') && (chr <= 'F')) || ((chr >= 'a') && (chr <= 'f'))) ? STRING_SUCCESS : STRING_ERROR_HEXADECIMAL_INVALID;
	return status;
}

/* CONVERTS THE ASCII CODE OF AN HEXADECIMAL CHARACTER TO THE CORRESPONDING 4-BIT VALUE.
 * @param chr:		Hexadecimal ASCII code to convert.
 * @param value:	Pointer to the corresponding value.
 * @return status:	Function executions status.
 */
static STRING_status_t STRING_hexadecimal_char_to_value(char chr, unsigned char* value) {
	// Local variables.
	STRING_status_t status = STRING_ERROR_HEXADECIMAL_INVALID;
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
	return status;
}

/* CONVERTS A 4-BITS VARIABLE TO THE ASCII CODE OF THE CORRESPONDING HEXADECIMAL CHARACTER IN ASCII.
 * @param value:	Decimal digit.
 * @return chr:		Corresponding ASCII code.
 */
static STRING_status_t STRING_hexadecimal_value_to_char(unsigned char value, char* chr) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	// Check overflow.
	if (value > STRING_DIGIT_HEXADECIMAL_MAX) {
		status = STRING_ERROR_HEXADECIMAL_OVERFLOW;
		goto errors;
	}
	(*chr) = (value <= 9 ? (value + '0') : (value + ('A' - 10)));
errors:
	return status;
}

/*** STRING functions ***/

/* VALUE TO STRING CONVERT FUNCTION.
 * @param value:        Value to print.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @param str:       	Output string.
 * @return status:		Function execution status.
 */
STRING_status_t STRING_value_to_string(int value, STRING_format_t format, unsigned char print_prefix, char* str) {
    // Local variables.
	STRING_status_t status = STRING_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	unsigned char first_non_zero_found = 0;
	unsigned int str_idx = 0;
    unsigned int idx = 0;
	unsigned char generic_byte = 0;
	unsigned int current_power = 0;
	unsigned int previous_decade = 0;
	// Manage negative numbers.
	if (value < 0) {
		str[str_idx++] = STRING_CHAR_MINUS;
	}
	// Build string according to format.
	switch (format) {
	case STRING_FORMAT_BOOLEAN:
		if (print_prefix != 0) {
			// Print "0b" prefix.
            str[str_idx++] = '0';
            str[str_idx++] = 'b';
		}
		for (idx=(MATH_BINARY_MAX_LENGTH - 1) ; idx>=0 ; idx--) {
			if (MATH_abs(value) & (0b1 << idx)) {
				str[str_idx++] = '1';
				first_non_zero_found = 1;
			}
			else {
				if ((first_non_zero_found != 0) || (idx == 0)) {
					str[str_idx++] = '0';
				}
			}
			if (idx == 0) break;
		}
		break;
	case STRING_FORMAT_HEXADECIMAL:
		if (print_prefix != 0) {
			// Print "0x" prefix.
			str[str_idx++] = '0';
			str[str_idx++] = 'x';
		}
		for (idx=(MATH_HEXADECIMAL_MAX_LENGTH - 1) ; idx>=0 ; idx--) {
			generic_byte = (MATH_abs(value) >> (8 * idx)) & 0xFF;
			if (generic_byte != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				// Convert to character.
				status = STRING_hexadecimal_value_to_char(((generic_byte & 0xF0) >> 4), &(str[str_idx++]));
				if (status != STRING_SUCCESS) goto errors;
				status = STRING_hexadecimal_value_to_char(((generic_byte & 0x0F) >> 0), &(str[str_idx++]));
				if (status != STRING_SUCCESS) goto errors;
			}
			if (idx == 0) break;
		}
		break;
	case STRING_FORMAT_DECIMAL:
		if (print_prefix != 0) {
			// Print "0d" prefix.
			str[str_idx++] = '0';
			str[str_idx++] = 'd';
		}
		for (idx=(MATH_DECIMAL_MAX_LENGTH - 1) ; idx>=0 ; idx--) {
			math_status = MATH_pow_10(idx, &current_power);
			MATH_status_check(STRING_ERROR_BASE_MATH);
			generic_byte = (MATH_abs(value) - previous_decade) / current_power;
			previous_decade += generic_byte * current_power;
			if (generic_byte != 0) {
				first_non_zero_found = 1;
			}
			if ((first_non_zero_found != 0) || (idx == 0)) {
				status = STRING_decimal_value_to_char(generic_byte, &(str[str_idx++]));
				if (status != STRING_SUCCESS) goto errors;
			}
			if (idx == 0) break;
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

/* BYTE ARRAY TO HEXADECIMAL STRING CONVERT FUNCTION.
 * @param data:			Input buffer.
 * @param data_length:	Data length in bytes.
 * @param str:       	Output string.
 * @return status:		Function execution status.
 */
STRING_status_t STRING_byte_array_to_hexadecimal_string(unsigned char* data, unsigned char data_length, unsigned char print_prefix, char* str) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	unsigned char idx = 0;
	// Build string.
	for (idx=0 ; idx<data_length ; idx++) {
		status = STRING_value_to_string((int) data[idx], STRING_FORMAT_HEXADECIMAL, print_prefix, &(str[2 * idx]));
		if (status != STRING_SUCCESS) goto errors;
	}
errors:
	str[idx] = STRING_CHAR_NULL; // End string.
	return status;
}

/* STRING TO VALUE CONVERT FUNCTION.
 * @param str:				String to convert.
 * @param format:       	Input string format.
 * @param number_of_digits:	Number of digits to parse.
 * @param value:			Pointer to the result.
 * @return status:			Function execution status.
 */
STRING_status_t STRING_string_to_value(char* str, STRING_format_t format, unsigned char number_of_digits, int* value) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	unsigned char char_idx = 0;
	unsigned char start_idx = 0;
	unsigned char negative_flag = 0;
	unsigned char digit_value = 0;
	unsigned int math_power10 = 0;
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
		if (number_of_digits != STRING_DIGIT_BOOLEAN_LENGTH) {
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
		if (number_of_digits > (STRING_HEXADECICMAL_DIGIT_PER_BYTE * MATH_HEXADECIMAL_MAX_LENGTH)) {
			// Error in parameter -> value is too large.
			status = STRING_ERROR_HEXADECIMAL_OVERFLOW;
			goto errors;
		}
		// Hexadecimal digits loop.
		for (char_idx=0 ; char_idx<number_of_digits ; char_idx++) {
			// Convert digit to value.
			status = STRING_hexadecimal_char_to_value((str[start_idx + char_idx]), &digit_value);
			if (status != STRING_SUCCESS) goto errors;
			// Add digit to result.
			(*value) |= (digit_value << ((number_of_digits - char_idx - 1) * 4));
		}
		break;
	case STRING_FORMAT_DECIMAL:
		// Check if parameter can be binary coded on 32 bits.
		if (number_of_digits > MATH_DECIMAL_MAX_LENGTH) {
			// Error in parameter -> value is too large.
			status = STRING_ERROR_DECIMAL_OVERFLOW;
			goto errors;
		}
		// Decimal digits loop.
		for (char_idx=0 ; char_idx<number_of_digits ; char_idx++) {
			// Convert digit to value.
			status = STRING_decimal_char_to_value(str[start_idx + char_idx], &digit_value);
			if (status != STRING_SUCCESS) goto errors;
			// Compute power.
			math_status = MATH_pow_10((number_of_digits -char_idx - 1), &math_power10);
			MATH_status_check(STRING_ERROR_BASE_MATH);
			// Add digit to result.
			(*value) += (math_power10 * digit_value);
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

/* HEXADECIMAL STRING TO BYTE ARRAY CONVERT FUNCTION.
 * @param str:				Hexadecimal string to convert.
 * @param end_char:			Ending character of the string.
 * @param data:				Pointer to the output byte array.
 * @param extracted_length:	Pointer to the effective extracted length.
 * @return status:			Function execution status.
 */
STRING_status_t STRING_hexadecimal_string_to_byte_array(char* str, char end_char, unsigned char* data, unsigned char* extracted_length) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	unsigned char char_idx = 0;
	int value = 0;
	// Reset extracted length.
	(*extracted_length) = 0;
	// Char loop.
	while ((str[char_idx] != end_char) && (str[char_idx] != STRING_CHAR_NULL)) {
		// Check character.
		if ((char_idx % 2) == 0) {
			if (STRING_is_hexadecimal_char(str[char_idx]) != STRING_SUCCESS) {
				// Hexadecimal string end reached before ending character.
				status = STRING_ERROR_HEXADECIMAL_INVALID;
				goto errors;
			}
		}
		else {
			// Check character.
			if (STRING_is_hexadecimal_char(str[char_idx]) != STRING_SUCCESS) {
				status = STRING_ERROR_HEXADECIMAL_ODD_SIZE;
				goto errors;
			}
			// Convert byte.
			status = STRING_string_to_value(&(str[char_idx - 1]), STRING_FORMAT_HEXADECIMAL, 2, &value);
			if (status !=  STRING_SUCCESS) goto errors;
			// Append byte.
			data[char_idx / 2] = (unsigned char) value;
			(*extracted_length)++;
		}
		char_idx++;
	}
errors:
	return status;
}
