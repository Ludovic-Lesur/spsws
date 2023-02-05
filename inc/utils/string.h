/*
 * string.h
 *
 *  Created on: 05 dec. 2021
 *      Author: Ludo
 */

#ifndef __STRING_H__
#define __STRING_H__

#include "math.h"
#include "types.h"

/*** STRING macros ***/

#define STRING_CHAR_NULL	'\0'
#define STRING_NULL			"\0"
#define STRING_CHAR_CR		'\r'
#define STRING_CHAR_LF		'\n'
#define STRING_CHAR_MINUS	'-'
#define STRING_CHAR_DOT		'.'

/*** STRING structures ***/

typedef enum {
	STRING_SUCCESS = 0,
	STRING_ERROR_NULL_PARAMETER,
	STRING_ERROR_FORMAT,
	STRING_ERROR_BOOLEAN_INVALID,
	STRING_ERROR_BOOLEAN_SIZE,
	STRING_ERROR_HEXADECIMAL_INVALID,
	STRING_ERROR_HEXADECIMAL_ODD_SIZE,
	STRING_ERROR_HEXADECIMAL_OVERFLOW,
	STRING_ERROR_DECIMAL_INVALID,
	STRING_ERROR_DECIMAL_OVERFLOW,
	STRING_ERROR_BASE_MATH = 0x0100,
	STRING_ERROR_BASE_LAST = (STRING_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} STRING_status_t;

typedef enum {
	STRING_FORMAT_BOOLEAN = 0,
	STRING_FORMAT_HEXADECIMAL,
	STRING_FORMAT_DECIMAL,
	STRING_FORMAT_LAST
} STRING_format_t;

/*** STRING functions ***/

STRING_status_t STRING_value_to_string(int32_t value, STRING_format_t format, uint8_t print_prefix, char_t* str);
STRING_status_t STRING_byte_array_to_hexadecimal_string(uint8_t* data, uint8_t data_length, uint8_t print_prefix, char_t* str);

STRING_status_t STRING_string_to_value(char_t* str, STRING_format_t format, uint8_t number_of_digits, int32_t* value);
STRING_status_t STRING_hexadecimal_string_to_byte_array(char_t* str, char_t end_char, uint8_t* data, uint8_t* extracted_length);

#define STRING_status_check(error_base) { if (string_status != STRING_SUCCESS) { status = error_base + string_status; goto errors; }}
#define STRING_error_check() { ERROR_status_check(string_status, STRING_SUCCESS, ERROR_BASE_STRING); }
#define STRING_error_check_print() { ERROR_status_check_print(string_status, STRING_SUCCESS, ERROR_BASE_STRING); }

#endif /* __STRING_H__ */
