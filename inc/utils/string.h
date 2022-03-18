/*
 * string.h
 *
 *  Created on: 05 dec. 2021
 *      Author: Ludo
 */

#ifndef STRING_H
#define STRING_H

#include "math.h"

/*** STRING macros ***/

#define STRING_CHAR_NULL	'\0'
#define STRING_CHAR_CR		'\r'
#define STRING_CHAR_LF		'\n'
#define STRING_CHAR_MINUS	'-'
#define STRING_CHAR_DOT		'.'

/*** STRING structures ***/

typedef enum {
	STRING_SUCCESS = 0,
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
	STRING_FORMAT_BOOLEAN,
	STRING_FORMAT_HEXADECIMAL,
	STRING_FORMAT_DECIMAL,
	STRING_FORMAT_LAST
} STRING_format_t;

/*** STRING functions ***/

STRING_status_t STRING_value_to_string(int value, STRING_format_t format, unsigned char print_prefix, char* str);
STRING_status_t STRING_byte_array_to_hexadecimal_string(unsigned char* data, unsigned char data_length, unsigned char print_prefix, char* str);

STRING_status_t STRING_string_to_value(char* str, STRING_format_t format, unsigned char number_of_digits, int* value);
STRING_status_t STRING_hexadecimal_string_to_byte_array(char* str, char end_char, unsigned char* data, unsigned char* extracted_length);

#define STRING_status_check(error_base) { if (string_status != STRING_SUCCESS) { status = error_base + string_status; goto errors; }}

#endif /* STRING_H */
