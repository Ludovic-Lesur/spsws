/*
 * string.h
 *
 *  Created on: 05 dec. 2021
 *      Author: Ludo
 */

#ifndef STRING_H
#define STRING_H

/*** STRING macros ***/

#define STRING_CHAR_NULL	'\0'
#define STRING_CHAR_CR		'\r'
#define STRING_CHAR_LF		'\n'
#define STRING_CHAR_MINUS	'-'
#define STRING_CHAR_DOT		'.'

/*** STRING structures ***/

typedef enum {
	STRING_SUCCESS = 0,
	STRING_ERROR_BASE_LAST = 0x0100
} STRING_status_t;

typedef enum {
	STRING_FORMAT_BINARY,
	STRING_FORMAT_HEXADECIMAL,
	STRING_FORMAT_DECIMAL,
	STRING_FORMAT_ASCII,
	STRING_FORMAT_LAST
} STRING_format_t;

/*** STRING functions ***/

unsigned char STRING_ascii_to_hexa(char ascii_code);
char STRING_decimal_to_ascii(unsigned char decimal_digit);
char STRING_hexa_to_ascii(unsigned char hexa_digit);
unsigned char STRING_is_hexa_char(char ascii_code);
unsigned char STRING_is_decimal_char(char ascii_code);
void STRING_convert_value(int value, STRING_format_t format, unsigned char print_prefix, char* string);

#endif /* STRING_H */
