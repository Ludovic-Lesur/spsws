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
	STRING_FORMAT_BINARY,
	STRING_FORMAT_HEXADECIMAL,
	STRING_FORMAT_DECIMAL,
	STRING_FORMAT_ASCII
} STRING_Format;

/*** STRING functions ***/

unsigned char STRING_AsciiToHexa(char ascii_code);
char STRING_DecimalToAscii(unsigned char decimal_digit);
char STRING_HexaToAscii(unsigned char hexa_digit);
unsigned char STRING_IsHexaChar(char ascii_code);
unsigned char STRING_IsDecimalChar(char ascii_code);
void STRING_ConvertValue(int value, STRING_Format format, unsigned char print_prefix, char* string);

#endif /* STRING_H */
