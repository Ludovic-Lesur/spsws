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

#define STRING_CHAR_NULL			'\0'
#define STRING_NULL					"\0"
#define STRING_CHAR_CR				'\r'
#define STRING_CHAR_LF				'\n'
#define STRING_CHAR_MINUS			'-'
#define STRING_CHAR_DOT				'.'
#define STRING_CHAR_SPACE			' '

#define STRING_DIGIT_FUNCTION_SIZE	5

/*** STRING structures ***/

/*!******************************************************************
 * \enum STRING_status_t
 * \brief STRING driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
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
	STRING_ERROR_SIZE_OVERFLOW,
	STRING_ERROR_COPY_OVERFLOW,
	STRING_ERROR_APPEND_OVERFLOW,
	STRING_ERROR_TEXT_JUSTIFICATION,
	// Low level drivers errors.
	STRING_ERROR_BASE_MATH = 0x0100,
	// Last base value.
	STRING_ERROR_BASE_LAST = (STRING_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} STRING_status_t;

/*!******************************************************************
 * \enum STRING_format_t
 * \brief String formats.
 *******************************************************************/
typedef enum {
	STRING_FORMAT_BOOLEAN = 0,
	STRING_FORMAT_HEXADECIMAL,
	STRING_FORMAT_DECIMAL,
	STRING_FORMAT_LAST
} STRING_format_t;

/*!******************************************************************
 * \enum STRING_justification_t
 * \brief String justification formats.
 *******************************************************************/
typedef enum {
	STRING_JUSTIFICATION_LEFT = 0,
	STRING_JUSTIFICATION_CENTER,
	STRING_JUSTIFICATION_RIGHT,
	STRING_JUSTIFICATION_LAST
} STRING_justification_t;

/*!******************************************************************
 * \enum STRING_copy_t
 * \brief String copy operation parameters.
 *******************************************************************/
typedef struct {
	char_t* source;
	char_t* destination;
	uint8_t destination_size;
	STRING_justification_t justification;
	uint8_t flush_flag;
	char_t flush_char;
} STRING_copy_t;

/*** STRING functions ***/

/*!******************************************************************
 * \fn STRING_status_t STRING_value_to_string(int32_t value, STRING_format_t format, uint8_t print_prefix, char_t* str)
 * \brief Convert an integer to the corresponding string representation.
 * \param[in]  	value: Integer to convert.
 * \param[in]	format: Format of the output string.
 * \param[in]	print_prefix: Print the base prefix if non zero.
 * \param[out] 	str: Pointer to the destination string.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_value_to_string(int32_t value, STRING_format_t format, uint8_t print_prefix, char_t* str);

/*!******************************************************************
 * \fn STRING_status_t STRING_byte_array_to_hexadecimal_string(uint8_t* data, uint8_t data_length, uint8_t print_prefix, char_t* str)
 * \brief Convert a byte array to the corresponding string representation.
 * \param[in]  	data: Byte array to convert.
 * \param[in]	data_size: Size of the input byte array.
 * \param[in]	print_prefix: Print the base prefix if non zero.
 * \param[out] 	str: Pointer to the destination string.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_byte_array_to_hexadecimal_string(uint8_t* data, uint8_t data_size, uint8_t print_prefix, char_t* str);

/*!******************************************************************
 * \fn STRING_status_t STRING_string_to_value(char_t* str, STRING_format_t format, uint8_t number_of_digits, int32_t* value)
 * \brief Convert a string to the corresponding value.
 * \param[in]  	str: String to convert.
 * \param[in]	format: Format of the input string.
 * \param[in]	number_of_digits: Number of digits of the output value.
 * \param[out] 	value: Pointer to the destination value.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_string_to_value(char_t* str, STRING_format_t format, uint8_t number_of_digits, int32_t* value);

/*!******************************************************************
 * \fn STRING_status_t STRING_hexadecimal_string_to_byte_array(char_t* str, char_t end_char, uint8_t* data, uint8_t* extracted_length)
 * \brief Convert a string to the corresponding byte array.
 * \param[in]  	str: String to convert.
 * \param[in]	end_character: Character used as string delimiter.
 * \param[out] 	data: Pointer to the destination byte array.
 * \param[out]	extracted_length: Pointer to the effective number of bytes converted.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_hexadecimal_string_to_byte_array(char_t* str, char_t end_character, uint8_t* data, uint8_t* extracted_length);

/*!******************************************************************
 * \fn STRING_status_t STRING_get_size(char_t* str, uint8_t* size)
 * \brief Get the size of a NULL terminated string.
 * \param[in]  	str: String to read.
 * \param[out] 	size: Pointer to the string size.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_get_size(char_t* str, uint8_t* size);

/*!******************************************************************
 * \fn STRING_status_t STRING_copy(STRING_copy_t* copy)
 * \brief Copy a string into another one.
 * \param[in]  	copy: Pointer to the copy operation parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_copy(STRING_copy_t* copy);

/*!******************************************************************
 * \fn STRING_status_t STRING_append_string(char_t* str, uint8_t str_size_max, char_t* new_str, uint8_t* str_size)
 * \brief Append a string to another one.
 * \param[in]  	str: Destination string.
 * \param[in]	str_size_max: Maximum size of the destination string.
 * \param[in]	new_str: String to append.
 * \param[out] 	str_size: Pointer to the new size of the destination string.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_append_string(char_t* str, uint8_t str_size_max, char_t* new_str, uint8_t* str_size);

/*!******************************************************************
 * \fn STRING_status_t STRING_append_value(char_t* str, uint8_t str_size_max, int32_t value, STRING_format_t format, uint8_t print_prefix, uint8_t* str_size)
 * \brief Append a string to another one.
 * \param[in]  	str: Destination string.
 * \param[in]	str_size_max: Maximum size of the destination string.
 * \param[in]	value: Value to convert and append.
 * \param[in]	format: Output format of the value to append.
 * \param[in]	print_prefix: Print the base prefix if non zero.
 * \param[out] 	str_size: Pointer to the new size of the destination string.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_append_value(char_t* str, uint8_t str_size_max, int32_t value, STRING_format_t format, uint8_t print_prefix, uint8_t* str_size);

/*!******************************************************************
 * \fn STRING_status_t STRING_value_to_5_digits_string(int32_t value, char_t* str)
 * \brief Convert a value to a 5 digits string.
 * \param[in]  	value: Value to convert (will be divided by 1000 and represented as a floating number).
 * \param[out] 	str: Pointer to the destination string.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t STRING_value_to_5_digits_string(int32_t value, char_t* str);

/*******************************************************************/
#define STRING_exit_error(error_base) { if (string_status != STRING_SUCCESS) { status = (error_base + string_status); goto errors; } }

/*******************************************************************/
#define STRING_stack_error(void) { if (string_status != STRING_SUCCESS) { ERROR_stack_add(ERROR_BASE_STRING + string_status); } }

/*******************************************************************/
#define STRING_stack_exit_error(error_code) { if (string_status != STRING_SUCCESS) { ERROR_stack_add(ERROR_BASE_STRING + string_status); status = error_code; goto errors; } }

#endif /* __STRING_H__ */
