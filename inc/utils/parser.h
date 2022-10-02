/*
 * parser.h
 *
 *  Created on: 10 dec. 2021
 *      Author: Ludo
 */

#ifndef __PARSER_H__
#define __PARSER_H__

#include "string.h"
#include "types.h"

/*** PARSER structures ***/

typedef enum {
    PARSER_SUCCESS,
	PARSER_ERROR_MODE,
    PARSER_ERROR_UNKNOWN_COMMAND,
    PARSER_ERROR_HEADER_NOT_FOUND,
    PARSER_ERROR_SEPARATOR_NOT_FOUND,
    PARSER_ERROR_PARAMETER_NOT_FOUND,
	PARSER_ERROR_BYTE_ARRAY_LENGTH,
	PARSER_ERROR_BASE_STRING = 0x0100,
	PARSER_ERROR_BASE_LAST = (PARSER_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST)
} PARSER_status_t;

typedef enum {
	PARSER_MODE_COMMAND,
	PARSER_MODE_HEADER,
	PARSER_MODE_LAST
} PARSER_mode_t;

typedef struct {
	char_t* rx_buf;
	uint32_t rx_buf_length;
	uint8_t start_idx;
	uint8_t separator_idx;
} PARSER_context_t;

/*** PARSER functions ***/

PARSER_status_t PARSER_compare(PARSER_context_t* parser_ctx, PARSER_mode_t mode, char_t* ref);
PARSER_status_t PARSER_get_parameter(PARSER_context_t* parser_ctx, STRING_format_t param_type, char_t separator, int32_t* param);
PARSER_status_t PARSER_get_byte_array(PARSER_context_t* parser_ctx, char_t separator, uint8_t max_length, uint8_t exact_length, uint8_t* param, uint8_t* extracted_length);

#define PARSER_status_check(error_base) { if (parser_status != PARSER_SUCCESS) { status = error_base + parser_status; goto errors; }}
#define PARSER_error_check() { ERROR_status_check(parser_status, PARSER_SUCCESS, ERROR_BASE_PARSER); }
#define PARSER_error_check_print() { ERROR_status_check_print(parser_status, PARSER_SUCCESS, ERROR_BASE_PARSER); }

#endif	/* __PARSER_H__ */

