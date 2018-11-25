/*
 * at.h
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludovic
 */

#ifndef AT_H
#define AT_H

#include "neom8n.h"

/*** AT macros ***/

// Input commands without parameter.
#define AT_IN_COMMAND_TEST						"AT"

// Input commands with parameters (headers).
#define AT_IN_HEADER_GPS						"AT$GPS=" 	// AT$GPS=<timeout_seconds><CR>.


// Output commands without data.
#define AT_OUT_COMMAND_OK						"OK"

// Output commands with data (headers).
#define AT_OUT_HEADER_ERROR						"ERROR " 	  	// ERROR <error_code><CR>

// Common errors.
#define AT_NO_ERROR						  		0x00 		// For internal processing ("OK" is returned in this case).
#define AT_OUT_ERROR_UNKNOWN_COMMAND			0x01 		// Unknown command or header.
#define AT_OUT_ERROR_NO_PARAM_FOUND				0x02 		// No parameter found after header.
#define AT_OUT_ERROR_NO_SEP_FOUND				0x03 		// No separator found.

// Errors in binary parameter parsing.
#define AT_OUT_ERROR_PARAM_BIT_INVALID_CHAR		0x04 		// Parameter is not a bit (0/1).
#define AT_OUT_ERROR_PARAM_BIT_OVERFLOW	  		0x05 		// Parameter length overflow (> 1 digit).

// Errors in hexadecimal parameter parsing.
#define AT_OUT_ERROR_PARAM_HEXA_ODD_SIZE	  	0x06 		// Odd number of character(s) to code an hexadecimal parameter.
#define AT_OUT_ERROR_PARAM_HEXA_INVALID_CHAR	0x07 		// Invalid character found in hexadecimal parameter.
#define AT_OUT_ERROR_PARAM_HEXA_OVERFLOW		0x08 		// Parameter value overflow (> 32 bits).

// Errors in decimal parameter parsing.
#define AT_OUT_ERROR_PARAM_DEC_INVALID_CHAR		0x09 		// Invalid character found in decimal parameter.
#define AT_OUT_ERROR_PARAM_DEC_OVERFLOW			0x0A 		// Parameter value overflow (> 9 digits).

/*** AT user functions ***/

void AT_Init(void);
void AT_Task(void);
void AT_PrintRtcTimestamp(Timestamp* rtc_timestamp);
void AT_PrintGpsTimestamp(Timestamp* gps_timestamp);
void AT_PrintGpsPosition(Position* gps_position);

/*** AT utility functions ***/

void AT_FillRxBuffer(unsigned char rx_byte);

#endif /* AT_H */
