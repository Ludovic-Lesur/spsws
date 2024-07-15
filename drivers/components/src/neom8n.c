/*
 * neom8n.c
 *
 *  Created on: 11 aug. 2018
 *      Author: Ludo
 */

#include "neom8n.h"

#include "dma.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "nvic.h"
#include "pwr.h"
#include "rtc.h"
#include "string.h"
#include "types.h"

/*** NEOM8N local macros ***/

#define NEOM8N_EXTERNAL_WAKEUP_COUNT_LIMIT	5

#define NEOM8N_MSG_OVERHEAD_SIZE			8 // 6 bytes header + 2 bytes checksum.
#define NEOM8N_CHECKSUM_OVERHEAD_SIZE		4
#define NEOM8N_CHECKSUM_OFFSET				2
#define NEOM8N_CFG_MSG_PAYLOAD_SIZE			8
#define NEOM8N_CFG_TP5_PAYLOAD_SIZE			32
#define NEOM8N_TIMEOUT_SECONDS_MIN			10

#define NMEA_RX_BUFFER_SIZE					128

#define NMEA_CHAR_MESSAGE_START				'$'
#define NMEA_CHAR_CHECKSUM_START			'*' // To skip '$'.
#define NMEA_CHAR_SEPARATOR					','

#define NMEA_ZDA_MASK						0x00020000 // Provided to _NEOM8N_select_nmea_messages() function.
#define NMEA_GGA_MASK						0x00000008 // Provided to _NEOM8N_select_nmea_messages() function.

#define NMEA_GGA_NORTH						'N'
#define NMEA_GGA_SOUTH						'S'
#define NMEA_GGA_EAST						'E'
#define NMEA_GGA_WEST						'W'
#define NMEA_GGA_METERS						'M'

#define NEOM8N_TIMEPULSE_FREQUENCY_MAX_HZ	10000000

/*** NEOM8N local structures ***/

/*******************************************************************/
typedef enum {
	NMEA_ZDA_FIELD_INDEX_MESSAGE = 0,
	NMEA_ZDA_FIELD_INDEX_TIME,
	NMEA_ZDA_FIELD_INDEX_DAY,
	NMEA_ZDA_FIELD_INDEX_MONTH,
	NMEA_ZDA_FIELD_INDEX_YEAR,
	NMEA_ZDA_FIELD_INDEX_LTZH,
	NMEA_ZDA_FIELD_INDEX_LTZN,
} NMEA_zda_field_index_t;

/*******************************************************************/
typedef enum {
	NMEA_ZDA_FIELD_SIZE_MESSAGE = 5,
	NMEA_ZDA_FIELD_SIZE_TIME = 9,
	NMEA_ZDA_FIELD_SIZE_DAY = 2,
	NMEA_ZDA_FIELD_SIZE_MONTH = 2,
	NMEA_ZDA_FIELD_SIZE_YEAR = 4,
	NMEA_ZDA_FIELD_SIZE_LTZH = 2,
	NMEA_ZDA_FIELD_SIZE_LTZN = 2
} NMEA_zda_field_length_t;

/*******************************************************************/
typedef enum {
	NMEA_GGA_FIELD_INDEX_MESSAGE = 0,
	NMEA_GGA_FIELD_INDEX_TIME,
	NMEA_GGA_FIELD_INDEX_LAT,
	NMEA_GGA_FIELD_INDEX_NS,
	NMEA_GGA_FIELD_INDEX_LONG,
	NMEA_GGA_FIELD_INDEX_EW,
	NMEA_GGA_FIELD_INDEX_QUALITY,
	NMEA_GGA_FIELD_INDEX_NUMSV,
	NMEA_GGA_FIELD_INDEX_HDOP,
	NMEA_GGA_FIELD_INDEX_ALT,
	NMEA_GGA_FIELD_INDEX_U_ALT,
	NMEA_GGA_FIELD_INDEX_SEP,
	NMEA_GGA_FIELD_INDEX_U_SEP,
	NMEA_GGA_FIELD_INDEX_DIFF_AGE,
	NMEA_GGA_FIELD_INDEX_DIFF_STATION
} NMEA_gga_field_index_t;

/*******************************************************************/
typedef enum {
	NMEA_GGA_FIELD_SIZE_MESSAGE = 5,
	NMEA_GGA_FIELD_SIZE_TIME = 9,
	NMEA_GGA_FIELD_SIZE_LAT = 10,
	NMEA_GGA_FIELD_SIZE_NS = 1,
	NMEA_GGA_FIELD_SIZE_LONG = 11,
	NMEA_GGA_FIELD_SIZE_EW = 1,
	NMEA_GGA_FIELD_SIZE_QUALITY = 1,
	NMEA_GGA_FIELD_SIZE_NUM_SV = 0,
	NMEA_GGA_FIELD_SIZE_HDOP = 0,
	NMEA_GGA_FIELD_SIZE_ALT = 0,
	NMEA_GGA_FIELD_SIZE_U_ALT = 1,
	NMEA_GGA_FIELD_SIZE_SEP = 0,
	NMEA_GGA_FIELD_SIZE_U_SEP = 1,
	NMEA_GGA_FIELD_SIZE_DIFF_AGE = 0,
	NMEA_GGA_FIELD_SIZE_DIFF_STATION = 0
} NMEA_gga_field_length_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned rx_irq : 1;
		unsigned line_end : 1;
		unsigned fill_buffer0 : 1;
	};
	uint8_t all;
} NEOM8N_flags_t;

/*******************************************************************/
typedef struct {
	// Buffers.
	volatile char_t rx_buffer0[NMEA_RX_BUFFER_SIZE];
	volatile char_t rx_buffer1[NMEA_RX_BUFFER_SIZE];
	volatile NEOM8N_flags_t flags;
	// Altitude stability filter.
	uint8_t gga_same_altitude_count;
	uint32_t gga_previous_altitude;
} NEOM8N_context_t;

/*** NEOM8N local global variables ***/

static NEOM8N_context_t neom8n_ctx;

/*** NEOM8N local functions ***/

/*******************************************************************/
static void _NEOM8N_switch_dma_buffer(uint8_t line_end_flag) {
	// Stop and start DMA transfer to switch buffer.
	DMA1_CH6_stop();
	// Switch buffer.
	if (neom8n_ctx.flags.fill_buffer0 == 0) {
		DMA1_CH6_set_destination_address((uint32_t) &(neom8n_ctx.rx_buffer0), NMEA_RX_BUFFER_SIZE); // Switch to buffer 0.
		neom8n_ctx.flags.fill_buffer0 = 1;
	}
	else {
		DMA1_CH6_set_destination_address((uint32_t) &(neom8n_ctx.rx_buffer1), NMEA_RX_BUFFER_SIZE); // Switch to buffer 1.
		neom8n_ctx.flags.fill_buffer0 = 0;
	}
	// Update flags.
	neom8n_ctx.flags.rx_irq = 1;
	neom8n_ctx.flags.line_end = line_end_flag;
	// Restart DMA transfer.
	DMA1_CH6_start();
}

/*******************************************************************/
void _NEOM8N_usart_cm_irq_callback(void) {
	// Switch buffer.
	_NEOM8N_switch_dma_buffer(1);
}

/*******************************************************************/
void _NEOM8N_dma_tc_irq_callback(void) {
	// Switch buffer.
	_NEOM8N_switch_dma_buffer(0);
}

/*******************************************************************/
#define _NEOM8N_check_field_length(field_length) { if ((char_idx - separator_idx) != (field_length + 1)) {status = NEOM8N_ERROR_NMEA_FIELD_SIZE; goto errors;} }

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_compute_ubx_checksum(uint8_t* neom8n_command, uint8_t payload_length) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	uint32_t checksum_idx = 0;
	// Check parameters.
	if (neom8n_command == NULL) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// See algorithm on p.136 of NEO-M8 programming manual.
	for (checksum_idx=NEOM8N_CHECKSUM_OFFSET ; checksum_idx<((uint32_t) (NEOM8N_CHECKSUM_OFFSET + NEOM8N_CHECKSUM_OVERHEAD_SIZE + payload_length)) ; checksum_idx++) {
		ck_a = ck_a + neom8n_command[checksum_idx];
		ck_b = ck_b + ck_a;
	}
	// Fill two last bytes of the NEOM8N message with CK_A and CK_B.
	neom8n_command[checksum_idx + 0] = ck_a;
	neom8n_command[checksum_idx + 1] = ck_b;
errors:
	return status;
}

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_get_nmea_checksum(char_t* nmea_rx_buf, uint8_t* ck) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	uint8_t checksum_start_char_idx = 0;
	int32_t ck_value = 0;
	// Check parameters.
	if ((nmea_rx_buf == NULL) || (ck == NULL)) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Get checksum start index (see NMEA messages format on p.105 of NEO-M8 programming manual).
	while ((nmea_rx_buf[checksum_start_char_idx] != NMEA_CHAR_CHECKSUM_START) && (checksum_start_char_idx < NMEA_RX_BUFFER_SIZE)) {
		checksum_start_char_idx++;
	}
	if (checksum_start_char_idx >= NMEA_RX_BUFFER_SIZE) {
		status = NEOM8N_ERROR_CHECKSUM_INDEX;
		goto errors;
	}
	// Convert hexadecimal to value.
	string_status = STRING_string_to_value(&(nmea_rx_buf[checksum_start_char_idx + 1]), STRING_FORMAT_HEXADECIMAL, 2, &ck_value);
	STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
	// Cast to byte.
	(*ck) = (uint8_t) ck_value;
errors:
	return status;
}

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_compute_nmea_checksum(char_t* nmea_rx_buf, uint8_t* ck) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	uint8_t message_start_char_idx = 0;
	uint8_t checksum_start_char_idx = 0;
	uint8_t checksum_idx = 0;
	// Check parameters.
	if ((nmea_rx_buf == NULL) || (ck == NULL)) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Get message start index (see algorithm on p.105 of NEO-M8 programming manual).
	while ((nmea_rx_buf[message_start_char_idx] != NMEA_CHAR_MESSAGE_START) && (message_start_char_idx < NMEA_RX_BUFFER_SIZE)) {
		message_start_char_idx++;
	}
	// Get checksum start index.
	checksum_start_char_idx = message_start_char_idx;
	while ((nmea_rx_buf[checksum_start_char_idx] != NMEA_CHAR_CHECKSUM_START) && (checksum_start_char_idx < NMEA_RX_BUFFER_SIZE)) {
		checksum_start_char_idx++;
	}
	if (checksum_start_char_idx >= NMEA_RX_BUFFER_SIZE) {
		status = NEOM8N_ERROR_CHECKSUM_INDEX;
		goto errors;
	}
	// Compute checksum.
	(*ck) = 0;
	for (checksum_idx=(message_start_char_idx + 1); checksum_idx<checksum_start_char_idx ; checksum_idx++) {
		(*ck) ^= nmea_rx_buf[checksum_idx]; // Exclusive OR of all characters between '$' and '*'.
	}
errors:
	return status;
}

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_time_is_valid(RTC_time_t* gps_time) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	// Check parameters.
	if (gps_time == NULL) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check time fields.
	if (((gps_time -> date) < 1) || ((gps_time -> date) > 31) ||
		((gps_time -> month) < 1) || ((gps_time -> month) > 12) ||
		((gps_time -> year) < 2021) || ((gps_time -> year) > 2094) ||
		((gps_time -> hours) > 23) ||
		((gps_time -> minutes) > 59) ||
		((gps_time -> seconds) > 59))
	{
		status = NEOM8N_ERROR_TIME_INVALID;
	}
errors:
	return status;
}

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_parse_nmea_zda(char_t* nmea_rx_buf, RTC_time_t* gps_time) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	uint8_t char_idx = 0;
	uint8_t separator_idx = 0;
	uint8_t received_checksum = 0;
	uint8_t computed_checksum = 0;
	int32_t value = 0;
	NMEA_zda_field_index_t field_idx = 0;
	// Check parameters.
	if ((nmea_rx_buf == NULL) || (gps_time == NULL)) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Verify checksum.
	status = _NEOM8N_get_nmea_checksum(nmea_rx_buf, &received_checksum);
	if (status != NEOM8N_SUCCESS) goto errors;
	status = _NEOM8N_compute_nmea_checksum(nmea_rx_buf, &computed_checksum);
	if (status != NEOM8N_SUCCESS) goto errors;
	if (computed_checksum != received_checksum) {
		status = NEOM8N_ERROR_CHECKSUM;
		goto errors;
	}
	// Search NMEA start character.
	while ((nmea_rx_buf[separator_idx] != NMEA_CHAR_MESSAGE_START) && (separator_idx < NMEA_RX_BUFFER_SIZE)) {
		separator_idx++;
		char_idx++;
	}
	// Extract NMEA data (see ZDA message format on p.127 of NEO-M8 programming manual).
	while ((nmea_rx_buf[char_idx] != STRING_CHAR_LF) && (char_idx < NMEA_RX_BUFFER_SIZE)) {
		// Check if separator is found.
		if (nmea_rx_buf[char_idx] == NMEA_CHAR_SEPARATOR) {
			// Get current field.
			switch (field_idx) {
			// Field 0 = address = <ID><message>.
			case NMEA_ZDA_FIELD_INDEX_MESSAGE:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_ZDA_FIELD_SIZE_MESSAGE);
				// Check if message = 'ZDA'.
				if ((nmea_rx_buf[separator_idx + 3] != 'Z') || (nmea_rx_buf[separator_idx + 4] != 'D') || (nmea_rx_buf[separator_idx + 5] != 'A')) {
					status = NEOM8N_ERROR_NMEA_MESSAGE;
					goto errors;
				}
				break;
			// Field 1 = time = hhmmss.ss.
			case NMEA_ZDA_FIELD_INDEX_TIME:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_ZDA_FIELD_SIZE_TIME);
				// Parse hours.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_time -> hours = (uint8_t) value;
				// Parse minutes.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 3]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_time -> minutes = (uint8_t) value;
				// Parse seconds.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 5]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_time -> seconds = (uint8_t) value;
				break;
			// Field 2 = day = dd.
			case NMEA_ZDA_FIELD_INDEX_DAY:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_ZDA_FIELD_SIZE_DAY);
				// Parse day.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_time -> date = (uint8_t) value;
				break;
			// Field 3 = month = mm.
			case NMEA_ZDA_FIELD_INDEX_MONTH:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_ZDA_FIELD_SIZE_MONTH);
				// Parse month.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_time -> month = (uint8_t) value;
				break;
			// Field 4 = year = yyyy.
			case NMEA_ZDA_FIELD_INDEX_YEAR:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_ZDA_FIELD_SIZE_YEAR);
				// Parse year.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 4, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_time -> year = (uint16_t) value;
				break;
			// Unused or unknown fields.
			default:
				break;
			}
			// Increment field index and update separator.
			field_idx++;
			separator_idx = char_idx;
		}
		// Increment character index.
		char_idx++;
	}
	// Check if time is valid.
	status = _NEOM8N_time_is_valid(gps_time);
errors:
	return status;
}

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_position_is_valid(NEOM8N_position_t* gps_position) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	// Check parameters.
	if (gps_position == NULL) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check position fields.
	if ((gps_position -> lat_degrees > 89) ||
		(gps_position -> lat_minutes > 59) ||
		(gps_position -> lat_seconds > 99999) ||
		(gps_position -> long_degrees > 179) ||
		(gps_position -> long_minutes > 59) ||
		(gps_position -> long_seconds > 99999))
	{
		status = NEOM8N_ERROR_POSITION_INVALID;
	}
errors:
	return status;
}

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_parse_nmea_gga(char_t* nmea_rx_buf, NEOM8N_position_t* gps_position) {
	// Local variables
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	STRING_status_t string_status = STRING_SUCCESS;
	uint8_t char_idx = 0;
	uint8_t separator_idx = 0;
	uint8_t field_idx = 0;
	uint8_t alt_field_length = 0;
	uint8_t alt_number_of_digits = 0;
	uint8_t received_checksum = 0;
	uint8_t computed_checksum = 0;
	int32_t value = 0;
	// Check parameters.
	if ((nmea_rx_buf == NULL) || (gps_position == NULL)) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Verify checksum.
	status = _NEOM8N_get_nmea_checksum(nmea_rx_buf, &received_checksum);
	if (status != NEOM8N_SUCCESS) goto errors;
	status = _NEOM8N_compute_nmea_checksum(nmea_rx_buf, &computed_checksum);
	if (status != NEOM8N_SUCCESS) goto errors;
	if (computed_checksum != received_checksum) {
		status = NEOM8N_ERROR_CHECKSUM;
		goto errors;
	}
	// Search NMEA start character.
	while ((nmea_rx_buf[separator_idx] != NMEA_CHAR_MESSAGE_START) && (separator_idx < NMEA_RX_BUFFER_SIZE)) {
		separator_idx++;
		char_idx++;
	}
	// Extract NMEA data (see GGA message format on p.114 of NEO-M8 programming manual).
	while ((nmea_rx_buf[char_idx] != STRING_CHAR_LF) && (char_idx < NMEA_RX_BUFFER_SIZE)) {
		// Check if separator is found.
		if (nmea_rx_buf[char_idx] == NMEA_CHAR_SEPARATOR) {
			// Get current field.
			switch (field_idx) {
			// Field 0 = address = <ID><message>.
			case NMEA_GGA_FIELD_INDEX_MESSAGE:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_GGA_FIELD_SIZE_MESSAGE);
				// Check if message = 'GGA'.
				if ((nmea_rx_buf[separator_idx + 3] != 'G') || (nmea_rx_buf[separator_idx + 4] != 'G') || (nmea_rx_buf[separator_idx + 5] != 'A')) {
					status = NEOM8N_ERROR_NMEA_MESSAGE;
					goto errors;
				}
				break;
			// Field 2 = latitude = ddmm.mmmmm.
			case NMEA_GGA_FIELD_INDEX_LAT:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_GGA_FIELD_SIZE_LAT);
				// Parse degrees.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_position -> lat_degrees = (uint8_t) value;
				// Parse minutes.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 3]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_position -> lat_minutes = (uint8_t) value;
				// Parse seconds.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 6]), STRING_FORMAT_DECIMAL, 5, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_position -> lat_seconds = (uint32_t) value;
				break;
			// Field 3 = N or S.
			case NMEA_GGA_FIELD_INDEX_NS:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_GGA_FIELD_SIZE_NS);
				// Parse north flag.
				switch (nmea_rx_buf[separator_idx + 1]) {
				case NMEA_GGA_NORTH:
					(*gps_position).lat_north_flag = 1;
					break;
				case NMEA_GGA_SOUTH:
					(*gps_position).lat_north_flag = 0;
					break;
				default:
					status = NEOM8N_ERROR_NMEA_NORTH_FLAG;
					goto errors;
				}
				break;
			// Field 4 = longitude = dddmm.mmmmm.
			case NMEA_GGA_FIELD_INDEX_LONG:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_GGA_FIELD_SIZE_LONG);
				// Parse degrees.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 3, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_position -> long_degrees = (uint8_t) value;
				// Parse minutes.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 4]), STRING_FORMAT_DECIMAL, 2, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_position -> long_minutes = (uint8_t) value;
				// Parse seconds.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 7]), STRING_FORMAT_DECIMAL, 5, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_position -> long_seconds = (uint32_t) value;
				break;
			// Field 5 = E or W.
			case NMEA_GGA_FIELD_INDEX_EW:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_GGA_FIELD_SIZE_EW);
				// Parse east flag.
				switch (nmea_rx_buf[separator_idx + 1]) {
				case NMEA_GGA_EAST:
					(*gps_position).long_east_flag = 1;
					break;
				case NMEA_GGA_WEST:
					(*gps_position).long_east_flag = 0;
					break;
				default:
					status = NEOM8N_ERROR_NMEA_EAST_FLAG;
					goto errors;
				}
				break;
			// Field 9 = altitude.
			case NMEA_GGA_FIELD_INDEX_ALT:
				// Get field length.
				alt_field_length = (char_idx - separator_idx) - 1;
				// Check field length.
				if (alt_field_length == 0) {
					status = NEOM8N_ERROR_NMEA_FIELD_SIZE;
					goto errors;
				}
				// Get number of digits of integer part (search dot).
				for (alt_number_of_digits=0 ; alt_number_of_digits<alt_field_length ; alt_number_of_digits++) {
					if (nmea_rx_buf[separator_idx + 1 + alt_number_of_digits] == STRING_CHAR_DOT) {
						break; // Dot found, stop counting integer part length.
					}
				}
				// Compute integer part.
				string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, alt_number_of_digits, &value);
				STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
				gps_position -> altitude = (uint32_t) value;
				// Rounding operation if fractional part exists.
				if ((char_idx - (separator_idx + alt_number_of_digits) - 1) >= 2) {
					// Convert tenth part.
					string_status = STRING_string_to_value(&(nmea_rx_buf[separator_idx + alt_number_of_digits + 2]), STRING_FORMAT_DECIMAL, 1, &value);
					STRING_exit_error(NEOM8N_ERROR_BASE_STRING);
					if (value >= 5) {
						(*gps_position).altitude++;
					}
				}
				break;
			// Field 10 = altitude unit.
			case NMEA_GGA_FIELD_INDEX_U_ALT:
				// Check field length.
				_NEOM8N_check_field_length(NMEA_GGA_FIELD_SIZE_U_ALT);
				// Parse altitude unit.
				if (nmea_rx_buf[separator_idx + 1] != NMEA_GGA_METERS) {
					status = NEOM8N_ERROR_NMEA_UNIT;
					goto errors;
				}
				break;
			// Unused or unknown fields.
			default:
				break;
			}
			// Increment field index and update separator.
			field_idx++;
			separator_idx = char_idx;
		}
		// Increment character index.
		char_idx++;
	}
	// Check if time is valid.
	status = _NEOM8N_position_is_valid(gps_position);
errors:
	return status;
}

/*******************************************************************/
static NEOM8N_status_t _NEOM8N_select_nmea_messages(uint32_t nmea_message_id_mask) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// See p.110 for NMEA messages ID.
	uint8_t nmea_message_id[18] = {0x0A, 0x44, 0x09, 0x00, 0x01, 0x43, 0x42, 0x0D, 0x40, 0x06, 0x02, 0x07, 0x03, 0x04, 0x41, 0x0F, 0x05, 0x08};
	uint8_t nmea_message_id_idx = 0;
	// See p.174 for NEOM8N message format.
	uint8_t neom8n_cfg_msg[NEOM8N_MSG_OVERHEAD_SIZE + NEOM8N_CFG_MSG_PAYLOAD_SIZE] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t idx = 0;
	// Send commands.
	for (nmea_message_id_idx=0 ; nmea_message_id_idx<18 ; nmea_message_id_idx++) {
		// Byte 7 = is the ID of the message to enable or disable.
		neom8n_cfg_msg[7] = nmea_message_id[nmea_message_id_idx];
		// Bytes 8-13 = message rate.
		for (idx=8 ; idx<14 ; idx++) {
			neom8n_cfg_msg[idx] = ((nmea_message_id_mask & (0b1 << nmea_message_id_idx)) != 0) ? 1 : 0;
		}
		// Bytes 14-15 = NEOM8N checksum (CK_A and CK_B).
		status = _NEOM8N_compute_ubx_checksum(neom8n_cfg_msg, NEOM8N_CFG_MSG_PAYLOAD_SIZE);
		if (status != NEOM8N_SUCCESS) goto errors;
		// Send message.
		lpuart1_status = LPUART1_write(neom8n_cfg_msg, (NEOM8N_MSG_OVERHEAD_SIZE + NEOM8N_CFG_MSG_PAYLOAD_SIZE));
		LPUART1_exit_error(NEOM8N_ERROR_BASE_LPUART1);
		// Delay between messages.
		lptim1_status = LPTIM1_delay_milliseconds(100, LPTIM_DELAY_MODE_SLEEP);
		LPTIM1_exit_error(NEOM8N_ERROR_BASE_LPTIM1);
	}
errors:
	return status;
}

/*******************************************************************/
static void _NEOM8N_start(void) {
	// Start DMA.
	DMA1_CH6_stop();
	neom8n_ctx.flags.all = 0;
	neom8n_ctx.flags.fill_buffer0 = 1;
	DMA1_CH6_set_destination_address((uint32_t) &(neom8n_ctx.rx_buffer0), NMEA_RX_BUFFER_SIZE); // Start with buffer 1.
	DMA1_CH6_start();
	// Start LPUART.
	NVIC_enable_interrupt(NVIC_INTERRUPT_LPUART1, NVIC_PRIORITY_LPUART1);
}

/*******************************************************************/
static void _NEOM8N_stop(void) {
	// Stop DMA.
	DMA1_CH6_stop();
	// Stop LPUART.
	NVIC_disable_interrupt(NVIC_INTERRUPT_LPUART1);
}

/*** NEOM8N functions ***/

/*******************************************************************/
NEOM8N_status_t NEOM8N_init(void) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	uint32_t idx = 0;
#ifdef HW1_1
	// Init backup pin.
	GPIO_configure(&GPIO_GPS_VBCKP, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Init LPUART and DMA.
	lpuart1_status = LPUART1_init(&_NEOM8N_usart_cm_irq_callback);
	LPUART1_exit_error(NEOM8N_ERROR_BASE_LPUART1);
	DMA1_CH6_init(&_NEOM8N_dma_tc_irq_callback);
errors:
	// Init context.
	for (idx=0 ; idx<NMEA_RX_BUFFER_SIZE ; idx++) neom8n_ctx.rx_buffer0[idx] = 0;
	for (idx=0 ; idx<NMEA_RX_BUFFER_SIZE ; idx++) neom8n_ctx.rx_buffer1[idx] = 0;
	neom8n_ctx.gga_same_altitude_count = 0;
	neom8n_ctx.gga_previous_altitude = 0;
	return status;
}

/*******************************************************************/
void NEOM8N_de_init(void) {
	// Release LPUART and DMA.
	DMA1_CH6_de_init();
	LPUART1_de_init();
}

/*******************************************************************/
NEOM8N_status_t NEOM8N_get_time(RTC_time_t* gps_time, uint32_t timeout_seconds, uint32_t* fix_duration_seconds) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	uint8_t valid_data_flag = 0;
	uint8_t external_wakeup_count = 0;
	// Check parameters.
	if ((gps_time == NULL) || (fix_duration_seconds == NULL)) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (timeout_seconds < NEOM8N_TIMEOUT_SECONDS_MIN) {
		status = NEOM8N_ERROR_TIMEOUT;
		goto errors;
	}
	// Reset fix duration.
	(*fix_duration_seconds) = 0;
	// Select ZDA message to get complete date and time.
	status = _NEOM8N_select_nmea_messages(NMEA_ZDA_MASK);
	if (status != NEOM8N_SUCCESS) goto errors;
	// Start NMEA reception.
	_NEOM8N_start();
	// Loop until data is retrieved or timeout expired.
	while ((*fix_duration_seconds) < timeout_seconds) {
		// Wait for NMEA frame.
		while (neom8n_ctx.flags.rx_irq == 0) {
			// Enter sleep mode.
			PWR_enter_sleep_mode();
			// Check wake-up reason.
			if (neom8n_ctx.flags.rx_irq == 0) {
				external_wakeup_count++;
				if (external_wakeup_count > NEOM8N_EXTERNAL_WAKEUP_COUNT_LIMIT) {
					status = NEOM8N_ERROR_NMEA_FRAME_RECEPTION;
					goto errors;
				}
			}
		}
		// Clear flag.
		neom8n_ctx.flags.rx_irq = 0;
		external_wakeup_count = 0;
		// Update fix duration.
		(*fix_duration_seconds)++; // NMEA frames are sent every seconds.
		// Check line end flag to trigger parsing operation.
		if (neom8n_ctx.flags.line_end != 0) {
			// Decode incoming NMEA message.
			if (neom8n_ctx.flags.fill_buffer0 != 0) {
				status = _NEOM8N_parse_nmea_zda((char_t*) neom8n_ctx.rx_buffer1, gps_time); // Buffer 1 is currently filled by DMA, buffer 2 is available for parsing.
			}
			else {
				status = _NEOM8N_parse_nmea_zda((char_t*) neom8n_ctx.rx_buffer0, gps_time); // Buffer 2 is currently filled by DMA, buffer 1 is available for parsing.
			}
			// Wait for next message.
			neom8n_ctx.flags.line_end = 0;
			// Check decoding result.
			if (status == NEOM8N_SUCCESS) {
				valid_data_flag = 1;
				break;
			}
		}
		IWDG_reload();
	}
	status = (valid_data_flag != 0) ? NEOM8N_SUCCESS : NEOM8N_ERROR_TIME_TIMEOUT;
errors:
	_NEOM8N_stop();
	return status;
}

/*******************************************************************/
NEOM8N_status_t NEOM8N_get_position(NEOM8N_position_t* gps_position, uint32_t timeout_seconds, uint8_t altitude_stability_filter_count, uint32_t* fix_duration_seconds) {
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	uint8_t valid_data_flag = 0;
	uint8_t external_wakeup_count = 0;
	NEOM8N_position_t local_position;
	// Check parameters.
	if ((gps_position == NULL) || (fix_duration_seconds == NULL)) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (timeout_seconds < NEOM8N_TIMEOUT_SECONDS_MIN) {
		status = NEOM8N_ERROR_TIMEOUT;
		goto errors;
	}
	// Reset altitude filter.
	neom8n_ctx.gga_same_altitude_count = 0;
	neom8n_ctx.gga_previous_altitude = 0;
	// Reset fix duration.
	(*fix_duration_seconds) = 0;
	// Select GGA message to get complete position.
	status = _NEOM8N_select_nmea_messages(NMEA_GGA_MASK);
	if (status != NEOM8N_SUCCESS) goto errors;
	// Start NMEA reception.
	_NEOM8N_start();
	// Loop until data is retrieved or timeout expired.
	while ((*fix_duration_seconds) < timeout_seconds) {
		// Wait for NMEA frame.
		while (neom8n_ctx.flags.rx_irq == 0) {
			// Enter sleep mode.
			PWR_enter_sleep_mode();
			// Check wake-up reason.
			if (neom8n_ctx.flags.rx_irq == 0) {
				external_wakeup_count++;
				if (external_wakeup_count > NEOM8N_EXTERNAL_WAKEUP_COUNT_LIMIT) {
					status = NEOM8N_ERROR_NMEA_FRAME_RECEPTION;
					goto errors;
				}
			}
		}
		// Clear flag.
		neom8n_ctx.flags.rx_irq = 0;
		external_wakeup_count = 0;
		// Update fix duration.
		(*fix_duration_seconds)++; // NMEA frames are sent every seconds.
		// Check line end flag to trigger parsing process.
		if (neom8n_ctx.flags.line_end != 0) {
			// Decode incoming NMEA message.
			if (neom8n_ctx.flags.fill_buffer0 != 0) {
				// Buffer 1 is currently filled by DMA, buffer 2 is available for parsing.
				status = _NEOM8N_parse_nmea_gga((char_t*) neom8n_ctx.rx_buffer1, &local_position);
			}
			else {
				// Buffer 2 is currently filled by DMA, buffer 1 is available for parsing.
				status = _NEOM8N_parse_nmea_gga((char_t*) neom8n_ctx.rx_buffer0, &local_position);
			}
			// Check decoding result.
			if (status == NEOM8N_SUCCESS) {
				// Store valid data.
				(gps_position -> lat_degrees) = local_position.lat_degrees;
				(gps_position -> lat_minutes) = local_position.lat_minutes;
				(gps_position -> lat_seconds) = local_position.lat_seconds;
				(gps_position -> lat_north_flag) = local_position.lat_north_flag;
				(gps_position -> long_degrees) = local_position.long_degrees;
				(gps_position -> long_minutes) = local_position.long_minutes;
				(gps_position -> long_seconds) = local_position.long_seconds;
				(gps_position -> long_east_flag) = local_position.long_east_flag;
				(gps_position -> altitude) = local_position.altitude;
				// Update flag.
				valid_data_flag = 1;
				// Directly exit if the filter is disabled.
				if (altitude_stability_filter_count < 2) break;
				// Manage altitude stability count.
				if ((gps_position -> altitude) == neom8n_ctx.gga_previous_altitude) {
					neom8n_ctx.gga_same_altitude_count++;
					if (neom8n_ctx.gga_same_altitude_count >= (altitude_stability_filter_count - 1)) break;
				}
				else {
					neom8n_ctx.gga_same_altitude_count = 0;
				}
				// Update previous altitude.
				neom8n_ctx.gga_previous_altitude = (gps_position -> altitude);
			}
			else {
				neom8n_ctx.gga_same_altitude_count = 0;
			}
			// Wait for next message.
			neom8n_ctx.flags.line_end = 0;
		}
		IWDG_reload();
	}
	// Force success status if any valid data has been retrieved.
	status = (valid_data_flag != 0) ? NEOM8N_SUCCESS : NEOM8N_ERROR_POSITION_TIMEOUT;
errors:
	_NEOM8N_stop();
	return status;
}

/*******************************************************************/
NEOM8N_status_t NEOM8N_configure_timepulse(NEOM8N_timepulse_config_t* timepulse_config){
	// Local variables.
	NEOM8N_status_t status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	uint64_t pulse_length_ratio = 0;
	// See p.222 for NEOM8N message format.
	uint8_t neom8n_cfg_tp5[NEOM8N_MSG_OVERHEAD_SIZE + NEOM8N_CFG_TP5_PAYLOAD_SIZE] = {
		0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, // Header.
		0x00, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Payload.
		0, 0 // Checksum.
	};
	// Check parameters.
	if (timepulse_config == NULL) {
		status = NEOM8N_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if ((timepulse_config -> frequency_hz) > NEOM8N_TIMEPULSE_FREQUENCY_MAX_HZ) {
		status = NEOM8N_ERROR_TIMEPULSE_FREQUENCY;
		goto errors;
	}
	if ((timepulse_config -> duty_cycle_percent) > 100) {
		status = NEOM8N_ERROR_TIMEPULSE_DUTY_CYCLE;
		goto errors;
	}
	// Frequency
	neom8n_cfg_tp5[14] = (uint8_t) (((timepulse_config -> frequency_hz) >> 0) & 0xFF);
	neom8n_cfg_tp5[15] = (uint8_t) (((timepulse_config -> frequency_hz) >> 8) & 0xFF);
	neom8n_cfg_tp5[16] = (uint8_t) (((timepulse_config -> frequency_hz) >> 16) & 0xFF);
	neom8n_cfg_tp5[17] = (uint8_t) (((timepulse_config -> frequency_hz) >> 24) & 0xFF);
	// Pulse length radio.
	pulse_length_ratio = ((uint64_t) (timepulse_config -> duty_cycle_percent)) * ((uint64_t) (0xFFFFFFFF));
	pulse_length_ratio /= 100;
	neom8n_cfg_tp5[22] = (uint8_t) ((pulse_length_ratio >> 0) & 0xFF);
	neom8n_cfg_tp5[23] = (uint8_t) ((pulse_length_ratio >> 8) & 0xFF);
	neom8n_cfg_tp5[24] = (uint8_t) ((pulse_length_ratio >> 16) & 0xFF);
	neom8n_cfg_tp5[25] = (uint8_t) ((pulse_length_ratio >> 24) & 0xFF);
	// Flags.
	neom8n_cfg_tp5[34] = ((timepulse_config -> active) == 0) ? 0x4A : 0x4B;
	neom8n_cfg_tp5[35] = 0;
	neom8n_cfg_tp5[36] = 0;
	neom8n_cfg_tp5[37] = 0;
	// NEOM8N checksum (CK_A and CK_B).
	status = _NEOM8N_compute_ubx_checksum(neom8n_cfg_tp5, NEOM8N_CFG_TP5_PAYLOAD_SIZE);
	if (status != NEOM8N_SUCCESS) goto errors;
	// Send message.
	lpuart1_status = LPUART1_write(neom8n_cfg_tp5, (NEOM8N_MSG_OVERHEAD_SIZE + NEOM8N_CFG_TP5_PAYLOAD_SIZE));
	LPUART1_exit_error(NEOM8N_ERROR_BASE_LPUART1);
errors:
	return status;
}
