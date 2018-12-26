/*
 * geoloc.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludo
 */

#include "geoloc.h"

#include "lpuart.h"
#include "neom8n.h"
#include "nvm.h"
#include "tim.h"

/*** GEOLOC local macros ***/

// Length of geolocation Sigfox frame in bytes.
#define GEOLOC_SIGFOX_DATA_LENGTH	12

/*** GEOLOC local structures ***/

// Status bits indexes.
typedef enum {
	GEOLOC_STATUS_BYTE_GGA_PARSING_SUCCESS_BIT_INDEX,
	GEOLOC_STATUS_BYTE_GGA_DATA_VALID_BIT_INDEX,
	GEOLOC_STATUS_BYTE_GGA_TIMEOUT_BIT_INDEX,
	GEOLOC_STATUS_BYTE_ZDA_PARSING_SUCCESS_BIT_INDEX,
	GEOLOC_STATUS_BYTE_ZDA_DATA_VALID_BIT_INDEX,
	GEOLOC_STATUS_BYTE_ZDA_TIMEOUT_BIT_INDEX
} GEOLOC_StatusBitsIndexes;

// State machine.
typedef enum {
	GEOLOC_STATE_GET_POSITION,
	GEOLOC_STATE_GET_TIMESTAMP,
	GEOLOC_STATE_OFF,
	GEOLOC_STATE_SIGFOX,
	GEOLOC_STATE_END
} GEOLOC_State;

typedef struct {
	// State.
	GEOLOC_State geoloc_state;
	// Position data.
	Position gps_position;
	// Fix duration and timeout.
	unsigned char geoloc_timeout_seconds; // Geolocation fix timeout in seconds (retrieved from NVM).
	unsigned char geoloc_fix_start_time_seconds; // Absolute time (since MCU start-up) of fix start in seconds.
	unsigned char geoloc_fix_duration_seconds; // Fix duration in seconds.
	// Sigfox data.
	unsigned char geoloc_sigfox_data[GEOLOC_SIGFOX_DATA_LENGTH];
	// Status byte.
	unsigned char geoloc_status_byte;
} GEOLOC_Context;

/*** GEOLOC local global variables ***/

static GEOLOC_Context geoloc_ctx;

/*** GEOLOC local functions ***/

/* INIT GEOLOC APPLICATION.
 * @param:	None.
 * @return:	NOne.
 */
void GEOLOC_Init(void) {

	/* Init context */
	// State.
	geoloc_ctx.geoloc_state = GEOLOC_STATE_GET_POSITION;
	// Position data.
	geoloc_ctx.gps_position.lat_degrees = 0;
	geoloc_ctx.gps_position.lat_minutes = 0;
	geoloc_ctx.gps_position.lat_seconds = 0;
	geoloc_ctx.gps_position.lat_north = 0;
	geoloc_ctx.gps_position.long_degrees = 0;
	geoloc_ctx.gps_position.long_minutes = 0;
	geoloc_ctx.gps_position.long_seconds = 0;
	geoloc_ctx.gps_position.long_east = 0;
	geoloc_ctx.gps_position.altitude = 0;
	// Fix duration and timeout.
	NVM_ReadByte(NVM_GPS_TIMEOUT_ADDRESS_OFFSET, &geoloc_ctx.geoloc_timeout_seconds);
	geoloc_ctx.geoloc_timeout_seconds = 60; // Bypass for debug.
	geoloc_ctx.geoloc_fix_start_time_seconds = 0;
	geoloc_ctx.geoloc_fix_duration_seconds = 0;
	// Sigfox.
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<GEOLOC_SIGFOX_DATA_LENGTH ; byte_idx++) geoloc_ctx.geoloc_sigfox_data[byte_idx] = 0;
	// Status byte.
	geoloc_ctx.geoloc_status_byte = 0;

	/* Init GPS module */
	NEOM8N_Init();
	LPUART1_PowerOn();
}

/* BUILD SIGFOX DATA STARTING FROM GPS POSITION AND STATUS.
 * @param:	None.
 * @return:	None.
 */
void GEOLOC_BuildSigfoxData(void) {
	// Latitude degrees.
	geoloc_ctx.geoloc_sigfox_data[0] = geoloc_ctx.gps_position.lat_degrees;
	// Latitude minutes.
	geoloc_ctx.gps_position.lat_minutes = (geoloc_ctx.gps_position.lat_minutes & 0x3F); // Ensure minutes are on 6 bits.
	geoloc_ctx.geoloc_sigfox_data[1] |= (geoloc_ctx.gps_position.lat_minutes << 2);
	// Latitude seconds.
	geoloc_ctx.gps_position.lat_seconds = (geoloc_ctx.gps_position.lat_seconds & 0x0001FFFF); // Ensure seconds are on 17 bits.
	geoloc_ctx.geoloc_sigfox_data[1] |= ((geoloc_ctx.gps_position.lat_seconds & 0x00018000) >> 15); // Keep bits 15-16.
	geoloc_ctx.geoloc_sigfox_data[2] |= ((geoloc_ctx.gps_position.lat_seconds & 0x00007F80) >> 7); // Keep bits 7-14.
	geoloc_ctx.geoloc_sigfox_data[3] |= ((geoloc_ctx.gps_position.lat_seconds & 0x0000007F) << 1); // Keep bits 0-6.
	// Latitude N/S.
	geoloc_ctx.gps_position.lat_north = (geoloc_ctx.gps_position.lat_north & 0x01); // Ensure north flag is a bit.
	geoloc_ctx.geoloc_sigfox_data[3] |= geoloc_ctx.gps_position.lat_north;
	// Longitude degrees.
	geoloc_ctx.geoloc_sigfox_data[4] = geoloc_ctx.gps_position.long_degrees;
	// Longitude minutes.
	geoloc_ctx.gps_position.long_minutes = (geoloc_ctx.gps_position.long_minutes & 0x3F); // Ensure minutes are on 6 bits.
	geoloc_ctx.geoloc_sigfox_data[5] |= (geoloc_ctx.gps_position.long_minutes << 2);
	// Longitude seconds.
	geoloc_ctx.gps_position.long_seconds = (geoloc_ctx.gps_position.long_seconds & 0x0001FFFF); // Ensure seconds are on 17 bits.
	geoloc_ctx.geoloc_sigfox_data[5] |= ((geoloc_ctx.gps_position.long_seconds & 0x00018000) >> 15); // Keep bits 15-16.
	geoloc_ctx.geoloc_sigfox_data[6] |= ((geoloc_ctx.gps_position.long_seconds & 0x00007F80) >> 7); // Keep bits 7-14.
	geoloc_ctx.geoloc_sigfox_data[7] |= ((geoloc_ctx.gps_position.long_seconds & 0x0000007F) << 1); // Keep bits 0-6.
	// Longitude E/O.
	geoloc_ctx.gps_position.long_east = (geoloc_ctx.gps_position.long_east & 0x01); // Ensure east flag is a bit.
	geoloc_ctx.geoloc_sigfox_data[7] |= geoloc_ctx.gps_position.long_east;
	// Altitude in m.
	geoloc_ctx.gps_position.altitude = (geoloc_ctx.gps_position.altitude & 0x0000FFFF); // Ensure altitude is on 16 bits.
	geoloc_ctx.geoloc_sigfox_data[8] |= ((geoloc_ctx.gps_position.altitude & 0x0000FF00) >> 8);
	geoloc_ctx.geoloc_sigfox_data[9] |= (geoloc_ctx.gps_position.altitude & 0x000000FF);
	// Fix duration.
	geoloc_ctx.geoloc_sigfox_data[10] = geoloc_ctx.geoloc_fix_duration_seconds;
	// Status byte.
	geoloc_ctx.geoloc_sigfox_data[11] = geoloc_ctx.geoloc_status_byte;
}

/*** GEOLOC functions ***/

/* MAIN ROUTINE OF GEOLOC MODULE.
 * @param gps_timestamp:		Pointer to GPS timestamp that will contain current GPS timestamp if success.
 * @param timestamp_retrieved:	'1' if GPS timestamp xas successfully retrieved, '0' otherwise.
 * @return:						None.
 */
void GEOLOC_Process(Timestamp* gps_timestamp, unsigned char* timestamp_retrieved) {

	GEOLOC_Init();
	(*timestamp_retrieved) = 0;
	unsigned char neom8n_result = 0;

	/* Loop until GEOLOC processing is done */
	while (geoloc_ctx.geoloc_state != GEOLOC_STATE_END) {

		/* Perform state machine */
		switch (geoloc_ctx.geoloc_state) {

		/* Retrieve GPS position */
		case GEOLOC_STATE_GET_POSITION:
			// Start NEOM8N message parsing.
			geoloc_ctx.geoloc_fix_start_time_seconds = TIM22_GetSeconds();
			neom8n_result = NEOM8N_GetPosition(&geoloc_ctx.gps_position, geoloc_ctx.geoloc_timeout_seconds);
			switch (neom8n_result) {
			case NEOM8N_SUCCESS:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = TIM22_GetSeconds()-geoloc_ctx.geoloc_fix_start_time_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_GGA_PARSING_SUCCESS_BIT_INDEX);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_GGA_DATA_VALID_BIT_INDEX);
				// Compute next state.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_GET_TIMESTAMP;
				break;
			case NEOM8N_INVALID_DATA:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_GGA_TIMEOUT_BIT_INDEX);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_GGA_PARSING_SUCCESS_BIT_INDEX);
				// Compute next state.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
				break;
			case NEOM8N_TIMEOUT:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_GGA_TIMEOUT_BIT_INDEX);
				// Compute next state.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
				break;
			default:
				// Unexpected result.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
				break;
			}
			break;

		/* Retrieve GPS timestamp used for RTC/HWT calibration (done here since GPS module is allready hot) */
		case GEOLOC_STATE_GET_TIMESTAMP:
			// Start NEOM8N message parsing.
			neom8n_result = NEOM8N_GetTimestamp(gps_timestamp, (geoloc_ctx.geoloc_timeout_seconds-geoloc_ctx.geoloc_fix_duration_seconds));
			switch (neom8n_result) {
			case NEOM8N_SUCCESS:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = TIM22_GetSeconds()-geoloc_ctx.geoloc_fix_start_time_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_ZDA_PARSING_SUCCESS_BIT_INDEX);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_ZDA_DATA_VALID_BIT_INDEX);
				(*timestamp_retrieved) = 1;
				break;
			case NEOM8N_INVALID_DATA:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_ZDA_TIMEOUT_BIT_INDEX);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_ZDA_PARSING_SUCCESS_BIT_INDEX);
				break;
			case NEOM8N_TIMEOUT:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_STATUS_BYTE_ZDA_TIMEOUT_BIT_INDEX);
				break;
			default:
				// Unexpected result.
				break;
			}
			// Compute next state.
			geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
			break;

		/* Switch GPS module off */
		case GEOLOC_STATE_OFF:
			// Stop LPUART, DMA and GPS.
			LPUART1_PowerOff();
			// Send Sigfox frame to report position.
			geoloc_ctx.geoloc_state = GEOLOC_STATE_SIGFOX;
			break;

		/* Send Sigfox geolocation frame */
		case GEOLOC_STATE_SIGFOX:
			// Build frame.
			GEOLOC_BuildSigfoxData();
			// TBD: send.
			geoloc_ctx.geoloc_state = GEOLOC_STATE_END;
			break;

		/* End of processing */
		case GEOLOC_STATE_END:
			break;

		/* Unknown state */
		default:
			//NEOM8N_Off();
			geoloc_ctx.geoloc_state = GEOLOC_STATE_END;
			break;
		}
	}
}
