/*
 * geoloc.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#include "geoloc.h"

#include "hwt.h"
#include "neom8n.h"
#include "nvm.h"
#include "pwr.h"
#include "tim.h"

/*** GEOLOC local macros ***/

#define GEOLOC_TIMEOUT_SECONDS_MIN_VALUE			60 // GEOLOC fix minimum time-out in seconds.
#define GEOLOC_TIMEOUT_SECONDS_DEFAULT_VALUE		60 // Default GEOLOC fix time-out in seconds (in case if NVM read failure).
#define GEOLOC_TIMEOUT_SECONDS_MAX_VALUE			180 // GEOLOC fix maximum time-out in seconds.

#define GEOLOC_SIGFOX_DATA_LENGTH					11 // Length of GEOLOC Sigfox frame in bytes.

// Assuming average number of days per year is 365.25.
#define AVERAGE_NUMBER_OF_SECONDS_PER_YEAR		31557600
#define AVERAGE_NUMBER_OF_SECONDS_PER_MONTH		2629800
#define NUMBER_OF_SECONDS_PER_DAY				86400
#define NUMBER_OF_SECONDS_PER_HOUR				3600
#define NUMBER_OF_SECONDS_PER_MINUTE			60

/*** GEOLOC local structures ***/

// Status bits offsets.
typedef enum {
	GEOLOC_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET,
	GEOLOC_CURRENT_TIMESTAMP_PARSING_SUCCESS_BIT_OFFSET,
	GEOLOC_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET,
	GEOLOC_POSITION_PARSING_SUCCESS_BIT_OFFSET,
	GEOLOC_POSITION_DATA_VALID_BIT_OFFSET,
	GEOLOC_TIMEOUT_FLAG_BIT_OFFSET,
} GEOLOC_StatusBitsOffsets;

// ____________________________________________________________________________
// |                                                                           |
// |                             GEOLOC status bits                               |
// |___________________________________________________________________________|
// | 7 | 6 | 5       | 4        | 3        | 2         | 1         | 0         |
// |   |   |         |          |          |           |           |           |
// |   |   | Timeout | Position | Position | Current   | Current   | Previous  |
// |   |   | flag    | data     | parsing  | timestamp | timestamp | timestamp |
// |   |   |         | valid    | success  | data      | parsing   | data      |
// |   |   |         |          |          | valid     | success   | valid     |
// |___|___|_________|__________|__________|___________|___________|___________|


// State machine.
typedef enum {
	GEOLOC_STATE_INIT,
	GEOLOC_STATE_GET_TIMESTAMP,
	GEOLOC_STATE_GET_POSITION,
	GEOLOC_STATE_OFF,
	GEOLOC_STATE_SIGFOX,
#ifdef IM_HWT
	GEOLOC_STATE_CALIBRATE_HARDWARE_TIMER,
#endif
	GEOLOC_STATE_END
} GEOLOC_State;

typedef struct {
	// State.
	GEOLOC_State geoloc_state;
	// Fix duration and timeout.
	unsigned char geoloc_timeout_seconds;					// GEOLOC fix timeout in seconds (retrieved from NVM and configurable via downlink).
	unsigned char geoloc_fix_start_time_seconds;			// Absolute time (since MCU start-up) of fix start in seconds.
	unsigned char geoloc_fix_duration_seconds;				// Fix duration in seconds (timestamp or position).
	// GEOLOC period.
	unsigned char geoloc_day_count;						// Current number of days since previous GEOLOC position fix.
	unsigned char geoloc_period_days;						// GEOLOC position fix period in days.
	// Sigfox.
	unsigned char geoloc_sigfox_required;
	unsigned char geoloc_sigfox_data[GEOLOC_SIGFOX_DATA_LENGTH];
	// Status byte.
	unsigned char geoloc_status_byte;						// Set to '1' as soon as a timeout condition is met.

} GEOLOC_Context;

/*** GEOLOC local global variables ***/

static GEOLOC_Context geoloc_ctx;
static GPS_TimestampData geoloc_previous_timestamp;
static GPS_TimestampData geoloc_current_timestamp;
static GPS_PositionData geoloc_position;

/*** GEOLOC local functions ***/

/* COMPARE PREVIOUS AND CURRENT GEOLOC DATES.
 * @param:	None.
 * @return:	None.
 */
unsigned char GEOLOC_DateChanged(void) {
	unsigned char result = 0;
	// Compare all fields.
	if ((geoloc_previous_timestamp.date_day != geoloc_current_timestamp.date_day) ||
		(geoloc_previous_timestamp.date_month != geoloc_current_timestamp.date_month) ||
		(geoloc_previous_timestamp.date_year != geoloc_current_timestamp.date_year)) {
		result = 1;
	}
	// Note: the result is positive when previous data is invalid (0 value in all fields) and current one is valid.
	// This way, a GEOLOC position is computed when previous GEOLOC timestamp failed, whatever the day.
	return result;
}

/* COMPUTE EFFECTIVE DURATION OF HARDWARE TIMER.
 * @param:							None.
 * @return delta_timestamp_seconds:	Number of seconds between previous ans current GEOLOC timestamp (assuming there are valid).
 */
unsigned int GEOLOC_GetDeltaTimestampSeconds(void) {
	unsigned int delta_timestamp_seconds = 0;
	// Convert previous timestamp into a number of seconds since January 1st of the previous year.
	unsigned int previous_timestamp_seconds = (geoloc_previous_timestamp.date_month-1)*AVERAGE_NUMBER_OF_SECONDS_PER_MONTH;
	previous_timestamp_seconds += (geoloc_previous_timestamp.date_day-1)*NUMBER_OF_SECONDS_PER_DAY;
	previous_timestamp_seconds += (geoloc_previous_timestamp.time_hours)*NUMBER_OF_SECONDS_PER_HOUR;
	previous_timestamp_seconds += (geoloc_previous_timestamp.time_minutes)*NUMBER_OF_SECONDS_PER_MINUTE;
	previous_timestamp_seconds += geoloc_previous_timestamp.time_seconds;
	// Compute number of year(s) between previous and current timestamp.
	unsigned int delta_year = 0;
	if ((geoloc_current_timestamp.date_year == 0) && (geoloc_previous_timestamp.date_year == 99)) {
		delta_year = 1;
	}
	else {
		unsigned short previous_year = geoloc_previous_timestamp.date_year;
		unsigned short current_year = geoloc_current_timestamp.date_year;
		delta_year = current_year-previous_year;
	}
	// Convert current timestamp into a number of seconds since January 1st of the previous year.
	unsigned int current_timestamp_seconds = delta_year*AVERAGE_NUMBER_OF_SECONDS_PER_YEAR;
	current_timestamp_seconds += (geoloc_current_timestamp.date_month-1)*AVERAGE_NUMBER_OF_SECONDS_PER_MONTH;
	current_timestamp_seconds += (geoloc_current_timestamp.date_day-1)*NUMBER_OF_SECONDS_PER_DAY;
	current_timestamp_seconds += (geoloc_current_timestamp.time_hours)*NUMBER_OF_SECONDS_PER_HOUR;
	current_timestamp_seconds += (geoloc_current_timestamp.time_minutes)*NUMBER_OF_SECONDS_PER_MINUTE;
	current_timestamp_seconds += geoloc_current_timestamp.time_seconds;
	// Compensate GEOLOC fix durations.
	unsigned int current_mcu_start_time_seconds = current_timestamp_seconds-geoloc_current_timestamp.mcu_time_seconds;
	unsigned int previous_mcu_start_time_seconds = previous_timestamp_seconds-geoloc_previous_timestamp.mcu_time_seconds;
	// Compute delta in seconds.
	delta_timestamp_seconds = current_mcu_start_time_seconds - previous_mcu_start_time_seconds;
	//
	return delta_timestamp_seconds;
}

/* GET GEOLOC PARAMETERS STORED IN NVM.
 * @param:	None.
 * @return:	None.
 */
void GEOLOC_GetNvmParameters(void) {
	// Previous GEOLOC timestamp.
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_DAY_ADDRESS_OFFSET, &geoloc_previous_timestamp.date_day);
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_MONTH_ADDRESS_OFFSET, &geoloc_previous_timestamp.date_month);
	unsigned char year_msb;
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET, &year_msb);
	unsigned char year_lsb;
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET+1, &year_lsb);
	geoloc_previous_timestamp.date_year = (year_msb << 8) + year_lsb;
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_HOURS_ADDRESS_OFFSET, &geoloc_previous_timestamp.time_hours);
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_MINUTES_ADDRESS_OFFSET, &geoloc_previous_timestamp.time_minutes);
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_SECONDS_ADDRESS_OFFSET, &geoloc_previous_timestamp.time_seconds);
	NVM_ReadByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_MCU_TIME_ADDRESS_OFFSET, &geoloc_previous_timestamp.mcu_time_seconds);
	// GEOLOC timeout.
	NVM_ReadByte(NVM_GEOLOC_TIMEOUT_SECONDS_ADDRESS_OFFSET, &geoloc_ctx.geoloc_timeout_seconds);
	if ((geoloc_ctx.geoloc_timeout_seconds < GEOLOC_TIMEOUT_SECONDS_MIN_VALUE) || (geoloc_ctx.geoloc_timeout_seconds > GEOLOC_TIMEOUT_SECONDS_MAX_VALUE)) {
		geoloc_ctx.geoloc_timeout_seconds = GEOLOC_TIMEOUT_SECONDS_DEFAULT_VALUE;
	}
	// Day count.
	NVM_ReadByte(NVM_GEOLOC_DAY_COUNT_ADDRESS_OFFSET, &geoloc_ctx.geoloc_day_count);
	if (geoloc_ctx.geoloc_day_count > GEOLOC_PERIOD_DAYS_MAX_VALUE) {
		geoloc_ctx.geoloc_day_count = 0;
	}
	// GEOLOC position fix period.
	NVM_ReadByte(NVM_GEOLOC_PERIOD_DAYS_ADDRESS_OFFSET, &geoloc_ctx.geoloc_period_days);
	if ((geoloc_ctx.geoloc_period_days < GEOLOC_PERIOD_DAYS_MIN_VALUE) || (geoloc_ctx.geoloc_period_days > GEOLOC_PERIOD_DAYS_MAX_VALUE)) {
		geoloc_ctx.geoloc_period_days = GEOLOC_PERIOD_DAYS_DEFAULT_VALUE;
	}
}

/* INIT GEOLOC APPLICATION.
 * @param:	None.
 * @return:	NOne.
 */
void GEOLOC_Init(void) {

	/* Init context */
	// State.
	geoloc_ctx.geoloc_state = GEOLOC_STATE_INIT;
	// Fix duration and timeout.
	geoloc_ctx.geoloc_fix_start_time_seconds = 0;
	geoloc_ctx.geoloc_fix_duration_seconds = 0;
	// Sigfox.
	geoloc_ctx.geoloc_sigfox_required = 0;
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<GEOLOC_SIGFOX_DATA_LENGTH ; byte_idx++) geoloc_ctx.geoloc_sigfox_data[byte_idx] = 0;
	// Status byte.
	geoloc_ctx.geoloc_status_byte = 0;
	if (NEOM8N_TimestampIsValid(geoloc_previous_timestamp) == 1) {
		geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET);
	}

	/* Init current GEOLOC timestamp */
	geoloc_current_timestamp.date_day = 0;
	geoloc_current_timestamp.date_month = 0;
	geoloc_current_timestamp.date_year = 0;
	geoloc_current_timestamp.time_hours = 0;
	geoloc_current_timestamp.time_minutes = 0;
	geoloc_current_timestamp.time_seconds = 0;
	geoloc_current_timestamp.mcu_time_seconds = 0;

	/* Init GEOLOC position */
	geoloc_position.lat_degrees = 0;
	geoloc_position.lat_minutes = 0;
	geoloc_position.lat_seconds = 0;
	geoloc_position.lat_north = 0;
	geoloc_position.long_degrees = 0;
	geoloc_position.long_minutes = 0;
	geoloc_position.long_seconds = 0;
	geoloc_position.long_east = 0;
	geoloc_position.altitude = 0;
}

/* BUILD SIGFOX DATA STARTING FROM GEOLOC POSITION AND STATUS.
 * @param:	None.
 * @return:	None.
 */
void GEOLOC_BuildSigfoxData(void) {

	/* Build frame */
	// Latitude degrees.
	geoloc_ctx.geoloc_sigfox_data[0] = geoloc_position.lat_degrees;
	// Latitude minutes.
	geoloc_position.lat_minutes = (geoloc_position.lat_minutes & 0x3F); // Ensure minutes are on 6 bits.
	geoloc_ctx.geoloc_sigfox_data[1] |= (geoloc_position.lat_minutes << 2);
	// Latitude seconds.
	geoloc_position.lat_seconds = (geoloc_position.lat_seconds & 0x0001FFFF); // Ensure seconds are on 17 bits.
	geoloc_ctx.geoloc_sigfox_data[1] |= ((geoloc_position.lat_seconds & 0x00018000) >> 15); // Keep bits 15-16.
	geoloc_ctx.geoloc_sigfox_data[2] |= ((geoloc_position.lat_seconds & 0x00007F80) >> 7); // Keep bits 7-14.
	geoloc_ctx.geoloc_sigfox_data[3] |= ((geoloc_position.lat_seconds & 0x0000007F) << 1); // Keep bits 0-6.
	// Latitude N/S.
	geoloc_position.lat_north = (geoloc_position.lat_north & 0x01); // Ensure north flag is a bit.
	geoloc_ctx.geoloc_sigfox_data[3] |= geoloc_position.lat_north;
	// Longitude degrees.
	geoloc_ctx.geoloc_sigfox_data[4] = geoloc_position.long_degrees;
	// Longitude minutes.
	geoloc_position.long_minutes = (geoloc_position.long_minutes & 0x3F); // Ensure minutes are on 6 bits.
	geoloc_ctx.geoloc_sigfox_data[5] |= (geoloc_position.long_minutes << 2);
	// Longitude seconds.
	geoloc_position.long_seconds = (geoloc_position.long_seconds & 0x0001FFFF); // Ensure seconds are on 17 bits.
	geoloc_ctx.geoloc_sigfox_data[5] |= ((geoloc_position.long_seconds & 0x00018000) >> 15); // Keep bits 15-16.
	geoloc_ctx.geoloc_sigfox_data[6] |= ((geoloc_position.long_seconds & 0x00007F80) >> 7); // Keep bits 7-14.
	geoloc_ctx.geoloc_sigfox_data[7] |= ((geoloc_position.long_seconds & 0x0000007F) << 1); // Keep bits 0-6.
	// Longitude E/O.
	geoloc_position.long_east = (geoloc_position.long_east & 0x01); // Ensure east flag is a bit.
	geoloc_ctx.geoloc_sigfox_data[7] |= geoloc_position.long_east;
	// Altitude in m.
	geoloc_position.altitude = (geoloc_position.altitude & 0x0000FFFF); // Ensure altitude is on 16 bits.
	geoloc_ctx.geoloc_sigfox_data[8] |= ((geoloc_position.altitude & 0x0000FF00) >> 8);
	geoloc_ctx.geoloc_sigfox_data[9] |= (geoloc_position.altitude & 0x000000FF);
	// GEOLOC fix duration.
	geoloc_ctx.geoloc_sigfox_data[10] = geoloc_ctx.geoloc_fix_duration_seconds;
}

/*** GEOLOC functions ***/

/* MAIN ROUTINE OF GEOLOC MODULE.
 * @param:	None.
 * @return:	None.
 */
void GEOLOC_Processing(void) {

	unsigned char neom8n_result = 0;

	/* Loop until GEOLOC processing is done */
	while (geoloc_ctx.geoloc_state != GEOLOC_STATE_END) {

		/* Perform state machine */
		switch (geoloc_ctx.geoloc_state) {

		/* Init */
		case GEOLOC_STATE_INIT:
			GEOLOC_GetNvmParameters();
			GEOLOC_Init();
			NEOM8N_Init();
			geoloc_ctx.geoloc_state = GEOLOC_STATE_GET_TIMESTAMP;
			break;

		/* Wait current GEOLOC timestamp */
		case GEOLOC_STATE_GET_TIMESTAMP:
			// Start NEO-M8N message parsing.
			geoloc_ctx.geoloc_fix_start_time_seconds = TIM_TimeGetSeconds();
			neom8n_result = NEOM8N_GetTimestamp(&geoloc_current_timestamp, geoloc_ctx.geoloc_timeout_seconds);
			// Parse return code.
			switch (neom8n_result) {

			case NEOM8N_SUCCESS:
				// Save current time to compute MCU start-up timestamp (used in GEOLOC_GetDeltaTimestampSeconds() function).
				geoloc_current_timestamp.mcu_time_seconds = TIM_TimeGetSeconds();
				// Update status byte.
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_CURRENT_TIMESTAMP_PARSING_SUCCESS_BIT_OFFSET);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET);
				// Compute next state.
				if (GEOLOC_DateChanged() == 1) {
					// Day changed or previous GEOLOC timestamp invalid -> increment day count and set downlink request.
					//CONF_RequestDownlink();
					geoloc_ctx.geoloc_day_count++;
					if (geoloc_ctx.geoloc_day_count == geoloc_ctx.geoloc_period_days) {
						// GEOLOC position fix period reached.
						geoloc_ctx.geoloc_day_count = 0;
						geoloc_ctx.geoloc_state = GEOLOC_STATE_GET_POSITION;
					}
					else {
						geoloc_ctx.geoloc_fix_duration_seconds = TIM_TimeGetSeconds()-geoloc_ctx.geoloc_fix_start_time_seconds;
						geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
					}
				}
				else {
					geoloc_ctx.geoloc_fix_duration_seconds = TIM_TimeGetSeconds()-geoloc_ctx.geoloc_fix_start_time_seconds;
					geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
				}
				break;

			case NEOM8N_INVALID_DATA:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_CURRENT_TIMESTAMP_PARSING_SUCCESS_BIT_OFFSET);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_TIMEOUT_FLAG_BIT_OFFSET);
				// Compute next state.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
				break;

			case NEOM8N_TIMEOUT:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_TIMEOUT_FLAG_BIT_OFFSET);
				// Compute next state.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
				break;

			default:
				break;
			}
			break;

		/* Wait GEOLOC position */
		case GEOLOC_STATE_GET_POSITION:
			// Start NEO-M8N message parsing.
			neom8n_result = NEOM8N_GetPosition(&geoloc_position, geoloc_ctx.geoloc_timeout_seconds);
			switch (neom8n_result) {

			case NEOM8N_SUCCESS:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = TIM_TimeGetSeconds()-geoloc_ctx.geoloc_fix_start_time_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_POSITION_PARSING_SUCCESS_BIT_OFFSET);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_POSITION_DATA_VALID_BIT_OFFSET);
				geoloc_ctx.geoloc_sigfox_required = 1;
				break;

			case NEOM8N_INVALID_DATA:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_TIMEOUT_FLAG_BIT_OFFSET);
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_POSITION_PARSING_SUCCESS_BIT_OFFSET);
				break;

			case NEOM8N_TIMEOUT:
				// Update status.
				geoloc_ctx.geoloc_fix_duration_seconds = geoloc_ctx.geoloc_timeout_seconds;
				geoloc_ctx.geoloc_status_byte |= (0b1 << GEOLOC_TIMEOUT_FLAG_BIT_OFFSET);
				break;

			default:
				break;
			}
			// Compute next state.
			geoloc_ctx.geoloc_state = GEOLOC_STATE_OFF;
			break;

		/* Switch LPUART and GEOLOC module off */
		case GEOLOC_STATE_OFF:
			// Switch GEOLOC module off.
			NEOM8N_Off();
			// Store current GEOLOC timestamp in NVM whatever the result.
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_DAY_ADDRESS_OFFSET, geoloc_current_timestamp.date_day);
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_MONTH_ADDRESS_OFFSET, geoloc_current_timestamp.date_month);
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET, ((geoloc_current_timestamp.date_year & 0x0000FF00) >> 8));
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET+1, (geoloc_current_timestamp.date_year & 0x000000FF));
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_HOURS_ADDRESS_OFFSET, geoloc_current_timestamp.time_hours);
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_MINUTES_ADDRESS_OFFSET, geoloc_current_timestamp.time_minutes);
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_SECONDS_ADDRESS_OFFSET, geoloc_current_timestamp.time_seconds);
			NVM_WriteByte(NVM_GEOLOC_PREVIOUS_TIMESTAMP_MCU_TIME_ADDRESS_OFFSET, geoloc_current_timestamp.mcu_time_seconds);
			// Update status byte in NVM.
			NVM_WriteByte(NVM_GEOLOC_STATUS_ADDRESS_OFFSET, geoloc_ctx.geoloc_status_byte);
			// Update day count in NVM.
			NVM_WriteByte(NVM_GEOLOC_DAY_COUNT_ADDRESS_OFFSET, geoloc_ctx.geoloc_day_count);
			// Compute next state.
			if (geoloc_ctx.geoloc_sigfox_required == 1) {
				// Send Sigfox frame to report position.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_SIGFOX;
			}
			else {
#ifdef IM_HWT
				if (((geoloc_ctx.geoloc_status_byte & (0b1 << GEOLOC_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
				    ((geoloc_ctx.geoloc_status_byte & (0b1 << GEOLOC_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
					(HWT_WasResetReason())) {
					// Compute effective duration and calibrate hardware timer thanks to GEOLOC timestamp.
					geoloc_ctx.geoloc_state = GEOLOC_STATE_CALIBRATE_HARDWARE_TIMER;
				}
				else {
					// Neither Sigfox frame to send nore calibration required, GEOLOC processing is done.
					geoloc_ctx.geoloc_state = GEOLOC_STATE_END;
				}
#else
				geoloc_ctx.geoloc_state = GEOLOC_STATE_END;
#endif
			}
			break;

		/* Send Sigfox GEOLOC frame */
		case GEOLOC_STATE_SIGFOX:
			// Build frame.
			GEOLOC_BuildSigfoxData();
			// TBD: send.
#ifdef IM_HWT
			// Compute next state.
			if (((geoloc_ctx.geoloc_status_byte & (0b1 << GEOLOC_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
				((geoloc_ctx.geoloc_status_byte & (0b1 << GEOLOC_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
				(HWT_WasResetReason())) {
				// Compute effective duration and calibrate hardware timer thanks to GEOLOC timestamp.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_CALIBRATE_HARDWARE_TIMER;
			}
			else {
				// No calibration required, GEOLOC processing is done.
				geoloc_ctx.geoloc_state = GEOLOC_STATE_END;
			}
#else
			geoloc_ctx.geoloc_state = GEOLOC_STATE_END;
#endif
			break;

#ifdef IM_HWT
		/* Hardware timer calibration */
		case GEOLOC_STATE_CALIBRATE_HARDWARE_TIMER:
			HWT_Calibrate(GEOLOC_GetDeltaTimestampSeconds());
			geoloc_ctx.geoloc_state = GEOLOC_STATE_END;
			break;
#endif

		/* End of processing */
		case GEOLOC_STATE_END:
			break;

		/* Unknown state */
		default:
			break;
		}
	}
}

/* GET GEOLOC STATUS BYTE.
 * @param geoloc_status_byte:	Pointer to char that will contain GEOLOC status byte.
 * @return:					None.
 */
void GEOLOC_GetStatusByte(unsigned char* geoloc_status_byte) {
	(*geoloc_status_byte) = geoloc_ctx.geoloc_status_byte;
}
