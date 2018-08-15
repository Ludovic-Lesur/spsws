/*
 * gps.c
 *
 *  Created on: 29 apr. 2018
 *      Author: Ludovic
 */

#include "gps.h"

#include "hwt.h"
#include "iwdg.h"
#include "neom8n.h"
#include "nvm.h"
#include "pwr.h"
#include "tim.h"

/*** GPS local macros ***/

#define GPS_TIMEOUT_SECONDS_MIN_VALUE			60 // GPS fix minimum time-out in seconds.
#define GPS_TIMEOUT_SECONDS_DEFAULT_VALUE		60 // Default GPS fix time-out in seconds (in case if NVM read failure).
#define GPS_TIMEOUT_SECONDS_MAX_VALUE			180 // GPS fix maximum time-out in seconds.

#define GPS_SIGFOX_DATA_LENGTH					11 // Length of GPS Sigfox frame in bytes.

// Assuming average number of days per year is 365.25.
#define AVERAGE_NUMBER_OF_SECONDS_PER_YEAR		31557600
#define AVERAGE_NUMBER_OF_SECONDS_PER_MONTH		2629800
#define NUMBER_OF_SECONDS_PER_DAY				86400
#define NUMBER_OF_SECONDS_PER_HOUR				3600
#define NUMBER_OF_SECONDS_PER_MINUTE			60

/*** GPS local structures ***/

// Status bits offsets.
typedef enum {
	GPS_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET,
	GPS_CURRENT_TIMESTAMP_PARSING_SUCCESS_BIT_OFFSET,
	GPS_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET,
	GPS_POSITION_PARSING_SUCCESS_BIT_OFFSET,
	GPS_POSITION_DATA_VALID_BIT_OFFSET,
	GPS_TIMEOUT_FLAG_BIT_OFFSET,
} GPS_StatusBitsOffsets;

// ____________________________________________________________________________
// |                                                                           |
// |                             GPS status bits                               |
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
	GPS_STATE_INIT,
	GPS_STATE_GET_TIMESTAMP,
	GPS_STATE_GET_POSITION,
	GPS_STATE_OFF,
	GPS_STATE_SIGFOX,
	GPS_STATE_CALIBRATE_HARDWARE_TIMER,
	GPS_STATE_END
} GPS_State;

typedef struct {
	// State.
	GPS_State gps_state;
	// Fix duration and timeout.
	unsigned char gps_timeout_seconds;					// GPS fix timeout in seconds (retrieved from NVM and configurable via downlink).
	unsigned int gps_fix_start_time_seconds;			// Start time in seconds.
	unsigned int gps_fix_duration_seconds;				// Duration in seconds.
	// GPS period.
	unsigned char gps_day_count;						// Current number of days since previous GPS position fix.
	unsigned char gps_period_days;						// GPS position fix period in days.
	// Sigfox.
	unsigned char gps_sigfox_required;
	unsigned char gps_sigfox_data[GPS_SIGFOX_DATA_LENGTH];
	// Status byte.
	unsigned char gps_status_byte;						// Set to '1' as soon as a timeout condition is met.

} GPS_Context;

/*** GPS local global variables ***/

static GPS_Context gps_ctx;
static GPS_TimestampData gps_previous_timestamp;
static GPS_TimestampData gps_current_timestamp;
static GPS_PositionData gps_position;

/*** GPS local functions ***/

/* COMPARE PREVIOUS AND CURRENT GPS DATES.
 * @param:	None.
 * @return:	None.
 */
unsigned char GPS_DateChanged(void) {
	unsigned char result = 0;
	// Compare all fields.
	if ((gps_previous_timestamp.date_day != gps_current_timestamp.date_day) ||
		(gps_previous_timestamp.date_month != gps_current_timestamp.date_month) ||
		(gps_previous_timestamp.date_year != gps_current_timestamp.date_year)) {
		result = 1;
	}
	// Note: the result is positive when previous data is invalid (0 value in all fields) and current one is valid.
	// This way, a GPS position is computed when previous GPS timestamp failed, whatever the day.
	return result;
}

/* COMPUTE EFFECTIVE DURATION OF HARDWARE TIMER.
 * @param:							None.
 * @return delta_timestamp_seconds:	Number of seconds between previous ans current GPS timestamp (assuming there are valid).
 */
unsigned int GPS_GetDeltaTimestampSeconds(void) {
	unsigned int delta_timestamp_seconds = 0;
	// Refering to January 1st of the previous year.
	unsigned int previous_timestamp_seconds = (gps_previous_timestamp.date_month-1)*AVERAGE_NUMBER_OF_SECONDS_PER_MONTH;
	previous_timestamp_seconds += (gps_previous_timestamp.date_day-1)*NUMBER_OF_SECONDS_PER_DAY;
	previous_timestamp_seconds += (gps_previous_timestamp.time_hours)*NUMBER_OF_SECONDS_PER_HOUR;
	previous_timestamp_seconds += (gps_previous_timestamp.time_minutes)*NUMBER_OF_SECONDS_PER_MINUTE;
	previous_timestamp_seconds += gps_previous_timestamp.time_seconds;
	// Compute number of year(s) between previous and current timestamp.
	unsigned int delta_year = 0;
	if ((gps_current_timestamp.date_year == 0) && (gps_previous_timestamp.date_year == 99)) {
		delta_year = 1;
	}
	else {
		unsigned short previous_year = gps_previous_timestamp.date_year;
		unsigned short current_year = gps_current_timestamp.date_year;
		delta_year = current_year-previous_year;
	}
	// Refering to January 1st of the previous year.
	unsigned int current_timestamp_seconds = delta_year*AVERAGE_NUMBER_OF_SECONDS_PER_YEAR;
	current_timestamp_seconds += (gps_current_timestamp.date_month-1)*AVERAGE_NUMBER_OF_SECONDS_PER_MONTH;
	current_timestamp_seconds += (gps_current_timestamp.date_day-1)*NUMBER_OF_SECONDS_PER_DAY;
	current_timestamp_seconds += (gps_current_timestamp.time_hours)*NUMBER_OF_SECONDS_PER_HOUR;
	current_timestamp_seconds += (gps_current_timestamp.time_minutes)*NUMBER_OF_SECONDS_PER_MINUTE;
	current_timestamp_seconds += gps_current_timestamp.time_seconds;
	// Compute delta.
	delta_timestamp_seconds = current_timestamp_seconds-previous_timestamp_seconds;
	return delta_timestamp_seconds;
}

/* GET GPS PARAMETERS STORED IN NVM.
 * @param:	None.
 * @return:	None.
 */
void GPS_GetNvmParameters(void) {
	// Previous GPS timestamp.
	NVM_ReadByte(NVM_GPS_PREVIOUS_TIMESTAMP_DAY_ADDRESS_OFFSET, &gps_previous_timestamp.date_day);
	NVM_ReadByte(NVM_GPS_PREVIOUS_TIMESTAMP_MONTH_ADDRESS_OFFSET, &gps_previous_timestamp.date_month);
	unsigned char year_msb;
	NVM_ReadByte(NVM_GPS_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET, &year_msb);
	unsigned char year_lsb;
	NVM_ReadByte(NVM_GPS_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET+1, &year_lsb);
	gps_previous_timestamp.date_year = (year_msb << 8) + year_lsb;
	NVM_ReadByte(NVM_GPS_PREVIOUS_TIMESTAMP_HOURS_ADDRESS_OFFSET, &gps_previous_timestamp.time_hours);
	NVM_ReadByte(NVM_GPS_PREVIOUS_TIMESTAMP_MINUTES_ADDRESS_OFFSET, &gps_previous_timestamp.time_minutes);
	NVM_ReadByte(NVM_GPS_PREVIOUS_TIMESTAMP_SECONDS_ADDRESS_OFFSET, &gps_previous_timestamp.time_seconds);
	// GPS timeout.
	NVM_ReadByte(NVM_GPS_TIMEOUT_SECONDS_ADDRESS_OFFSET, &gps_ctx.gps_timeout_seconds);
	if ((gps_ctx.gps_timeout_seconds < GPS_TIMEOUT_SECONDS_MIN_VALUE) || (gps_ctx.gps_timeout_seconds > GPS_TIMEOUT_SECONDS_MAX_VALUE)) {
		gps_ctx.gps_timeout_seconds = GPS_TIMEOUT_SECONDS_DEFAULT_VALUE;
	}
	// Day count.
	NVM_ReadByte(NVM_GPS_DAY_COUNT_ADDRESS_OFFSET, &gps_ctx.gps_day_count);
	if (gps_ctx.gps_day_count > GPS_PERIOD_DAYS_MAX_VALUE) {
		gps_ctx.gps_day_count = 0;
	}
	// GPS position fix period.
	NVM_ReadByte(NVM_GPS_PERIOD_DAYS_ADDRESS_OFFSET, &gps_ctx.gps_period_days);
	if ((gps_ctx.gps_period_days < GPS_PERIOD_DAYS_MIN_VALUE) || (gps_ctx.gps_period_days > GPS_PERIOD_DAYS_MAX_VALUE)) {
		gps_ctx.gps_period_days = GPS_PERIOD_DAYS_DEFAULT_VALUE;
	}
}

/* INIT GPS APPLICATION.
 * @param:	None.
 * @return:	NOne.
 */
void GPS_Init(void) {

	/* Init context */
	// State.
	gps_ctx.gps_state = GPS_STATE_INIT;
	// Fix duration and timeout.
	gps_ctx.gps_fix_start_time_seconds = 0;
	gps_ctx.gps_fix_duration_seconds = 0;
	// Sigfox.
	gps_ctx.gps_sigfox_required = 0;
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<GPS_SIGFOX_DATA_LENGTH ; byte_idx++) gps_ctx.gps_sigfox_data[byte_idx] = 0;
	// Status byte.
	gps_ctx.gps_status_byte = 0;
	if (NEOM8N_TimestampIsValid(gps_previous_timestamp) == 1) {
		gps_ctx.gps_status_byte |= (0b1 << GPS_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET);
	}

	/* Init current GPS timestamp */
	gps_current_timestamp.date_day = 0;
	gps_current_timestamp.date_month = 0;
	gps_current_timestamp.date_year = 0;
	gps_current_timestamp.time_hours = 0;
	gps_current_timestamp.time_minutes = 0;
	gps_current_timestamp.time_seconds = 0;

	/* Init GPS position */
	gps_position.lat_degrees = 0;
	gps_position.lat_minutes = 0;
	gps_position.lat_seconds = 0;
	gps_position.lat_north = 0;
	gps_position.long_degrees = 0;
	gps_position.long_minutes = 0;
	gps_position.long_seconds = 0;
	gps_position.long_east = 0;
	gps_position.altitude = 0;
}

/* BUILD SIGFOX DATA STARTING FROM GPS POSITION AND STATUS.
 * @param:	None.
 * @return:	None.
 */
void GPS_BuildSigfoxData(void) {

	/* Build frame */
	// Latitude degrees.
	gps_ctx.gps_sigfox_data[0] = gps_position.lat_degrees;
	// Latitude minutes.
	gps_position.lat_minutes = (gps_position.lat_minutes & 0x3F); // Ensure minutes are on 6 bits.
	gps_ctx.gps_sigfox_data[1] |= (gps_position.lat_minutes << 2);
	// Latitude seconds.
	gps_position.lat_seconds = (gps_position.lat_seconds & 0x0001FFFF); // Ensure seconds are on 17 bits.
	gps_ctx.gps_sigfox_data[1] |= ((gps_position.lat_seconds & 0x00018000) >> 15); // Keep bits 15-16.
	gps_ctx.gps_sigfox_data[2] |= ((gps_position.lat_seconds & 0x00007F80) >> 7); // Keep bits 7-14.
	gps_ctx.gps_sigfox_data[3] |= ((gps_position.lat_seconds & 0x0000007F) << 1); // Keep bits 0-6.
	// Latitude N/S.
	gps_position.lat_north = (gps_position.lat_north & 0x01); // Ensure north flag is a bit.
	gps_ctx.gps_sigfox_data[3] |= gps_position.lat_north;
	// Longitude degrees.
	gps_ctx.gps_sigfox_data[4] = gps_position.long_degrees;
	// Longitude minutes.
	gps_position.long_minutes = (gps_position.long_minutes & 0x3F); // Ensure minutes are on 6 bits.
	gps_ctx.gps_sigfox_data[5] |= (gps_position.long_minutes << 2);
	// Longitude seconds.
	gps_position.long_seconds = (gps_position.long_seconds & 0x0001FFFF); // Ensure seconds are on 17 bits.
	gps_ctx.gps_sigfox_data[5] |= ((gps_position.long_seconds & 0x00018000) >> 15); // Keep bits 15-16.
	gps_ctx.gps_sigfox_data[6] |= ((gps_position.long_seconds & 0x00007F80) >> 7); // Keep bits 7-14.
	gps_ctx.gps_sigfox_data[7] |= ((gps_position.long_seconds & 0x0000007F) << 1); // Keep bits 0-6.
	// Longitude E/O.
	gps_position.long_east = (gps_position.long_east & 0x01); // Ensure east flag is a bit.
	gps_ctx.gps_sigfox_data[7] |= gps_position.long_east;
	// Altitude in m.
	gps_position.altitude = (gps_position.altitude & 0x0000FFFF); // Ensure altitude is on 16 bits.
	gps_ctx.gps_sigfox_data[8] |= ((gps_position.altitude & 0x0000FF00) >> 8);
	gps_ctx.gps_sigfox_data[9] |= (gps_position.altitude & 0x000000FF);
	// GPS fix duration.
	gps_ctx.gps_sigfox_data[10] = gps_ctx.gps_fix_duration_seconds;
}

/*** GPS functions ***/

/* MAIN ROUTINE OF GPS MODULE.
 * @param:	None.
 * @return:	None.
 */
void GPS_Processing(void) {

	unsigned char neom8n_result = 0;

	/* Loop until GPS processing is done */
	while (gps_ctx.gps_state != GPS_STATE_END) {

		/* Reload watchdog counter */
		IWDG_Reload();

		/* Perform state machine */
		switch (gps_ctx.gps_state) {

		/* Init */
		case GPS_STATE_INIT:
			GPS_GetNvmParameters();
			GPS_Init();
			NEOM8N_Init();
			gps_ctx.gps_state = GPS_STATE_GET_TIMESTAMP;
			break;

		/* Wait current GPS timestamp */
		case GPS_STATE_GET_TIMESTAMP:
			// Start NEO-M8N message parsing.
			gps_ctx.gps_fix_start_time_seconds = TIM_TimeGetSeconds();
			neom8n_result = NEOM8N_GetTimestamp(&gps_current_timestamp, gps_ctx.gps_timeout_seconds);
			// Parse return code.
			switch (neom8n_result) {

			case NEOM8N_SUCCESS:
				// Update status byte.
				gps_ctx.gps_status_byte |= (0b1 << GPS_CURRENT_TIMESTAMP_PARSING_SUCCESS_BIT_OFFSET);
				gps_ctx.gps_status_byte |= (0b1 << GPS_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET);
				// Compute next state.
				if (GPS_DateChanged() == 1) {
					// Day changed or previous GPS timestamp invalid -> increment day count and set downlink request.
					//CONF_RequestDownlink();
					gps_ctx.gps_day_count++;
					if (gps_ctx.gps_day_count == gps_ctx.gps_period_days) {
						// GPS position fix period reached.
						gps_ctx.gps_day_count = 0;
						gps_ctx.gps_state = GPS_STATE_GET_POSITION;
					}
					else {
						gps_ctx.gps_fix_duration_seconds = TIM_TimeGetSeconds()-gps_ctx.gps_fix_start_time_seconds;
						gps_ctx.gps_state = GPS_STATE_OFF;
					}
				}
				else {
					gps_ctx.gps_fix_duration_seconds = TIM_TimeGetSeconds()-gps_ctx.gps_fix_start_time_seconds;
					gps_ctx.gps_state = GPS_STATE_OFF;
				}
				break;

			case NEOM8N_INVALID_DATA:
				// Update status.
				gps_ctx.gps_fix_duration_seconds = gps_ctx.gps_timeout_seconds;
				gps_ctx.gps_status_byte |= (0b1 << GPS_CURRENT_TIMESTAMP_PARSING_SUCCESS_BIT_OFFSET);
				gps_ctx.gps_status_byte |= (0b1 << GPS_TIMEOUT_FLAG_BIT_OFFSET);
				// Compute next state.
				gps_ctx.gps_state = GPS_STATE_OFF;
				break;

			case NEOM8N_TIMEOUT:
				// Update status.
				gps_ctx.gps_fix_duration_seconds = gps_ctx.gps_timeout_seconds;
				gps_ctx.gps_status_byte |= (0b1 << GPS_TIMEOUT_FLAG_BIT_OFFSET);
				// Compute next state.
				gps_ctx.gps_state = GPS_STATE_OFF;
				break;

			default:
				break;
			}
			break;

		/* Wait GPS position */
		case GPS_STATE_GET_POSITION:
			// Start NEO-M8N message parsing.
			neom8n_result = NEOM8N_GetPosition(&gps_position, gps_ctx.gps_timeout_seconds);
			switch (neom8n_result) {

			case NEOM8N_SUCCESS:
				// Update status.
				gps_ctx.gps_fix_duration_seconds = TIM_TimeGetSeconds()-gps_ctx.gps_fix_start_time_seconds;
				gps_ctx.gps_status_byte |= (0b1 << GPS_POSITION_PARSING_SUCCESS_BIT_OFFSET);
				gps_ctx.gps_status_byte |= (0b1 << GPS_POSITION_DATA_VALID_BIT_OFFSET);
				gps_ctx.gps_sigfox_required = 1;
				break;

			case NEOM8N_INVALID_DATA:
				// Update status.
				gps_ctx.gps_fix_duration_seconds = gps_ctx.gps_timeout_seconds;
				gps_ctx.gps_status_byte |= (0b1 << GPS_TIMEOUT_FLAG_BIT_OFFSET);
				gps_ctx.gps_status_byte |= (0b1 << GPS_POSITION_PARSING_SUCCESS_BIT_OFFSET);
				break;

			case NEOM8N_TIMEOUT:
				// Update status.
				gps_ctx.gps_fix_duration_seconds = gps_ctx.gps_timeout_seconds;
				gps_ctx.gps_status_byte |= (0b1 << GPS_TIMEOUT_FLAG_BIT_OFFSET);
				break;

			default:
				break;
			}
			// Compute next state.
			gps_ctx.gps_state = GPS_STATE_OFF;
			break;

		/* Switch LPUART and GPS module off */
		case GPS_STATE_OFF:
			// Switch GPS module off.
			NEOM8N_Off();
			// Store current GPS timestamp in NVM whatever the result.
			NVM_WriteByte(NVM_GPS_PREVIOUS_TIMESTAMP_DAY_ADDRESS_OFFSET, gps_current_timestamp.date_day);
			NVM_WriteByte(NVM_GPS_PREVIOUS_TIMESTAMP_MONTH_ADDRESS_OFFSET, gps_current_timestamp.date_month);
			NVM_WriteByte(NVM_GPS_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET, ((gps_current_timestamp.date_year & 0x0000FF00) >> 8));
			NVM_WriteByte(NVM_GPS_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET+1, (gps_current_timestamp.date_year & 0x000000FF));
			NVM_WriteByte(NVM_GPS_PREVIOUS_TIMESTAMP_HOURS_ADDRESS_OFFSET, gps_current_timestamp.time_hours);
			NVM_WriteByte(NVM_GPS_PREVIOUS_TIMESTAMP_MINUTES_ADDRESS_OFFSET, gps_current_timestamp.time_minutes);
			NVM_WriteByte(NVM_GPS_PREVIOUS_TIMESTAMP_SECONDS_ADDRESS_OFFSET, gps_current_timestamp.time_seconds);
			// Update status byte in NVM.
			NVM_WriteByte(NVM_GPS_STATUS_ADDRESS_OFFSET, gps_ctx.gps_status_byte);
			// Update day count in NVM.
			NVM_WriteByte(NVM_GPS_DAY_COUNT_ADDRESS_OFFSET, gps_ctx.gps_day_count);
			// Compute next state.
			if (gps_ctx.gps_sigfox_required == 1) {
				// Send Sigfox frame to report position.
				gps_ctx.gps_state = GPS_STATE_SIGFOX;
			}
			else {
				if (((gps_ctx.gps_status_byte & (0b1 << GPS_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
				    ((gps_ctx.gps_status_byte & (0b1 << GPS_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
					(HWT_WasResetReason())) {
					// Compute effective duration and calibrate hardware timer thanks to GPS timestamp.
					gps_ctx.gps_state = GPS_STATE_CALIBRATE_HARDWARE_TIMER;
				}
				else {
					// Neither Sigfox frame to send nore calibration required, GPS processing is done.
					gps_ctx.gps_state = GPS_STATE_END;
				}
			}
			break;

		/* Send Sigfox GPS frame */
		case GPS_STATE_SIGFOX:
			// Build frame.
			GPS_BuildSigfoxData();
			// TBD: send.
			// Compute next state.
			if (((gps_ctx.gps_status_byte & (0b1 << GPS_PREVIOUS_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
				((gps_ctx.gps_status_byte & (0b1 << GPS_CURRENT_TIMESTAMP_DATA_VALID_BIT_OFFSET)) != 0) &&
				(HWT_WasResetReason())) {
				// Compute effective duration and calibrate hardware timer thanks to GPS timestamp.
				gps_ctx.gps_state = GPS_STATE_CALIBRATE_HARDWARE_TIMER;
			}
			else {
				// No calibration required, GPS processing is done.
				gps_ctx.gps_state = GPS_STATE_END;
			}
			break;

		/* Hardware timer calibration */
		case GPS_STATE_CALIBRATE_HARDWARE_TIMER:
			HWT_Calibrate(GPS_GetDeltaTimestampSeconds());
			gps_ctx.gps_state = GPS_STATE_END;
			break;

		/* End of processing */
		case GPS_STATE_END:
			break;

		/* Unknown state */
		default:
			break;
		}
	}
}

#ifdef HARDWARE_TIMER
/* GET GPS STATUS BYTE.
 * @param gps_status_byte:	Pointer to char that will contain GPS status byte.
 * @return:					None.
 */
void GPS_GetStatusByte(unsigned char* gps_status_byte) {
	(*gps_status_byte) = gps_ctx.gps_status_byte;
}
#endif
