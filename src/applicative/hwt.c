/*
 * hwt.c
 *
 *  Created on: 11 aug. 2018
 *      Author: Ludo
 */

#include "hwt.h"

#include "gpio.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "max11136.h"
#include "max5495.h"
#include "mode.h"
#include "neom8n.h"
#include "nvm.h"
#include "rcc_reg.h"
#include "tim.h"

#ifdef IM_HWT

/*** Hardware timer local macros ***/

// Assuming average number of days per year is 365.25.
#define AVERAGE_NUMBER_OF_SECONDS_PER_YEAR		31557600
#define AVERAGE_NUMBER_OF_SECONDS_PER_MONTH		2629800
#define NUMBER_OF_SECONDS_PER_DAY				86400
#define NUMBER_OF_SECONDS_PER_HOUR				3600
#define NUMBER_OF_SECONDS_PER_MINUTE			60
// Feedback parameters.
#define HWT_TARGETTED_DURATION_SECONDS			3600	// HWT targetted duration in seconds.
#define HWT_ZERO_FEEDBACK_ERROR_SECONDS			60 		// No feedback applied if duration error is under this value.
#define HWT_MAXIMUM_FEEDBACK_ERROR_SECONDS		900		// Maximum feedback value is applied as soon as duration error exceeds this value.
#define HWT_MAXIMUM_FEEDBACK_STEPS				10		// Maximum feedback value expressed in digital potentiometer steps.
// Length of geolocation Sigfox frame in bytes.
#define HWT_SIGFOX_DATA_LENGTH					9

/*** Hardware Timer local structures ***/

// Status bit indexes.
typedef enum {
	HWT_STATUS_BYTE_PREVIOUS_TIMESTAMP_VALID_BIT_INDEX,
	HWT_STATUS_BYTE_ZDA_PARSING_SUCCESS_BIT_INDEX,
	HWT_STATUS_BYTE_ZDA_DATA_VALID_BIT_INDEX,
	HWT_STATUS_BYTE_ZDA_TIMEOUT_BIT_INDEX,
	HWT_STATUS_BYTE_FEEDBACK_DIRECTION_BIT_INDEX,
	HWT_STATUS_BYTE_CALIBRATION_DONE_BIT_INDEX
} HWT_StatusBitsIndexes;

// State machine.
typedef enum {
	HWT_STATE_GET_TIMESTAMP,
	HWT_STATE_OFF,
	HWT_STATE_CALIBRATION,
	HWT_STATE_SIGFOX,
	HWT_STATE_UPDATE_NVM,
	HWT_STATE_END
} HWT_State;

typedef struct {
	// State.
	HWT_State hwt_state;
	// Previous and current timestamp.
	Timestamp hwt_previous_timestamp;
	Timestamp hwt_current_timestamp;
	// Fix duration and timeout.
	unsigned char hwt_timeout_seconds; // Timestamp fix timeout in seconds (retrieved from NVM).
	unsigned char hwt_fix_start_time_seconds; // Absolute time (since MCU start-up) of fix start in seconds.
	unsigned char hwt_fix_duration_seconds; // Fix duration in seconds.
	// Calibration data.
	unsigned int hwt_effective_duration_seconds;
	unsigned int hwt_absolute_error_seconds;
	unsigned char hwt_feedback_direction; // 0 = duration error is negative (> 3600s) , 1 = duration error is positive (< 3600s).
	unsigned char hwt_feedback_value_steps;
	// Sigfox data.
	unsigned char hwt_sigfox_data[HWT_SIGFOX_DATA_LENGTH];
	// Status byte.
	unsigned char hwt_status_byte;
} HWT_Context;

/*** Hardware Timer local global variables ***/

static HWT_Context hwt_ctx;

/*** Hardware Timer local functions ***/

/* COMPARE PREVIOUS AND CURRENT GEOLOC DATES.
 * @param:	None.
 * @return:	None.
 */
unsigned char HWT_DateChanged(void) {
	unsigned char result = 0;
	// Compare all fields.
	if ((hwt_ctx.hwt_previous_timestamp.date != hwt_ctx.hwt_current_timestamp.date) ||
		(hwt_ctx.hwt_previous_timestamp.month != hwt_ctx.hwt_current_timestamp.month) ||
		(hwt_ctx.hwt_previous_timestamp.year != hwt_ctx.hwt_current_timestamp.year)) {
		result = 1;
	}
	// Note: the result is positive when previous data is invalid (0 value in all fields) and current one is valid.
	// This way, a fix is launched when previous timestamp failed, whatever the day.
	return result;
}

/* COMPUTE EFFECTIVE DURATION OF HARDWARE TIMER.
 * @param:	None.
 * @return:	None.
 */
void HWT_ComputeEffectiveDuration(void) {
	// Convert previous timestamp into a number of seconds since January 1st of the previous year.
	unsigned int previous_timestamp_seconds = (hwt_ctx.hwt_previous_timestamp.month-1)*AVERAGE_NUMBER_OF_SECONDS_PER_MONTH;
	previous_timestamp_seconds += (hwt_ctx.hwt_previous_timestamp.date-1)*NUMBER_OF_SECONDS_PER_DAY;
	previous_timestamp_seconds += (hwt_ctx.hwt_previous_timestamp.hours)*NUMBER_OF_SECONDS_PER_HOUR;
	previous_timestamp_seconds += (hwt_ctx.hwt_previous_timestamp.minutes)*NUMBER_OF_SECONDS_PER_MINUTE;
	previous_timestamp_seconds += hwt_ctx.hwt_previous_timestamp.seconds;
	// Compute number of year(s) between previous and current timestamp.
	unsigned int delta_year = 0;
	if ((hwt_ctx.hwt_current_timestamp.year == 0) && (hwt_ctx.hwt_previous_timestamp.year == 99)) {
		delta_year = 1;
	}
	else {
		unsigned short previous_year = hwt_ctx.hwt_previous_timestamp.year;
		unsigned short current_year = hwt_ctx.hwt_current_timestamp.year;
		delta_year = current_year-previous_year;
	}
	// Convert current timestamp into a number of seconds since January 1st of the previous year.
	unsigned int current_timestamp_seconds = delta_year*AVERAGE_NUMBER_OF_SECONDS_PER_YEAR;
	current_timestamp_seconds += (hwt_ctx.hwt_current_timestamp.month-1)*AVERAGE_NUMBER_OF_SECONDS_PER_MONTH;
	current_timestamp_seconds += (hwt_ctx.hwt_current_timestamp.date-1)*NUMBER_OF_SECONDS_PER_DAY;
	current_timestamp_seconds += (hwt_ctx.hwt_current_timestamp.hours)*NUMBER_OF_SECONDS_PER_HOUR;
	current_timestamp_seconds += (hwt_ctx.hwt_current_timestamp.minutes)*NUMBER_OF_SECONDS_PER_MINUTE;
	current_timestamp_seconds += hwt_ctx.hwt_current_timestamp.seconds;
	// Compensate GPS fix durations.
	unsigned int current_mcu_start_time_seconds = current_timestamp_seconds-hwt_ctx.hwt_current_timestamp.mcu_time_seconds;
	unsigned int previous_mcu_start_time_seconds = previous_timestamp_seconds-hwt_ctx.hwt_previous_timestamp.mcu_time_seconds;
	// Compute delta in seconds.
	hwt_ctx.hwt_effective_duration_seconds = current_mcu_start_time_seconds - previous_mcu_start_time_seconds;
}

/* ADJUST DIGITAL POTENTIOMETER TO CALIBRATE HARDWARE TIMER (1 HOUR TARGET).
 * @param:	None.
 * @return:	None.
 */
void HWT_Calibrate(void) {
	// Compute error.
	signed int hwt_duration_error_seconds = HWT_TARGETTED_DURATION_SECONDS - hwt_ctx.hwt_effective_duration_seconds;
	// Compute feedback direction.
	if (hwt_duration_error_seconds < 0) {
		hwt_ctx.hwt_feedback_direction = 0;
		hwt_ctx.hwt_absolute_error_seconds = (-1)*hwt_duration_error_seconds;
	}
	else {
		hwt_ctx.hwt_feedback_direction = 1;
		hwt_ctx.hwt_absolute_error_seconds = hwt_duration_error_seconds;
	}
	// Implement numeric proportional control.
	if (hwt_ctx.hwt_absolute_error_seconds > HWT_ZERO_FEEDBACK_ERROR_SECONDS) {
		// Feedback is required, compute number of steps.
		if (hwt_ctx.hwt_absolute_error_seconds > HWT_MAXIMUM_FEEDBACK_ERROR_SECONDS) {
			// Feedback saturation.
			hwt_ctx.hwt_feedback_value_steps = HWT_MAXIMUM_FEEDBACK_STEPS;
		}
		else {
			// Linear feedback.
			hwt_ctx.hwt_feedback_value_steps = 1 + (((HWT_MAXIMUM_FEEDBACK_STEPS-1)*(hwt_ctx.hwt_absolute_error_seconds-HWT_ZERO_FEEDBACK_ERROR_SECONDS)) / (HWT_MAXIMUM_FEEDBACK_ERROR_SECONDS));
		}
		// Apply feedback.
		unsigned char step_idx = 0;
		for (step_idx=0 ; step_idx<hwt_ctx.hwt_feedback_value_steps ; step_idx++) {
			switch (hwt_ctx.hwt_feedback_direction) {
			case 0:
				// Voltage reference is too low -> increase potentiometer value.
				//MAX5495_Increment();
				break;
			case 1:
				// Voltage reference is too high -> decrease potentiometer value.
				//MAX5495_Decrement();
				break;
			default:
				// Incorrect value.
				break;
			}
		}
	}
}

/* INIT HARDWARE TIMER.
 * @param:	None.
 * @return:	None.
 */
void HWT_Init(unsigned char was_wake_up_reason, unsigned char timestamp_retrieved) {

	/* Init context */
	// Current timestamp.
	hwt_ctx.hwt_current_timestamp.date = 0;
	hwt_ctx.hwt_current_timestamp.month = 0;
	hwt_ctx.hwt_current_timestamp.year = 0;
	hwt_ctx.hwt_current_timestamp.hours = 0;
	hwt_ctx.hwt_current_timestamp.minutes = 0;
	hwt_ctx.hwt_current_timestamp.seconds = 0;
	hwt_ctx.hwt_current_timestamp.mcu_time_seconds = 0;
	// Previous timestamp.
	NVM_ReadByte(NVM_HWT_PGT_DATE_ADDRESS_OFFSET, &hwt_ctx.hwt_previous_timestamp.date);
	NVM_ReadByte(NVM_HWT_PGT_MONTH_ADDRESS_OFFSET, &hwt_ctx.hwt_previous_timestamp.month);
	unsigned char year_msb;
	NVM_ReadByte(NVM_HWT_PGT_YEAR_ADDRESS_OFFSET, &year_msb);
	unsigned char year_lsb;
	NVM_ReadByte(NVM_HWT_PGT_YEAR_ADDRESS_OFFSET+1, &year_lsb);
	hwt_ctx.hwt_previous_timestamp.year = (year_msb << 8) + year_lsb;
	NVM_ReadByte(NVM_HWT_PGT_HOURS_ADDRESS_OFFSET, &hwt_ctx.hwt_previous_timestamp.hours);
	NVM_ReadByte(NVM_HWT_PGT_MINUTES_ADDRESS_OFFSET, &hwt_ctx.hwt_previous_timestamp.minutes);
	NVM_ReadByte(NVM_HWT_PGT_SECONDS_ADDRESS_OFFSET, &hwt_ctx.hwt_previous_timestamp.seconds);
	NVM_ReadByte(NVM_HWT_PGT_MCU_TIME_ADDRESS_OFFSET, &hwt_ctx.hwt_previous_timestamp.mcu_time_seconds);
	// Fix duration and timeout.
	NVM_ReadByte(NVM_CONFIG_GPS_TIMEOUT_ADDRESS_OFFSET, &hwt_ctx.hwt_timeout_seconds);
	hwt_ctx.hwt_fix_start_time_seconds = 0;
	hwt_ctx.hwt_fix_duration_seconds = 0;
	// Calibration data.
	hwt_ctx.hwt_absolute_error_seconds = 0;
	hwt_ctx.hwt_feedback_direction = 0;
	hwt_ctx.hwt_feedback_value_steps = 0;
	// Sigfox.
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<HWT_SIGFOX_DATA_LENGTH ; byte_idx++) hwt_ctx.hwt_sigfox_data[byte_idx] = 0;
	// Status byte.
	hwt_ctx.hwt_status_byte = 0;
	if (NEOM8N_TimestampIsValid(&hwt_ctx.hwt_previous_timestamp) == 1) {
		hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_PREVIOUS_TIMESTAMP_VALID_BIT_INDEX);
	}
	// State.
	if (timestamp_retrieved == 0) {
		hwt_ctx.hwt_state = HWT_STATE_GET_TIMESTAMP;
	}
	else {
		// Update status byte.
		hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_ZDA_PARSING_SUCCESS_BIT_INDEX);
		hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_ZDA_DATA_VALID_BIT_INDEX);
		// Check if calibration can be performed.
		if (((hwt_ctx.hwt_status_byte & HWT_STATUS_BYTE_PREVIOUS_TIMESTAMP_VALID_BIT_INDEX) != 0) && (was_wake_up_reason != 0)) {
			// Hardware timer was wake-up reason, previous and current timestamp are both valid -> perform calibration.
			hwt_ctx.hwt_state = HWT_STATE_CALIBRATION;
		}
		else {
			// Timestamp retrieved but no calibration is not possible -> directly send Sigfox HWT frame.
			hwt_ctx.hwt_state = HWT_STATE_SIGFOX;
		}
	}

	/* Init digital potentiometer */
	MAX5495_Init();
}

/* BUILD SIGFOX DATA STARTING FROM HWT STATUS.
 * @param:	None.
 * @return:	None.
 */
void HWT_BuildSigfoxData(void) {
	// Absolute error.
	hwt_ctx.hwt_sigfox_data[0] = (hwt_ctx.hwt_absolute_error_seconds & 0xFF000000) >> 24;
	hwt_ctx.hwt_sigfox_data[1] = (hwt_ctx.hwt_absolute_error_seconds & 0x00FF0000) >> 16;
	hwt_ctx.hwt_sigfox_data[2] = (hwt_ctx.hwt_absolute_error_seconds & 0x0000FF00) >> 8;
	hwt_ctx.hwt_sigfox_data[3] = hwt_ctx.hwt_absolute_error_seconds & 0x000000FF;
	// Feedback value.
	hwt_ctx.hwt_sigfox_data[4] = hwt_ctx.hwt_feedback_value_steps;
	// Reference voltage.
	unsigned int hwt_reference_voltage_mv = 0;
	// TBD with MAX11136 AIN3.
	hwt_ctx.hwt_sigfox_data[5] = (hwt_reference_voltage_mv & 0x0000FF00) >> 8;
	hwt_ctx.hwt_sigfox_data[6] = hwt_reference_voltage_mv & 0x000000FF;
	// Fix duration.
	hwt_ctx.hwt_sigfox_data[7] = hwt_ctx.hwt_fix_duration_seconds;
	// Status byte.
	hwt_ctx.hwt_sigfox_data[8] = hwt_ctx.hwt_status_byte;
}

/*** Hardware timer functions ***/

/* CHECK IF HARDWARE TIMER EXPIRED.
 * @param:	None.
 * @return:	'1' if hardware timer expired, '0' otherwise.
 */
unsigned char HWT_Expired(void) {
	// Check HWT output.
	unsigned char hwt_state = 0;
#ifdef IM_HWT
	hwt_state = GPIO_Read(GPIO_HWT_OUT);
#endif
	return hwt_state;
}

/* RESET HARDWARE TIMER.
 * @param:	None.
 * @return:	None.
 */
void HWT_Reset(void) {
	// Close relay to reset hardware timer.
#ifdef IM_HWT
	GPIO_Write(GPIO_HWT_RESET, 1);
	LPTIM1_DelayMilliseconds(2000);
	GPIO_Write(GPIO_HWT_RESET, 0);
#endif
}

/* MAIN ROUTINE OF HARDWARE TIMER.
 * @param was_wake_up_reason:	'1' if hardware timer woke-up the MCU from standby mode, '0' otherwise.
 * @param gps_timestamp:		Pointer to GPS timestamp potentially retrieved by a previous hwtation.
 * @param timestamp_retrieved:	'1' if GPS timestamp was previously retrived (parameter is directly usable), '0' otherwise.
 * @return:						None.
 */
void HWT_Process(unsigned char was_wake_up_reason, unsigned char timestamp_retrieved, Timestamp* gps_timestamp) {

	HWT_Init(was_wake_up_reason, timestamp_retrieved);
	unsigned char neom8n_result = 0;

	/* Loop until HWT calibration is done */
	while (hwt_ctx.hwt_state != HWT_STATE_END) {

		/* Perform state machine */
		switch (hwt_ctx.hwt_state) {

		/* Retrieve GPS timestamp */
		case HWT_STATE_GET_TIMESTAMP:
			// Start NEOM8N message parsing.
			neom8n_result = NEOM8N_GetTimestamp(&hwt_ctx.hwt_current_timestamp, hwt_ctx.hwt_timeout_seconds);
			switch (neom8n_result) {
			case NEOM8N_SUCCESS:
				// Update status.
				hwt_ctx.hwt_fix_duration_seconds = TIM22_GetSeconds()-hwt_ctx.hwt_fix_start_time_seconds;
				hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_ZDA_PARSING_SUCCESS_BIT_INDEX);
				hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_ZDA_DATA_VALID_BIT_INDEX);
				break;
			case NEOM8N_INVALID_DATA:
				// Update status.
				hwt_ctx.hwt_fix_duration_seconds = hwt_ctx.hwt_timeout_seconds;
				hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_ZDA_TIMEOUT_BIT_INDEX);
				hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_ZDA_PARSING_SUCCESS_BIT_INDEX);
				break;
			case NEOM8N_TIMEOUT:
				// Update status.
				hwt_ctx.hwt_fix_duration_seconds = hwt_ctx.hwt_timeout_seconds;
				hwt_ctx.hwt_status_byte |= (0b1 << HWT_STATUS_BYTE_ZDA_TIMEOUT_BIT_INDEX);
				break;
			default:
				// Unexpected result.
				break;
			}
			// Compute next state.
			hwt_ctx.hwt_state = HWT_STATE_OFF;
			break;

		/* Switch GPS module off */
		case HWT_STATE_OFF:
			// Stop LPUART, DMA and GPS module.
			LPUART1_PowerOff();
			// Compute next state.
			if (((hwt_ctx.hwt_status_byte & HWT_STATUS_BYTE_PREVIOUS_TIMESTAMP_VALID_BIT_INDEX) != 0) && (was_wake_up_reason != 0)) {
				// Hardware timer was wake-up reason, previous and current timestamp are both valid -> perform calibration.
				hwt_ctx.hwt_state = HWT_STATE_CALIBRATION;
			}
			else {
				// Timestamp retrieved but no calibration is not possible -> directly send Sigfox HWT frame.
				hwt_ctx.hwt_state = HWT_STATE_SIGFOX;
			}
			break;

		/* Hardware timer calibration */
		case HWT_STATE_CALIBRATION:
			// Perform calibration.
			HWT_ComputeEffectiveDuration();
			HWT_Calibrate();
			// Send Sigfox frame to report position.
			hwt_ctx.hwt_state = HWT_STATE_SIGFOX;
			break;

		/* Send Sigfox HWT frame */
		case HWT_STATE_SIGFOX:
			// Build frame.
			HWT_BuildSigfoxData();
			// TBD: send.
			hwt_ctx.hwt_state = HWT_STATE_UPDATE_NVM;
			break;

		/* NVM update */
		case HWT_STATE_UPDATE_NVM:
			// Store current timestamp in NVM whatever the result.
			NVM_WriteByte(NVM_HWT_PGT_DATE_ADDRESS_OFFSET, hwt_ctx.hwt_current_timestamp.date);
			NVM_WriteByte(NVM_HWT_PGT_MONTH_ADDRESS_OFFSET, hwt_ctx.hwt_current_timestamp.month);
			NVM_WriteByte(NVM_HWT_PGT_YEAR_ADDRESS_OFFSET, ((hwt_ctx.hwt_current_timestamp.year & 0x0000FF00) >> 8));
			NVM_WriteByte(NVM_HWT_PGT_YEAR_ADDRESS_OFFSET+1, (hwt_ctx.hwt_current_timestamp.year & 0x000000FF));
			NVM_WriteByte(NVM_HWT_PGT_HOURS_ADDRESS_OFFSET, hwt_ctx.hwt_current_timestamp.hours);
			NVM_WriteByte(NVM_HWT_PGT_MINUTES_ADDRESS_OFFSET, hwt_ctx.hwt_current_timestamp.minutes);
			NVM_WriteByte(NVM_HWT_PGT_SECONDS_ADDRESS_OFFSET, hwt_ctx.hwt_current_timestamp.seconds);
			NVM_WriteByte(NVM_HWT_PGT_MCU_TIME_ADDRESS_OFFSET, hwt_ctx.hwt_current_timestamp.mcu_time_seconds);
			// Compute next state.
			hwt_ctx.hwt_state = HWT_STATE_END;
			break;

		/* End of processing */
		case HWT_STATE_END:
			break;

		/* Unknown state */
		default:
			// Unknwon state.
			LPUART1_PowerOff();
			hwt_ctx.hwt_state = HWT_STATE_END;
			break;
		}
	}
}

#endif
