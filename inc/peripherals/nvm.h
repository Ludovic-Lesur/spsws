/*
 * nvm.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef NVM_H
#define NVM_H

/*** NVM macros ***/

typedef enum {
	NVM_SUCCESS = 0,
	NVM_ERROR_ADDRESS,
	NVM_ERROR_UNLOCK,
	NVM_ERROR_LOCK,
	NVM_ERROR_WRITE,
	NVM_ERROR_BASE_LAST = 0x0100
} NVM_status_t;

typedef enum {
	NVM_ADDRESS_SIGFOX_DEVICE_ID = 0,
	NVM_ADDRESS_SIGFOX_DEVICE_KEY = 4,
	NVM_ADDRESS_SIGFOX_PN = 20,
	NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER = 22,
	NVM_ADDRESS_SIGFOX_FH = 24,
	NVM_ADDRESS_SIGFOX_RL = 26,
	NVM_ADDRESS_DEVICE_CONFIGURATION = 27, // Mapped on downlink payload.
	NVM_ADDRESS_GEOLOC_DATA_DAY_COUNT = 35,
	NVM_ADDRESS_WEATHER_DATA_HOUR_COUNT = 36,
	NVM_ADDRESS_STATUS = 37,
	NVM_ADDRESS_PREVIOUS_WAKE_UP_YEAR = 38,
	NVM_ADDRESS_PREVIOUS_WAKE_UP_MONTH = 40,
	NVM_ADDRESS_PREVIOUS_WAKE_UP_DATE = 41,
	NVM_ADDRESS_PREVIOUS_WAKE_UP_HOUR = 42,
	NVM_ADDRESS_LAST
} NVM_address_t;

/*** NVM functions ***/

void NVM_enable(void);
void NVM_disable(void);
NVM_status_t NVM_read_byte(NVM_address_t address_offset, unsigned char* data);
NVM_status_t NVM_write_byte(NVM_address_t address_offset, unsigned char data);
NVM_status_t NVM_reset_default(void);

#define NVM_status_check(error_base) { if (nvm_status != NVM_SUCCESS) { status = error_base + nvm_status; goto errors; }}

#endif /* NVM_H */
