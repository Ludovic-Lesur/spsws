/*
 * nvm.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_NVM_H_
#define PERIPHERALS_NVM_H_

/*** NVM macros ***/

// Sigfox device parameters size.
#define SIGFOX_ID_SIZE										4 	// Device ID size in bytes (must equal ID_LENGTH macros of Sigfox library).
#define SIGFOX_KEY_SIZE										16 	// Device key size in bytes.
#define SIGFOX_PAC_SIZE										8 	// Device initial PAC size in bytes (must equal PAC_LENGTH macros of Sigfox library).

// Sigfox device parameters address offsets.
#define NVM_SIGFOX_ID_ADDRESS_OFFSET						0
#define NVM_SIGFOX_KEY_ADDRESS_OFFSET						4
#define NVM_SIGFOX_INITIAL_PAC_ADDRESS_OFFSET				20
#define NVM_SIGFOX_PN_ADDRESS_OFFSET						28
#define NVM_SIGFOX_SEQ_NUM_ADDRESS_OFFSET					30
#define NVM_SIGFOX_FH_ADDRESS_OFFSET						32
#define NVM_SIGFOX_RL_ADDRESS_OFFSET						34

// Station parameters address offsets.
#define NVM_GPS_PREVIOUS_TIMESTAMP_DAY_ADDRESS_OFFSET		35
#define NVM_GPS_PREVIOUS_TIMESTAMP_MONTH_ADDRESS_OFFSET		36
#define NVM_GPS_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET		37
#define NVM_GPS_PREVIOUS_TIMESTAMP_HOURS_ADDRESS_OFFSET		39
#define NVM_GPS_PREVIOUS_TIMESTAMP_MINUTES_ADDRESS_OFFSET	40
#define NVM_GPS_PREVIOUS_TIMESTAMP_SECONDS_ADDRESS_OFFSET	41
#define NVM_GPS_TIMEOUT_SECONDS_ADDRESS_OFFSET				42
#define NVM_GPS_DAY_COUNT_ADDRESS_OFFSET					43
#define NVM_GPS_PERIOD_DAYS_ADDRESS_OFFSET					44
#define NVM_GPS_STATUS_ADDRESS_OFFSET						45
#define NVM_DOWNLINK_STATUS_ADDRESS_OFFSET					46
#define NVM_SENSORS_HOUR_COUNT_ADDRESS_OFFSET				47
#define NVM_SENSORS_PERIOD_HOURS_ADDRESS_OFFSET				48
#define NVM_SENSORS_STATUS_ADDRESS_OFFSET					49

/*** NVM functions ***/

void NVM_Init(void);
void NVM_ReadByte(unsigned short address_offset, unsigned char* byte_to_read);
void NVM_WriteByte(unsigned short address_offset, unsigned char byte_to_store);

#endif
