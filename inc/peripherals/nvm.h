/*
 * nvm.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef NVM_H
#define NVM_H

#include "mapping.h"

/*** NVM macros ***/

// Sigfox device parameters.
#define NVM_SIGFOX_ID_ADDRESS_OFFSET				0
#define NVM_SIGFOX_KEY_ADDRESS_OFFSET				4
#define NVM_SIGFOX_PN_ADDRESS_OFFSET				20
#define NVM_SIGFOX_SEQ_ADDRESS_OFFSET				22
#define NVM_SIGFOX_FH_ADDRESS_OFFSET				24
#define NVM_SIGFOX_RL_ADDRESS_OFFSET				26

// Device configuration (mapped on downlink frame).
#define NVM_CONFIG_START_ADDRESS_OFFSET				27
#define NVM_CONFIG_LOCAL_UTC_OFFSET_ADDRESS_OFFSET	27
#define NVM_CONFIG_UPLINK_FRAMES_ADDRESS_OFFSET		28
#define NVM_CONFIG_GPS_TIMEOUT_ADDRESS_OFFSET		29

// Period management and status.
#define NVM_DAY_COUNT_ADDRESS_OFFSET				35
#define NVM_HOURS_COUNT_ADDRESS_OFFSET				36
#define NVM_MONITORING_STATUS_BYTE_ADDRESS_OFFSET	37

// RTC.
#define NVM_RTC_PWKUP_YEAR_ADDRESS_OFFSET			38
#define NVM_RTC_PWKUP_MONTH_ADDRESS_OFFSET			40
#define NVM_RTC_PWKUP_DATE_ADDRESS_OFFSET			41
#define NVM_RTC_PWKUP_HOURS_ADDRESS_OFFSET			42

#ifdef USE_HWT
// HWT.
#define NVM_HWT_PGT_YEAR_ADDRESS_OFFSET				43
#define NVM_HWT_PGT_MONTH_ADDRESS_OFFSET			45
#define NVM_HWT_PGT_DATE_ADDRESS_OFFSET				46
#define NVM_HWT_PGT_HOURS_ADDRESS_OFFSET			47
#define NVM_HWT_PGT_MINUTES_ADDRESS_OFFSET			48
#define NVM_HWT_PGT_SECONDS_ADDRESS_OFFSET			49
#define NVM_HWT_PGT_MCU_TIME_ADDRESS_OFFSET			50
#define NVM_HWT_MAX5495_VALUE_ADDRESS_OFFSET		52
#endif

/*** NVM functions ***/

void NVM_Enable(void);
void NVM_Disable(void);
void NVM_ReadByte(unsigned short address_offset, unsigned char* byte_to_read);
void NVM_WriteByte(unsigned short address_offset, unsigned char byte_to_store);
void NVM_ResetDefault(void);

#endif /* NVM_H */
