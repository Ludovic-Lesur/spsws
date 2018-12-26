/*
 * nvm.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludo
 */

#ifndef NVM_H
#define NVM_H

// Sigfox and station parameters are stored in NVM according to the following mapping (index 0 = NVM_START_ADDRESS):
// _____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
// |                                         |                                                                                                                                                                                                          |
// |             Sigfox parameters           |                                                                                           Station parameters                                                                                             |
// |_________________________________________|__________________________________________________________________________________________________________________________________________________________________________________________________________|
// |0    3|4    19|20  21|22   23|24  25| 26 | 27     | 28      | 29     | 30           | 31 | 32 | 33 | 34     | 35        | 36           | 37     |38  39| 40    | 41   | 42	  |43  44| 45    | 46   | 47    | 48     | 49      |50      51|52     53|
// |      |       |      |       |      |    | UTC    | GPS     | GEOLOC | WEATHER DATA |    |    |    | UPLINK | GEOLOC    | WEATHER DATA | SPSWS  | PWUT | PWUT  | PWUT | PWUT  | PGT  | PGT   | PGT  | PGT   | PGT    | PGT     | PGT      | MAX5495 |
// |  ID  |  KEY  |  PN  |  SEQ  |  FH  | RL | OFFSET | TIMEOUT | PERIOD | PERIOD       |    |    |    | MASK   | DAY COUNT | HOUR COUNT   | STATUS | YEAR | MONTH | DATE | HOURS | YEAR | MONTH | DATE | HOURS | MINUTE | SECONDS | MCU TIME | VALUE   |
// |______|_______|______|_______|______|____|________|_________|________|______________|____|____|____|________|___________|______________|________|______|_______|______|_______|______|_______|______|_______|________|_________|__________|_________|

/*** NVM macros ***/

// Sigfox device parameters address offsets.
#define NVM_SIGFOX_ID_ADDRESS_OFFSET				0
#define NVM_SIGFOX_KEY_ADDRESS_OFFSET				4
#define NVM_SIGFOX_PN_ADDRESS_OFFSET				20
#define NVM_SIGFOX_SEQ_NUM_ADDRESS_OFFSET			22
#define NVM_SIGFOX_FH_ADDRESS_OFFSET				24
#define NVM_SIGFOX_RL_ADDRESS_OFFSET				26

// Station parameters address offsets.
#define NVM_LOCAL_UTC_OFFSET_ADDRESS_OFFSET			27
#define NVM_GPS_TIMEOUT_ADDRESS_OFFSET				28
#define NVM_GEOLOC_PERIOD_ADDRESS_OFFSET			29
#define NVM_WEATHER_DATA_PERIOD_ADDRESS_OFFSET		30
#define NVM_UPLINK_MASK_ADDRESS_OFFSET				34
#define NVM_GEOLOC_DAY_COUNT_ADDRESS_OFFSET			35
#define NVM_WEATHER_DATA_HOUR_COUNT_ADDRESS_OFFSET	36
#define NVM_SPSWS_STATUS_ADDRESS_OFFSET				37
#define NVM_PWUT_YEAR_ADDRESS_OFFSET				38
#define NVM_PWUT_MONTH_ADDRESS_OFFSET				40
#define NVM_PWUT_DATE_ADDRESS_OFFSET				41
#define NVM_PWUT_HOURS_ADDRESS_OFFSET				42
#define NVM_PGT_YEAR_ADDRESS_OFFSET					43
#define NVM_PGT_MONTH_ADDRESS_OFFSET				45
#define NVM_PGT_DATE_ADDRESS_OFFSET					46
#define NVM_PGT_HOURS_ADDRESS_OFFSET				47
#define NVM_PGT_MINUTES_ADDRESS_OFFSET				48
#define NVM_PGT_SECONDS_ADDRESS_OFFSET				49
#define NVM_PGT_MCU_TIME_ADDRESS_OFFSET				50
#define NVM_MAX5495_VALUE_ADDRESS_OFFSET			52

/*** NVM functions ***/

void NVM_Enable(void);
void NVM_Disable(void);
void NVM_ReadByte(unsigned short address_offset, unsigned char* byte_to_read);
void NVM_WriteByte(unsigned short address_offset, unsigned char byte_to_store);

#endif /* NVM_H */
