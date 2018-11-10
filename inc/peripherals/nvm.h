/*
 * nvm.h
 *
 *  Created on: 19 june 2018
 *      Author: Ludovic
 */

#ifndef PERIPHERALS_NVM_H_
#define PERIPHERALS_NVM_H_

// Sigfox and station parameters are stored in NVM according to the following mapping (index 0 = NVM_START_ADDRESS):
// Acronyms:	ID = identifier.
//				PAC = product authentification code.
//				PN =
//				SEQ = sequence number.
//				FH =
//				RL =
//				GEOLOC = global positioning system.
//				PGT = previous GEOLOC timestamp.
//				DLK = downlink.
// __________________________________________________________________________________________________________________________________________________________________________________________________________
// |                                                   |                                                                                                                                                     |
// |                 Sigfox parameters                 |                                                               Station parameters                                                                    |
// |___________________________________________________|_____________________________________________________________________________________________________________________________________________________|
// |0    3|4    19|20   27|28  29|30   31|32  33|  34  | 35  | 36    | 37       | 38       | 39    | 40      | 41      | 42       | 43      | 44    | 45     | 46	  | 47     | 48      | 49      | 50      |
// |      |       |       |      |       |      |      | PGT | PGT   | PGT      | PGT      | PGT   | PGT     | PGT     | PGT      | GEOLOC     | GEOLOC   | GEOLOC    | GEOLOC    | DLK    | SENSORS | SENSORS | SENSORS |
// |  ID  |  KEY  |  PAC  |  PN  |  SEQ  |  FH  |  RL  | DAY | MONTH | YEAR MSB | YEAR LSB | HOURS | MINUTES | SECONDS | MCU TIME | TIMEOUT | COUNT | PERIOD | STATUS | STATUS | COUNT   | PERIOD  | STATUS  |
// |______|_______|_______|______|_______|______|______|_____|_______|__________|__________|_______|_________|_________|__________|_________|_______|________|________|________|_________|_________|_________|

/*** NVM macros ***/

// Sigfox device parameters size.
#define SIGFOX_ID_SIZE											4 	// Device ID size in bytes (must equal ID_LENGTH macros of Sigfox library).
#define SIGFOX_KEY_SIZE											16 	// Device key size in bytes.
#define SIGFOX_PAC_SIZE											8 	// Device initial PAC size in bytes (must equal PAC_LENGTH macros of Sigfox library).

// Sigfox device parameters address offsets.
#define NVM_SIGFOX_ID_ADDRESS_OFFSET							0
#define NVM_SIGFOX_KEY_ADDRESS_OFFSET							4
#define NVM_SIGFOX_INITIAL_PAC_ADDRESS_OFFSET					20
#define NVM_SIGFOX_PN_ADDRESS_OFFSET							28
#define NVM_SIGFOX_SEQ_NUM_ADDRESS_OFFSET						30
#define NVM_SIGFOX_FH_ADDRESS_OFFSET							32
#define NVM_SIGFOX_RL_ADDRESS_OFFSET							34

// Station parameters address offsets.
#define NVM_HWT_PREVIOUS_TIMESTAMP_DAY_ADDRESS_OFFSET			35
#define NVM_HWT_PREVIOUS_TIMESTAMP_MONTH_ADDRESS_OFFSET			36
#define NVM_HWT_PREVIOUS_TIMESTAMP_YEAR_ADDRESS_OFFSET			37
#define NVM_HWT_PREVIOUS_TIMESTAMP_HOURS_ADDRESS_OFFSET			39
#define NVM_HWT_PREVIOUS_TIMESTAMP_MINUTES_ADDRESS_OFFSET		40
#define NVM_HWT_PREVIOUS_TIMESTAMP_SECONDS_ADDRESS_OFFSET		41
#define NVM_HWT_PREVIOUS_TIMESTAMP_MCU_TIME_ADDRESS_OFFSET		42
#define NVM_GPS_TIMEOUT_SECONDS_ADDRESS_OFFSET					43
#define NVM_GEOLOC_DAY_COUNT_ADDRESS_OFFSET						44
#define NVM_GEOLOC_PERIOD_DAYS_ADDRESS_OFFSET					45
#define NVM_GEOLOC_STATUS_ADDRESS_OFFSET						46
#define NVM_DOWNLINK_STATUS_ADDRESS_OFFSET						47
#define NVM_SENSORS_HOUR_COUNT_ADDRESS_OFFSET					48
#define NVM_SENSORS_PERIOD_HOURS_ADDRESS_OFFSET					49
#define NVM_SENSORS_STATUS_ADDRESS_OFFSET						50

/*** NVM functions ***/

void NVM_Init(void);
void NVM_ReadByte(unsigned short address_offset, unsigned char* byte_to_read);
void NVM_WriteByte(unsigned short address_offset, unsigned char byte_to_store);

#endif
