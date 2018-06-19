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
#define SIGFOX_ID_SIZE	4 	// Device ID size in bytes (must equal ID_LENGTH macros of Sigfox library).
#define SIGFOX_KEY_SIZE	16 	// Device key size in bytes.
#define SIGFOX_PAC_SIZE	8 	// Device initial PAC size in bytes (must equal PAC_LENGTH macros of Sigfox library).

/*** NVM functions ***/

void NVM_Init(void);

void NVM_GetSigfoxId(unsigned char sfx_id[SIGFOX_ID_SIZE]);
void NVM_GetSigfoxKey(unsigned char sfx_key[SIGFOX_KEY_SIZE]);
void NVM_GetSigfoxInitialPac(unsigned char sfx_initial_pac[SIGFOX_ID_SIZE]);
void NVM_GetSigfoxPn(unsigned short* sfx_pn);
void NVM_GetSigfoxSeqNum(unsigned short* sfx_seq_num);
void NVM_GetSigfoxFh(unsigned short* sfx_fh);
void NVM_GetSigfoxRl(unsigned char* sfx_rl);

void NVM_SetSigfoxPn(unsigned short new_sfx_pn);
void NVM_SetSigfoxSeqNum(unsigned short new_sfx_seq_num);
void NVM_SetSigfoxFh(unsigned short new_sfx_fh);
void NVM_SetSigfoxRl(unsigned char new_sfx_rl);

#endif /* PERIPHERALS_NVM_H_ */
