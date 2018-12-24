/*
 * mcu_api.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludo
 */

#include "mcu_api.h"

#include "adc.h"
#include "aes.h"
#include "max11136.h"
#include "nvm.h"
#include "sigfox_types.h"
#include "tim.h"

/*** MCU API local global variables ***/

sfx_u8 malloc_buffer[200];

/*!******************************************************************
 * \fn sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
 * \brief Allocate memory for library usage (Memory usage = size (Bytes))
 * This function is only called once at the opening of the Sigfox Library ( SIGF
 *
 * IMPORTANT NOTE:
 * --------------
 * The address reported need to be aligned with architecture of the microprocessor used.
 * For a Microprocessor of:
 *   - 8 bits  => any address is allowed
 *   - 16 bits => only address multiple of 2 are allowed
 *   - 32 bits => only address multiple of 4 are allowed
 *
 * \param[in] sfx_u16 size                  size of buffer to allocate in bytes
 * \param[out] sfx_u8** returned_pointer    pointer to buffer (can be static)
 *
 * \retval SFX_ERR_NONE:              No error
 * \retval MCU_ERR_API_MALLOC         Malloc error
 *******************************************************************/
sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8** returned_pointer) {
	// Allocate local buffer in RAM and return its address.
	(*returned_pointer) = &malloc_buffer[0];
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_free(sfx_u8 *ptr)
 * \brief Free memory allocated to library
 *
 * \param[in] sfx_u8 *ptr                        pointer to buffer
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_FREE:                     Free error
 *******************************************************************/
sfx_u8 MCU_API_free(sfx_u8* ptr) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle, sfx_u16 *voltage_tx, sfx_s16 *temperature)
 * \brief Get voltage and temperature for Out of band frames
 * Value must respect the units bellow for <B>backend compatibility</B>
 *
 * \param[in] none
 * \param[out] sfx_u16 *voltage_idle             Device's voltage in Idle state (mV)
 * \param[out] sfx_u16 *voltage_tx               Device's voltage in Tx state (mV) - for the last transmission
 * \param[out] sfx_s16 *temperature              Device's temperature in 1/10 of degrees celcius
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_VOLT_TEMP:                Get voltage/temperature error
 *******************************************************************/
sfx_u8 MCU_API_get_voltage_temperature(sfx_u16* voltage_idle, sfx_u16* voltage_tx, sfx_s16* temperature) {
	// Get MCU supply voltage.
	unsigned int mcu_supply_voltage_mv;
	MAX11136_GetSupplyVoltage(&mcu_supply_voltage_mv);
	(*voltage_idle) = (sfx_u16) mcu_supply_voltage_mv;
	(*voltage_tx) = (sfx_u16) mcu_supply_voltage_mv;
	// Get MCU internal temperature.
	signed char mcu_temperature_degrees;
	ADC1_GetMcuTemperature(&mcu_temperature_degrees);
	(*temperature) = ((sfx_s16) mcu_temperature_degrees) * 10; // Unit = 1/10 of degrees.

	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_delay(sfx_delay_t delay_type)
 * \brief Inter stream delay, called between each RF_API_send
 * - SFX_DLY_INTER_FRAME_TX  : 0 to 2s in Uplink DC
 * - SFX_DLY_INTER_FRAME_TRX : 500 ms in Uplink/Downlink FH & Downlink DC
 * - SFX_DLY_OOB_ACK :         1.4s to 4s for Downlink OOB
 * - SFX_DLY_CS_SLEEP :        delay between several trials of Carrier Sense (for the first frame only)
 *
 * \param[in] sfx_delay_t delay_type             Type of delay to call
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_DLY:                      Delay error
 *******************************************************************/
sfx_u8 MCU_API_delay(sfx_delay_t delay_type) {
	switch (delay_type) {
	case SFX_DLY_INTER_FRAME_TX:
		// 0 to 2s in Uplink DC.
		TIM22_WaitMilliseconds(500);
		break;
	case SFX_DLY_INTER_FRAME_TRX:
		// 500 ms in Uplink/Downlink FH & Downlink DC.
		TIM22_WaitMilliseconds(500);
		break;
	case SFX_DLY_OOB_ACK:
		// 1.4s to 4s for Downlink OOB.
		TIM22_WaitMilliseconds(2000);
		break;
	case SFX_DLY_CS_SLEEP:
		// Delay between several trials of Carrier Sense (for the first frame only).
		TIM22_WaitMilliseconds(1000);
		break;
	default:
		break;
	}
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_aes_128_cbc_encrypt(sfx_u8 *encrypted_data, sfx_u8 *data_to_encrypt, sfx_u8 aes_block_len, sfx_u8 key[16], sfx_credentials_use_key_t use_key)
 * \brief Encrypt a complete buffer with Secret or Test key.<BR>
 * The secret key corresponds to the private key provided from the CRA.
 * <B>These keys must be stored in a secure place.</B> <BR>
 * Can be hardcoded or soft coded (iv vector contains '0')
 *
 * \param[out] sfx_u8 *encrypted_data            Result of AES Encryption
 * \param[in] sfx_u8 *data_to_encrypt            Input data to Encrypt
 * \param[in] sfx_u8 aes_block_len               Input data length (should be a multiple of an AES block size, ie. AES_BLOCK_SIZE bytes)
 * \param[in] sfx_u8 key[16]                     Input key
 * \param[in] sfx_credentials_use_key_t use_key  Key to use - private key or input key
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_AES:                      AES Encryption error
 *******************************************************************/
sfx_u8 MCU_API_aes_128_cbc_encrypt(sfx_u8* encrypted_data, sfx_u8* data_to_encrypt, sfx_u8 aes_block_len, sfx_u8 key[AES_BLOCK_SIZE], sfx_credentials_use_key_t use_key) {

	/* Local variables */
	unsigned char byte_idx = 0;
	unsigned char local_key[AES_BLOCK_SIZE] = {0};
	unsigned char init_vector[AES_BLOCK_SIZE] = {0};
	unsigned char data_in[AES_BLOCK_SIZE] = {0};
	unsigned char data_out[AES_BLOCK_SIZE] = {0};
	unsigned char number_of_blocks = aes_block_len / AES_BLOCK_SIZE;
	unsigned char block_idx;
	unsigned char key_byte = 0;

	/* Get accurate key */
	switch (use_key) {
		case CREDENTIALS_PRIVATE_KEY:
			// Retrieve device key from NVM.
			for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE ; byte_idx++) {
				NVM_ReadByte(NVM_SIGFOX_KEY_ADDRESS_OFFSET+byte_idx, &key_byte);
				local_key[byte_idx] = key_byte;
			}
			break;
		case CREDENTIALS_KEY_IN_ARGUMENT:
			// Use key in argument.
			for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE ; byte_idx++) {
				local_key[byte_idx] = key[byte_idx];
			}
			break;
		default:
			break;
	}

	/* Init AES peripheral */
	AES_Init();

	/* Perform encryption */
	for (block_idx=0; block_idx<number_of_blocks ; block_idx++) {
		// Fill data in and initialization vector with previous result.
		for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE; byte_idx++) data_in[byte_idx] = data_to_encrypt[(block_idx * AES_BLOCK_SIZE) + byte_idx];
		for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE; byte_idx++) init_vector[byte_idx] = data_out[byte_idx];
		// Run algorithme.
		AES_EncodeCbc(data_in, data_out, init_vector, local_key);
	}

	/* Get last result */
	for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE; byte_idx++) encrypted_data[byte_idx] = data_out[byte_idx];

	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE])
 * \brief This function copies the data read from non volatile memory
 * into the buffer pointed by read_data.<BR>
 * The size of the data to read is \link SFX_NVMEM_BLOCK_SIZE \endlink
 * bytes.
 * CAREFUL : this value can change according to the features included
 * in the library (covered zones, etc.)
 *
 * \param[in] none
 * \param[out] sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE] Pointer to the data bloc to write with the data stored in memory
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GETNVMEM:                 Read nvmem error
 *******************************************************************/
sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE]) {

	// read_data is expected as follow:
	// ______________________________
	// |0    1|2     3|4    5|  6   |
	// |      |       |      |      |
	// |  PN  |  SEQ  |  FH  |  RL  |
	// |______|_______|______|______|

	// Enable NVM interface.
	NVM_Enable();
	// PN.
	NVM_ReadByte(NVM_SIGFOX_PN_ADDRESS_OFFSET, &(read_data[SFX_NVMEM_PN]));
	NVM_ReadByte(NVM_SIGFOX_PN_ADDRESS_OFFSET+1, &(read_data[SFX_NVMEM_PN+1]));
	// Sequence number.
	NVM_ReadByte(NVM_SIGFOX_SEQ_NUM_ADDRESS_OFFSET, &(read_data[SFX_NVMEM_SEQ_NUM]));
	NVM_ReadByte(NVM_SIGFOX_SEQ_NUM_ADDRESS_OFFSET+1, &(read_data[SFX_NVMEM_SEQ_NUM+1]));
	// FH.
	NVM_ReadByte(NVM_SIGFOX_FH_ADDRESS_OFFSET, &(read_data[SFX_NVMEM_FH]));
	NVM_ReadByte(NVM_SIGFOX_FH_ADDRESS_OFFSET+1, &(read_data[SFX_NVMEM_FH+1]));
	// RL.
	NVM_ReadByte(NVM_SIGFOX_RL_ADDRESS_OFFSET, &(read_data[SFX_NVMEM_RL]));
	// Disable NVM interface.
	NVM_Enable();

	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE])
 * \brief This function writes data pointed by data_to_write to non
 * volatile memory.<BR> It is strongly recommanded to use NV memory
 * like EEPROM since this function is called at each SIGFOX_API_send_xxx.
 * The size of the data to write is \link SFX_NVMEM_BLOCK_SIZE \endlink
 * bytes.
 * CAREFUL : this value can change according to the features included
 * in the library (covered zones, etc.)
 *
 * \param[in] sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE] Pointer to data bloc to be written in memory
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_SETNVMEM:                 Write nvmem error
 *******************************************************************/
sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE]) {

	// data_to_write is provided as follow:
	// ______________________________
	// |0    1|2     3|4    5|  6   |
	// |      |       |      |      |
	// |  PN  |  SEQ  |  FH  |  RL  |
	// |______|_______|______|______|

	// Enable NVM interface.
	NVM_Enable();
	// PN.
	NVM_WriteByte(NVM_SIGFOX_PN_ADDRESS_OFFSET, data_to_write[SFX_NVMEM_PN]);
	NVM_WriteByte(NVM_SIGFOX_PN_ADDRESS_OFFSET+1, data_to_write[SFX_NVMEM_PN+1]);
	// Sequence number.
	NVM_WriteByte(NVM_SIGFOX_SEQ_NUM_ADDRESS_OFFSET, data_to_write[SFX_NVMEM_SEQ_NUM]);
	NVM_WriteByte(NVM_SIGFOX_SEQ_NUM_ADDRESS_OFFSET+1, data_to_write[SFX_NVMEM_SEQ_NUM+1]);
	// FH.
	NVM_WriteByte(NVM_SIGFOX_FH_ADDRESS_OFFSET, data_to_write[SFX_NVMEM_FH]);
	NVM_WriteByte(NVM_SIGFOX_FH_ADDRESS_OFFSET+1, data_to_write[SFX_NVMEM_FH+1]);
	// RL.
	NVM_WriteByte(NVM_SIGFOX_RL_ADDRESS_OFFSET, data_to_write[SFX_NVMEM_RL]);
	// Disable NVM interface.
	NVM_Enable();

	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
 * \brief Start timer for :
 * - carrier sense maximum window (used in ARIB standard)
 *
 * \param[in] sfx_u16 time_duration_in_ms        Timer value in milliseconds
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_START_CS:           Start CS timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s)
 * \brief Start timer for in second duration
 *
 * \param[in] sfx_u32 time_duration_in_s         Timer value in seconds
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_START:              Start timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_stop(void)
 * \brief Stop the timer (started with MCU_API_timer_start)
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_STOP:               Stop timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_stop(void) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_stop_carrier_sense(void)
 * \brief Stop the timer (started with MCU_API_timer_start_carrier_sense)
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_STOP_CS:            Stop timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_stop_carrier_sense(void) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_timer_wait_for_end(void)
 * \brief Blocking function to wait for interrupt indicating timer
 * elapsed.<BR> This function is only used for the 20 seconds wait
 * in downlink.
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TIMER_END:                Wait end of timer error
 *******************************************************************/
sfx_u8 MCU_API_timer_wait_for_end(void) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi)
 * \brief To report the result of Rx test for each valid message
 * received/validated by library.<BR> Manufacturer api to show the result
 * of RX test mode : can be uplink radio frame or uart print or
 * gpio output.
 * RSSI parameter is only used to report the rssi of received frames (downlink test)
 *
 * \param[in] sfx_bool status                    Is SFX_TRUE when result ok else SFX_FALSE
 *                                               See SIGFOX_API_test_mode summary
 * \param[out] rssi                              RSSI of the received frame
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_TEST_REPORT:              Report test result error
 *******************************************************************/
sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi) {
	// TBC: print result on LPUART.
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_version(sfx_u8 **version, sfx_u8 *size)
 * \brief Returns current MCU API version
 *
 * \param[out] sfx_u8 **version                  Pointer to Byte array (ASCII format) containing library version
 * \param[out] sfx_u8 *size                      Size of the byte array pointed by *version
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GET_VERSION:              Get Version error
 *******************************************************************/
sfx_u8 MCU_API_get_version(sfx_u8** version, sfx_u8* size) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_device_id_and_payload_encryption_flag(sfx_u8 dev_id[ID_LENGTH], sfx_bool *payload_encryption_enabled)
 * \brief This function copies the device ID in dev_id, and
 * the payload encryption flag in payload_encryption_enabled.
 *
 * \param[in]  none
 * \param[out] sfx_u8 dev_id[ID_LENGTH]          Pointer on the device ID
 * \param[out] sfx_bool *payload_encryption_enabled  Payload is encrypted if SFX_TRUE, not encrypted else
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GET_ID_PAYLOAD_ENCR_FLAG: Error when getting device ID or payload encryption flag
 *******************************************************************/
sfx_u8 MCU_API_get_device_id_and_payload_encryption_flag(sfx_u8 dev_id[ID_LENGTH], sfx_bool* payload_encryption_enabled) {
	// Get device ID.
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<ID_LENGTH ; byte_idx++) {
		NVM_ReadByte(NVM_SIGFOX_ID_ADDRESS_OFFSET+byte_idx, &(dev_id[byte_idx]));
	}
	// No payload encryption.
	(*payload_encryption_enabled) = SFX_FALSE;
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_initial_pac(sfx_u8 initial_pac[PAC_LENGTH])
 * \brief Get the value of the initial PAC stored in the device. This
 * value is used when the device is registered for the first time on
 * the backend.
 *
 * \param[in]  none
 * \param[out] sfx_u8 *initial_pac               Pointer to initial PAC
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GET_PAC:                  Error when getting initial PAC
 *******************************************************************/
sfx_u8 MCU_API_get_initial_pac(sfx_u8 initial_pac[PAC_LENGTH]) {
	// Get device initial PAC.
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<PAC_LENGTH ; byte_idx++) {
		NVM_ReadByte(NVM_SIGFOX_INITIAL_PAC_ADDRESS_OFFSET+byte_idx, &(initial_pac[byte_idx]));
	}
	return SFX_ERR_NONE;
}
