/*
 * mcu_api.c
 *
 *  Created on: 18 juin 2018
 *      Author: Ludo
 */

#include "mcu_api.h"

#include "adc.h"
#include "at.h"
#include "aes.h"
#include "exti.h"
#include "iwdg.h"
#include "lptim.h"
#include "mode.h"
#include "nvm.h"
#include "pwr.h"
#include "rtc.h"
#include "sigfox_types.h"

/*** MCU API local macros ***/

#define MCU_API_MALLOC_BUFFER_SIZE		200

/*** MCU API local structures ***/

typedef struct {
	sfx_u8 malloc_buf[MCU_API_MALLOC_BUFFER_SIZE];
	sfx_u32 timer_duration_seconds;
} MCU_API_context_t;

/*** MCU API local global variables ***/

static MCU_API_context_t mcu_api_ctx;

/*** MCU API functions ***/

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
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	// Check size.
	if (size > MCU_API_MALLOC_BUFFER_SIZE) {
		status = SIGFOX_API_ERROR_MALLOC;
		goto errors;

	}
	(*returned_pointer) = &(mcu_api_ctx.malloc_buf[0]);
errors:
	return status;
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
	return SIGFOX_API_SUCCESS;
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
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	ADC_status_t adc_status = ADC_SUCCESS;
	unsigned int mcu_supply_voltage_mv = 0;
	signed char mcu_temperature_degrees = 0;
	// Perform measurements.
	adc_status = ADC1_init();
	ADC_status_check(SIGFOX_API_ERROR_BASE_ADC);
	adc_status = ADC1_perform_measurements();
	ADC_status_check(SIGFOX_API_ERROR_BASE_ADC);
	ADC1_disable();
	// Get MCU supply voltage.
	adc_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &mcu_supply_voltage_mv);
	ADC_status_check(SIGFOX_API_ERROR_BASE_ADC);
	(*voltage_idle) = (sfx_u16) mcu_supply_voltage_mv;
	(*voltage_tx) = (sfx_u16) mcu_supply_voltage_mv;
	// Get MCU internal temperature.
	ADC1_get_tmcu(&mcu_temperature_degrees);
	(*temperature) = ((sfx_s16) mcu_temperature_degrees) * 10; // Unit = 1/10 of degrees.
errors:
	return status;
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
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	LPTIM_status_t lptim1_status;
	switch (delay_type) {
	case SFX_DLY_INTER_FRAME_TX:
		// 0 to 2s in Uplink DC.
		lptim1_status = LPTIM1_delay_milliseconds(500, 1);
		LPTIM1_status_check(SIGFOX_API_ERROR_BASE_LPTIM);
		break;
	case SFX_DLY_INTER_FRAME_TRX:
		// 500 ms in Uplink/Downlink FH & Downlink DC.
		lptim1_status = LPTIM1_delay_milliseconds(500, 1);
		LPTIM1_status_check(SIGFOX_API_ERROR_BASE_LPTIM);
		break;
	case SFX_DLY_OOB_ACK:
		// 1.4s to 4s for Downlink OOB.
		lptim1_status = LPTIM1_delay_milliseconds(2000, 1);
		LPTIM1_status_check(SIGFOX_API_ERROR_BASE_LPTIM);
		break;
	case SFX_DLY_CS_SLEEP:
		// Delay between several trials of Carrier Sense (for the first frame only).
		lptim1_status = LPTIM1_delay_milliseconds(1000, 1);
		LPTIM1_status_check(SIGFOX_API_ERROR_BASE_LPTIM);
		break;
	default:
		status = SIGFOX_API_ERROR_DELAY_TYPE;
		goto errors;
	}
errors:
	return status;
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
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	AES_status_t aes_status = AES_SUCCESS;
	unsigned char byte_idx = 0;
	unsigned char local_key[AES_BLOCK_SIZE] = {0};
	unsigned char init_vector[AES_BLOCK_SIZE] = {0};
	unsigned char data_in[AES_BLOCK_SIZE] = {0};
	unsigned char data_out[AES_BLOCK_SIZE] = {0};
	unsigned char block_idx;
	unsigned char key_byte = 0;
	// Get accurate key.
	switch (use_key) {
		case CREDENTIALS_PRIVATE_KEY:
			// Retrieve device key from NVM.
			for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE ; byte_idx++) {
				NVM_enable();
				nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_DEVICE_KEY+byte_idx, &key_byte);
				NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
				NVM_disable();
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
			status = SIGFOX_API_ERROR_KEY_TYPE;
			goto errors;
	}
	// Perform encryption.
	AES_init();
	for (block_idx=0; block_idx<(aes_block_len / AES_BLOCK_SIZE) ; block_idx++) {
		// Fill input data and initialization vector with previous result.
		for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE; byte_idx++) data_in[byte_idx] = data_to_encrypt[(block_idx * AES_BLOCK_SIZE) + byte_idx];
		for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE; byte_idx++) init_vector[byte_idx] = data_out[byte_idx];
		// Run algorithme.
		aes_status = AES_encode_cbc(data_in, data_out, init_vector, local_key);
		AES_status_check(SIGFOX_API_ERROR_BASE_AES);
		// Fill output data.
		for (byte_idx=0 ; byte_idx<AES_BLOCK_SIZE; byte_idx++) encrypted_data[(block_idx * AES_BLOCK_SIZE) + byte_idx] = data_out[byte_idx];
	}
	AES_disable();
errors:
	return status;
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

	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	// PN.
	NVM_enable();
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_PN, &(read_data[SFX_NVMEM_PN]));
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_PN + 1, &(read_data[SFX_NVMEM_PN + 1]));
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	// Sequence number.
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER, &(read_data[SFX_NVMEM_MSG_COUNTER]));
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER + 1, &(read_data[SFX_NVMEM_MSG_COUNTER + 1]));
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	// FH.
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_FH, &(read_data[SFX_NVMEM_FH]));
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_FH + 1, &(read_data[SFX_NVMEM_FH + 1]));
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	// RL.
	nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_RL, &(read_data[SFX_NVMEM_RL]));
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	NVM_disable();
errors:
	return status;
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

	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	// PN.
	NVM_enable();
	nvm_status = NVM_write_byte(NVM_ADDRESS_SIGFOX_PN, data_to_write[SFX_NVMEM_PN]);
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	nvm_status = NVM_write_byte(NVM_ADDRESS_SIGFOX_PN + 1, data_to_write[SFX_NVMEM_PN + 1]);
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	// Sequence number.
	nvm_status = NVM_write_byte(NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER, data_to_write[SFX_NVMEM_MSG_COUNTER]);
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	nvm_status = NVM_write_byte(NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER + 1, data_to_write[SFX_NVMEM_MSG_COUNTER + 1]);
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	// FH.
	nvm_status = NVM_write_byte(NVM_ADDRESS_SIGFOX_FH, data_to_write[SFX_NVMEM_FH]);
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	nvm_status = NVM_write_byte(NVM_ADDRESS_SIGFOX_FH + 1, data_to_write[SFX_NVMEM_FH + 1]);
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	// RL.
	nvm_status = NVM_write_byte(NVM_ADDRESS_SIGFOX_RL, data_to_write[SFX_NVMEM_RL]);
	NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
	NVM_disable();
errors:
	return status;
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
	return SIGFOX_API_SUCCESS;
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
	// Save duration.
	mcu_api_ctx.timer_duration_seconds = time_duration_in_s;
	return SIGFOX_API_SUCCESS;
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
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	// Stop wake-up timer.
	rtc_status = RTC_stop_wakeup_timer();
	RTC_status_check(SIGFOX_API_ERROR_BASE_RTC);
errors:
	return status;
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
	return SIGFOX_API_SUCCESS;
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
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	unsigned int remaining_delay = mcu_api_ctx.timer_duration_seconds;
	unsigned int sub_delay = 0;
	// Clear watchdog.
	IWDG_reload();
	// Enter stop mode until GPIO interrupt or RTC wake-up.
	while (remaining_delay > 0) {
		// Compute sub-delay.
		sub_delay = (remaining_delay > IWDG_REFRESH_PERIOD_SECONDS) ? (IWDG_REFRESH_PERIOD_SECONDS) : (remaining_delay);
		remaining_delay -= sub_delay;
		// Start wake-up timer.
		rtc_status = RTC_start_wakeup_timer(sub_delay);
		RTC_status_check(SIGFOX_API_ERROR_BASE_RTC);
		// Enter stop mode.
		PWR_enter_stop_mode();
		// Wake-up: clear watchdog and flags.
		IWDG_reload();
		RTC_clear_wakeup_timer_flag();
		EXTI_clear_all_flags();
	}
errors:
	return status;
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
#ifdef ATM
	AT_print_test_result(status, rssi);
#endif
	return SIGFOX_API_SUCCESS;
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
	return SIGFOX_API_SUCCESS;
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
	// Local variables.
	SIGFOX_API_status_t status = SIGFOX_API_SUCCESS;
	NVM_status_t nvm_status = NVM_SUCCESS;
	unsigned char byte_idx = 0;
	// No payload encryption.
	(*payload_encryption_enabled) = SFX_FALSE;
	// Get device ID.
	for (byte_idx=0 ; byte_idx<ID_LENGTH ; byte_idx++) {
		NVM_enable();
		nvm_status = NVM_read_byte(NVM_ADDRESS_SIGFOX_DEVICE_ID+byte_idx, &(dev_id[byte_idx]));
		NVM_status_check(SIGFOX_API_ERROR_BASE_NVM);
		NVM_disable();
	}
errors:
	return status;
}

/*!******************************************************************
 * \fn sfx_u8 MCU_API_get_msg_counter_rollover(e_sfx_msg_counter_rollover* msgCounterRollover)
 * \brief This function copies the msg counter rollover value in msgCounterRollover.
 *
 * \param[in]  none
 * \param[out] e_sfx_msg_counter_rollover* msgCounterRollover   Pointer on the msg counter rollover
 *
 * \retval SFX_ERR_NONE:                         No error
 * \retval MCU_ERR_API_GET_MSG_COUNTER_ROLLOVER: Error when getting msg counter rollover
 *******************************************************************/
sfx_u8 MCU_API_get_msg_counter_rollover(e_sfx_msg_counter_rollover* msgCounterRollover) {
	(*msgCounterRollover) = SFX_MSG_COUNTER_ROLLOVER_4096;
	return SIGFOX_API_SUCCESS;
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
	return SIGFOX_API_SUCCESS;
}
