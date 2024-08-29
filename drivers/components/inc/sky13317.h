/*
 * sky13317.h
 *
 *  Created on: 15 dec. 2018
 *      Author: Ludo
 */

#ifndef __SKY13317_H__
#define __SKY13317_H__

/*** SKY13317 structures ***/

/*!******************************************************************
 * \enum SKY13317_status_t
 * \brief SKY13317 driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	SKY13317_SUCCESS,
	SKY13317_ERROR_CHANNEL,
	// Last base value.
	SKY13317_ERROR_BASE_LAST = 0x0100
} SKY13317_status_t;

/*!******************************************************************
 * \enum SKY13317_channel_t
 * \brief SKY13317 channels list.
 *******************************************************************/
typedef enum {
	SKY13317_CHANNEL_NONE,
#ifdef HW1_0
	SKY13317_CHANNEL_RF1,
#endif
	SKY13317_CHANNEL_RF2,
	SKY13317_CHANNEL_RF3,
	SKY13317_CHANNEL_LAST
} SKY13317_channel_t;

/*** SKY13317 functions ***/

/*!******************************************************************
 * \fn void SKY13317_init(void)
 * \brief Init SKY13317 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SKY13317_init(void);

/*!******************************************************************
 * \fn void SKY13317_de_init(void)
 * \brief Release SKY13317 driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void SKY13317_de_init(void);

/*!******************************************************************
 * \fn SKY13317_status_t SKY13317_set_channel(SKY13317_channel_t channel)
 * \brief Set SKY13317 channel.
 * \param[in]  	channel: Channel to select.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SKY13317_status_t SKY13317_set_channel(SKY13317_channel_t channel);

/*******************************************************************/
#define SKY13317_exit_error(base) { ERROR_check_exit(sky13317_status, SKY13317_SUCCESS, base) }

/*******************************************************************/
#define SKY13317_stack_error(base) { ERROR_check_stack(sky13317_status, SKY13317_SUCCESS, base) }

/*******************************************************************/
#define SKY13317_stack_exit_error(base, code) { ERROR_check_stack_exit(sky13317_status, SKY13317_SUCCESS, base, code) }

#endif /* __SKY13317_H__ */
