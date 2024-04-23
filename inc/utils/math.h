/*
 * math.h
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#ifndef __MATH_H__
#define __MATH_H__

#include "types.h"

/*** MATH macros ***/

#define MATH_BOOLEAN_DIGIT_MAX_NUMBER		1

#define MATH_HEXADECIMAL_DIGIT_MAX_NUMBER	8
#define MATH_HEXADECIMAL_DIGIT_MAX_VALUE	0x0F

#define MATH_DECIMAL_DIGIT_MAX_NUMBER		10
#define MATH_DECIMAL_DIGIT_MAX_VALUE		9

#define MATH_BINARY_DIGIT_MAX_NUMBER		32

/*** MATH global variables ***/

extern const int16_t MATH_COS_TABLE[360];
extern const int16_t MATH_SIN_TABLE[360];
extern const uint32_t MATH_POWER_10[MATH_DECIMAL_DIGIT_MAX_NUMBER];

/*** MATH structures ***/

/*!******************************************************************
 * \enum MATH_status_t
 * \brief MATH driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	MATH_SUCCESS = 0,
	MATH_ERROR_NULL_PARAMETER,
	MATH_ERROR_OVERFLOW,
	MATH_ERROR_UNDEFINED,
	MATH_ERROR_SIGN_BIT,
	MATH_ERROR_MAGNITUDE_OVERFLOW,
	// Last base value.
	MATH_ERROR_BASE_LAST = 0x0100
} MATH_status_t;

/*** MATH functions ***/

/*!******************************************************************
 * \fn MATH_status_t MATH_min_u8(uint8_t* data, uint8_t data_size, uint8_t* result)
 * \brief Returns the minimum value of a 8-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_min_u8(uint8_t* data, uint8_t data_size, uint8_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_min_u16(uint16_t* data, uint8_t data_size, uint16_t* result)
 * \brief Returns the minimum value of a 16-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_min_u16(uint16_t* data, uint8_t data_size, uint16_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_min_u32(uint32_t* data, uint8_t data_size, uint32_t* result)
 * \brief Returns the minimum value of a 32-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_min_u32(uint32_t* data, uint8_t data_size, uint32_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_max_u8(uint8_t* data, uint8_t data_size, uint8_t* result)
 * \brief Returns the maximum value of a 8-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_max_u8(uint8_t* data, uint8_t data_size, uint8_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_max_u16(uint16_t* data, uint8_t data_size, uint16_t* result)
 * \brief Returns the maximum value of a 16-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_max_u16(uint16_t* data, uint8_t data_size, uint16_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_max_u32(uint32_t* data, uint8_t data_size, uint32_t* result)
 * \brief Returns the maximum value of a 32-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_max_u32(uint32_t* data, uint8_t data_size, uint32_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_average_u8(uint8_t* data, uint8_t data_size, uint8_t* result)
 * \brief Compute the average value of a 8-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_average_u8(uint8_t* data, uint8_t data_size, uint8_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_average_u16(uint16_t* data, uint8_t data_size, uint16_t* result)
 * \brief Compute the average value of a 16-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_average_u16(uint16_t* data, uint8_t data_size, uint16_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_average_u32(uint32_t* data, uint8_t data_size, uint32_t* result)
 * \brief Compute the average value of a 32-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	data_size: Number of elements in the array.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_average_u32(uint32_t* data, uint8_t data_size, uint32_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_median_filter_u8(uint8_t* data, uint8_t median_size, uint8_t average_size, uint8_t* result)
 * \brief Compute an averaged median value of a 8-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	median_size: Number of elements used to compute the median filter.
 * \param[in]	average_size: Number of elements used to compute average of the median values.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_median_filter_u8(uint8_t* data, uint8_t median_size, uint8_t average_size, uint8_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_median_filter_u16(uint16_t* data, uint8_t median_size, uint8_t average_size, uint16_t* result)
 * \brief Compute an averaged median value of a 16-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	median_size: Number of elements used to compute the median filter.
 * \param[in]	average_size: Number of elements used to compute average of the median values.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_median_filter_u16(uint16_t* data, uint8_t median_size, uint8_t average_size, uint16_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_median_filter_u32(uint32_t* data, uint8_t median_size, uint8_t average_size, uint32_t* result)
 * \brief Compute an averaged median value of a 32-bits data array.
 * \param[in]  	data: Input array.
 * \param[in]	median_size: Number of elements used to compute the median filter.
 * \param[in]	average_size: Number of elements used to compute average of the median values.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_median_filter_u32(uint32_t* data, uint8_t median_size, uint8_t average_size, uint32_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_atan2(int32_t x, int32_t y, uint32_t* alpha)
 * \brief Compute the atan2 approximated value.
 * \param[in]  	x: Input argument 1.
 * \param[in]  	y: Input argument 2.
 * \param[out] 	alpha: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_atan2(int32_t x, int32_t y, uint32_t* alpha);

/*!******************************************************************
 * \fn MATH_status_t MATH_two_complement_to_int32(uint32_t value, uint8_t sign_bit_position, int32_t* result)
 * \brief Convert a two complement value with configurable sign bit position to the standard 32-bits two complement value.
 * \param[in]  	value: Two complement value to convert.
 * \param[in]  	sign_bit_position: Sign bit position expressed as a bit index from 0.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_two_complement_to_int32(uint32_t value, uint8_t sign_bit_position, int32_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_int32_to_signed_magnitude(int32_t value, uint8_t sign_bit_position, uint32_t* result)
 * \brief Convert a standard 32-bits two complement value to the corresponding signed magnitude representation.
 * \param[in]  	value: Two complement value to convert.
 * \param[in]  	sign_bit_position: Sign bit position in the result, expressed as a bit index from 0.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
MATH_status_t MATH_int32_to_signed_magnitude(int32_t value, uint8_t sign_bit_position, uint32_t* result);

/*!******************************************************************
 * \fn MATH_status_t MATH_abs(int32_t x, uint32_t* result)
 * \brief Compute the absolute value.
 * \param[in]  	x: Input argument.
 * \param[out] 	result: Pointer to the result.
 * \retval		Function execution status.
 *******************************************************************/
#define MATH_abs(x, y) { \
	if (x >= 0) { y = x; } \
	else { y = ((-1) * x); } \
}

/*!******************************************************************
 * \fn MATH_rolling_mean(value, number_of_samples, new_sample, value_type)
 * \brief Compute rolling mean.
 * \param[in]  	value: Current value to update.
 * \param[in]  	number_of_samples: Current number of samples.
 * \param[in]	new_sample: New sample to add.
 * \param[in]	value_type: Type of the value.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
#define MATH_rolling_mean(value, number_of_samples, new_sample, value_type) { \
	/* Compute rolling mean */ \
	value = (((value * ((value_type) number_of_samples)) + ((value_type) new_sample)) / ((value_type) (number_of_samples + 1))); \
	number_of_samples++; \
}

/*******************************************************************/
#define MATH_exit_error(error_base) { if (math_status != MATH_SUCCESS) { status = (error_base + math_status); goto errors; } }

/*******************************************************************/
#define MATH_stack_error(void) { if (math_status != MATH_SUCCESS) { ERROR_stack_add(ERROR_BASE_MATH + math_status); } }

/*******************************************************************/
#define MATH_stack_exit_error(error_code) { if (math_status != MATH_SUCCESS) { ERROR_stack_add(ERROR_BASE_MATH + math_status); status = error_code; goto errors; } }

#endif /* __MATH_H__ */
