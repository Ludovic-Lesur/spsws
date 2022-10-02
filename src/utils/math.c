/*
 * math.c
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#include "math.h"
#include "types.h"

/*** MATH local macros ***/

#define MATH_MEDIAN_FILTER_LENGTH_MAX	0xFF
static const uint32_t MATH_POW10[MATH_DECIMAL_MAX_LENGTH] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/*** MATH local functions ***/

/* GENERIC FUNCTION TO GET MINIMUM VALUE OF AN ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Length of the input buffer.
 * @return min:			Minimum value of the buffer.
 */
#define _MATH_min(data, data_length) { \
	uint8_t idx = 0; \
	for (idx=0 ; idx<data_length ; idx++) { \
		if (data[idx] < min) { \
			min = data[idx]; \
		} \
	} \
	return min; \
}

/* GENERIC FUNCTION TO GET MINIMUM VALUE OF AN ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Length of the input buffer.
 * @return min:			Minimum value of the buffer.
 */
#define _MATH_max(data, data_length) { \
	uint8_t idx = 0; \
	for (idx=0 ; idx<data_length ; idx++) { \
		if (data[idx] > max) { \
			max = data[idx]; \
		} \
	} \
	return max; \
}

/* GENERIC FUNCTION TO COMPUTE AVERAGE VALUE OF AN ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Length of the input buffer.
 * @return average:		Mean value of the buffer.
 */
#define _MATH_average(data, data_length) { \
	uint8_t idx = 0; \
	for (idx=0 ; idx<data_length ; idx++) { \
		average = ((average * idx) + data[idx]) / (idx + 1); \
	} \
	return average; \
}

/* GENERIC FUNCTION TO COMPUTE AVERAGE MEDIAN VALUE OF AN ARRAY.
 * @param data:				Input buffer.
 * @param median_length:	Number of elements taken for median value search.
 * @param average_length:	Number of center elements taken for final average.
 * @return filter_out:		Output value of the median filter.
 */
#define _MATH_median_filter(data, median_length, average_length) { \
	uint8_t buffer_sorted = 0; \
	uint8_t idx1 = 0; \
	uint8_t idx2 = 0; \
	/* Copy input buffer into local buffer */ \
	for (idx1=0 ; idx1<median_length ; idx1++) { \
		local_buf[idx1] = data[idx1]; \
	} \
	/* Sort buffer in ascending order. */ \
	for (idx1=0; idx1<median_length; ++idx1) { \
		buffer_sorted = 1; \
		for (idx2=1 ; idx2<(median_length-idx1) ; ++idx2) { \
			if (local_buf[idx2 - 1] > local_buf[idx2]) { \
				temp = local_buf[idx2 - 1]; \
				local_buf[idx2 - 1] = local_buf[idx2]; \
				local_buf[idx2] = temp; \
				buffer_sorted = 0; \
			} \
		} \
		if (buffer_sorted != 0) break; \
	} \
	/* Compute start and end indexes for final averaging */ \
	if (average_length > 0) { \
		/* Clamp value */ \
		if (average_length > median_length) { \
			average_length = median_length; \
		} \
		start_idx = (median_length / 2) - (average_length / 2); \
		end_idx = (median_length / 2) + (average_length / 2); \
		if (end_idx >= median_length) { \
			end_idx = (median_length - 1); \
		} \
	} \
}

/*** MATH functions ***/

/* COMPUTE A POWER A 10.
 * @param power:	Desired power.
 * @param result:	Pointer to the result.
 * @return status:	Function execution status.
 */
MATH_status_t MATH_pow_10(uint8_t power, uint32_t* result) {
	// Local variables.
	MATH_status_t status = MATH_SUCCESS;
	// Check power.
	if (power >= MATH_DECIMAL_MAX_LENGTH) {
		status = MATH_ERROR_OVERFLOW;
		goto errors;
	}
	(*result) = MATH_POW10[power];
errors:
	return status;
}

/* GET MINIMUM VALUE OF A 8-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
uint8_t MATH_min_u8(uint8_t* data, uint8_t data_length) {
	// Local variables.
	uint8_t min = 0xFF;
	// Compute minimum value.
	_MATH_min(data, data_length);
}

/* GET MINIMUM VALUE OF A 16-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
uint16_t MATH_min_u16(uint16_t* data, uint8_t data_length) {
	// Local variables.
	uint16_t min = 0xFFFF;
	// Compute minimum value.
	_MATH_min(data, data_length);
}

/* GET MINIMUM VALUE OF A 32-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
uint32_t MATH_min_u32(uint32_t* data, uint8_t data_length) {
	// Local variables.
	uint32_t min = 0xFFFFFFFF;
	// Compute minimum value.
	_MATH_min(data, data_length);
}

/* GET MAXIMUM VALUE OF A 8-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
uint8_t MATH_max_u8(uint8_t* data, uint8_t data_length) {
	// Local variables.
	uint8_t max = 0;
	// Compute minimum value.
	_MATH_max(data, data_length);
}

/* GET MAXIMUM VALUE OF A 16-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
uint16_t MATH_max_u16(uint16_t* data, uint8_t data_length) {
	// Local variables.
	uint16_t max = 0;
	// Compute minimum value.
	_MATH_max(data, data_length);
}

/* GET MAXIMUM VALUE OF A 32-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
uint32_t MATH_max_u32(uint32_t* data, uint8_t data_length) {
	// Local variables.
	uint32_t max = 0;
	// Compute minimum value.
	_MATH_max(data, data_length);
}

/* COMPUTE AVERAGE VALUE OF A 8-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return average: 	Average value of the input buffer.
 */
uint8_t MATH_average_u8(uint8_t* data, uint8_t data_length) {
	// Local variables.
	uint8_t average = 0;
	// Compute average.
	_MATH_average(data, data_length);
}

/* COMPUTE AVERAGE VALUE OF A 16-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return average: 	Average value of the input buffer.
 */
uint16_t MATH_average_u16(uint16_t* data, uint8_t data_length) {
	// Local variables.
	uint16_t average = 0;
	// Compute average.
	_MATH_average(data, data_length);
}

/* COMPUTE AVERAGE VALUE OF A 32-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return average: 	Average value of the input buffer.
 */
uint32_t MATH_average_u32(uint32_t* data, uint8_t data_length) {
	// Local variables.
	uint32_t average = 0;
	// Compute average.
	_MATH_average(data, data_length);
}

/* COMPUTE AVERAGE MEDIAN VALUE OF A 8-BITS VALUES ARRAY.
 * @param data:				Input buffer.
 * @param median_length:	Number of elements taken for median value search.
 * @param average_length:	Number of center elements taken for final average.
 * @return filter_out:		Output value of the median filter.
 */
uint8_t MATH_median_filter_u8(uint8_t* data, uint8_t median_length, uint8_t average_length) {
	// Local variables.
	uint8_t filter_out = 0;
	uint8_t local_buf[MATH_MEDIAN_FILTER_LENGTH_MAX];
	uint8_t temp = 0;
	uint8_t start_idx = 0;
	uint8_t end_idx = 0;
	// Compute median filter.
	_MATH_median_filter(data, median_length, average_length);
	// Compute average or median value.
	filter_out = (average_length > 0)? MATH_average_u8(&(data[start_idx]), (end_idx - start_idx + 1)) : local_buf[(median_length / 2)];
	return filter_out;
}

/* COMPUTE AVERAGE MEDIAN VALUE OF A 16-BITS VALUES ARRAY.
 * @param data:				Input buffer.
 * @param median_length:	Number of elements taken for median value search.
 * @param average_length:	Number of center elements taken for final average.
 * @return filter_out:		Output value of the median filter.
 */
uint16_t MATH_median_filter_u16(uint16_t* data, uint8_t median_length, uint8_t average_length) {
	// Local variables.
	uint16_t filter_out = 0;
	uint16_t local_buf[MATH_MEDIAN_FILTER_LENGTH_MAX];
	uint16_t temp = 0;
	uint8_t start_idx = 0;
	uint8_t end_idx = 0;
	// Compute median filter.
	_MATH_median_filter(data, median_length, average_length);
	// Compute average or median value.
	filter_out = (average_length > 0)? MATH_average_u16(&(data[start_idx]), (end_idx - start_idx + 1)) : local_buf[(median_length / 2)];
	return filter_out;
}

/* COMPUTE AVERAGE MEDIAN VALUE OF A 32-BITS VALUES ARRAY.
 * @param data:				Input buffer.
 * @param median_length:	Number of elements taken for median value search.
 * @param average_length:	Number of center elements taken for final average.
 * @return filter_out:		Output value of the median filter.
 */
uint32_t MATH_median_filter_u32(uint32_t* data, uint8_t median_length, uint8_t average_length) {
	// Local variables.
	uint32_t filter_out = 0;
	uint32_t local_buf[MATH_MEDIAN_FILTER_LENGTH_MAX];
	uint32_t temp = 0;
	uint8_t start_idx = 0;
	uint8_t end_idx = 0;
	// Compute median filter.
	_MATH_median_filter(data, median_length, average_length);
	// Compute average or median value.
	filter_out = (average_length > 0)? MATH_average_u32(&(data[start_idx]), (end_idx - start_idx + 1)) : local_buf[(median_length / 2)];
	return filter_out;
}

/* COMPUTE ABSOLUTE VALUE.
 * @param x:	Parameter.
 * @return:		|x|.
 */
uint32_t MATH_abs(int32_t x) {
	// Local variables.
	uint32_t result = 0;
	// Check sign.
	if (x > 0) result = x;
	if (x < 0) result = (-1) * x;
	return result;
}

/* COMPUTE ATAN2 FUNCTION.
 * @param x:		x parameter.
 * @param y:		y parameter.
 * @param alpha:	Pointer to the angle of the point (x,y).
 * @return status:	Function execution status.
 */
MATH_status_t MATH_atan2(int32_t x, int32_t y, uint32_t* alpha) {
	// Local variables.
	MATH_status_t status = MATH_SUCCESS;
	int32_t local_x = x;
	int32_t local_y = y;
	uint32_t abs_x = 0;
	uint32_t abs_y = 0;
	// Check x and y are not null.
	if ((x == 0) && (y == 0)) {
		status = MATH_ERROR_UNDEFINED;
		goto errors;
	}
	// Scale x and y to avoid overflow.
	while ((MATH_abs(local_x) > 10000) || (MATH_abs(local_y) > 10000)) {
		local_x = local_x >> 1;
		local_y = local_y >> 1;
	}
	// Compute atan2 function.
	abs_x = MATH_abs(local_x);
	abs_y = MATH_abs(local_y);
	// Use the quotient within [-1,1]
	if (abs_x >= abs_y) {
		// Use arctan approximation: arctan(z)=(pi/4)*z.
		(*alpha) = (((45 * abs_y) << 10) / (abs_x)) >> 10; // Quadrant 1.
		// Add offset depending on quadrant.
		if ((x > 0) && (y < 0)) {
			(*alpha) = (360 - (*alpha)); // Quadrant 8.
		}
		if (x < 0) {
			if (y > 0) {
				(*alpha) = (180 - (*alpha)); // Quadrant 4.
			}
			else {
				(*alpha) = (180 + (*alpha)); // Quadrant 5.
			}
		}
	}
	else {
		// Use arctan approximation: arctan(z)=(pi/4)*z.
		(*alpha) = (((45 * abs_x) << 10) / (abs_y)) >> 10;
		// Add offset depending on quadrant and arctan(1/z)=+/-90-arctan(z).
		if (x > 0) {
			if (y > 0) {
				(*alpha) = (90 - (*alpha)); // Quadrant 2.
			}
			else {
				(*alpha) = (270 + (*alpha)); // Quadrant 7.
			}
		}
		else {
			if (y > 0) {
				(*alpha) = (90 + (*alpha)); // Quadrant 3.
			}
			else {
				(*alpha) = (270 - (*alpha)); // Quadrant 6.
			}
		}
	}
	// Ensure angle is in [0,359] range.
	(*alpha) = ((*alpha) % 360);
errors:
	return status;
}

/* COMPUTE THE TWO'S COMPLEMENT OF A GIVEN VALUE.
 * @param value:				Input unsigned value.
 * @param sign_bit_position:	Position of the sign bit in the input.
 * @param result:				Pointer to the result.
 * @return status:				Function execution status.
 */
MATH_status_t MATH_two_complement(uint32_t value, uint8_t sign_bit_position, int32_t* result) {
	// Local variables.
	MATH_status_t status = MATH_SUCCESS;
	uint8_t bit_idx = 0;
	uint32_t not_value = 0;
	uint32_t absolute_value = 0;
	// Check parameters.
	if (sign_bit_position > (MATH_BINARY_MAX_LENGTH - 1)) {
		status = MATH_ERROR_SIGN_BIT;
		goto errors;
	}
	// Check sign bit.
	if ((value & (0b1 << sign_bit_position)) == 0) {
		// Value is positive: nothing to do.
		(*result) = (int32_t) value;
	}
	else {
		// Value is negative.
		for (bit_idx=0 ; bit_idx<=sign_bit_position ; bit_idx++) {
			if ((value & (0b1 << bit_idx)) == 0) {
				not_value |= (0b1 << bit_idx);
			}
		}
		absolute_value = not_value + 1;
		(*result) = (-1) * absolute_value;
	}
errors:
	return status;
}

/* COMPUTE THE ONE'S COMPLEMENT OF A GIVEN VALUE.
 * @param value:				Input signed value.
 * @param sign_bit_position:	Position of the sign bit in the output.
 * @return result:				Result of computation.
 */
MATH_status_t MATH_one_complement(int32_t value, uint8_t sign_bit_position, uint32_t* result) {
	// Local variables.
	MATH_status_t status = MATH_SUCCESS;
	uint32_t absolute_value = 0;
	uint32_t absolute_mask = ((0b1 << sign_bit_position) - 1);
	// Check parameters.
	if (sign_bit_position > (MATH_BINARY_MAX_LENGTH - 1)) {
		status = MATH_ERROR_SIGN_BIT;
		goto errors;
	}
	// Check value sign.
	if (value >= 0) {
		// Value is positive: nothing to do.
		(*result) = ((uint32_t) value) & absolute_mask;
	}
	else {
		// Set sign bit.
		absolute_value = (uint32_t) ((-1) * value);
		(*result) = (0b1 << sign_bit_position) | (absolute_value & absolute_mask);
	}
errors:
	return status;
}
