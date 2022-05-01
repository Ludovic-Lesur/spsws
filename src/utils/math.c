/*
 * math.c
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#include "math.h"

/*** MATH local macros ***/

#define MATH_MEDIAN_FILTER_LENGTH_MAX	0xFF
static const unsigned int MATH_POW10[MATH_DECIMAL_MAX_LENGTH] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/*** MATH local functions ***/

/* GENERIC FUNCTION TO GET MINIMUM VALUE OF AN ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Length of the input buffer.
 * @return min:			Minimum value of the buffer.
 */
#define MATH_min(data, data_length) { \
	unsigned char idx = 0; \
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
#define MATH_max(data, data_length) { \
	unsigned char idx = 0; \
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
#define MATH_average(data, data_length) { \
	unsigned char idx = 0; \
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
#define MATH_median_filter(data, median_length, average_length) { \
	unsigned char buffer_sorted = 0; \
	unsigned char idx1 = 0; \
	unsigned char idx2 = 0; \
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
MATH_status_t MATH_pow_10(unsigned char power, unsigned int* result) {
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
unsigned char MATH_min_u8(unsigned char* data, unsigned char data_length) {
	// Local variables.
	unsigned char min = 0xFF;
	// Compute minimum value.
	MATH_min(data, data_length);
}

/* GET MINIMUM VALUE OF A 16-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
unsigned short MATH_min_u16(unsigned short* data, unsigned char data_length) {
	// Local variables.
	unsigned short min = 0xFFFF;
	// Compute minimum value.
	MATH_min(data, data_length);
}

/* GET MINIMUM VALUE OF A 32-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
unsigned int MATH_min_u32(unsigned int* data, unsigned char data_length) {
	// Local variables.
	unsigned int min = 0xFFFFFFFF;
	// Compute minimum value.
	MATH_min(data, data_length);
}

/* GET MAXIMUM VALUE OF A 8-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
unsigned char MATH_max_u8(unsigned char* data, unsigned char data_length) {
	// Local variables.
	unsigned char max = 0;
	// Compute minimum value.
	MATH_max(data, data_length);
}

/* GET MAXIMUM VALUE OF A 16-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
unsigned short MATH_max_u16(unsigned short* data, unsigned char data_length) {
	// Local variables.
	unsigned short max = 0;
	// Compute minimum value.
	MATH_max(data, data_length);
}

/* GET MAXIMUM VALUE OF A 32-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return min: 		Minimum value of the input buffer.
 */
unsigned int MATH_max_u32(unsigned int* data, unsigned char data_length) {
	// Local variables.
	unsigned int max = 0;
	// Compute minimum value.
	MATH_max(data, data_length);
}

/* COMPUTE AVERAGE VALUE OF A 8-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return average: 	Average value of the input buffer.
 */
unsigned char MATH_average_u8(unsigned char* data, unsigned char data_length) {
	// Local variables.
	unsigned char average = 0;
	// Compute average.
	MATH_average(data, data_length);
}

/* COMPUTE AVERAGE VALUE OF A 16-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return average: 	Average value of the input buffer.
 */
unsigned short MATH_average_u16(unsigned short* data, unsigned char data_length) {
	// Local variables.
	unsigned short average = 0;
	// Compute average.
	MATH_average(data, data_length);
}

/* COMPUTE AVERAGE VALUE OF A 32-BITS VALUES ARRAY.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return average: 	Average value of the input buffer.
 */
unsigned int MATH_average_u32(unsigned int* data, unsigned char data_length) {
	// Local variables.
	unsigned int average = 0;
	// Compute average.
	MATH_average(data, data_length);
}

/* COMPUTE AVERAGE MEDIAN VALUE OF A 8-BITS VALUES ARRAY.
 * @param data:				Input buffer.
 * @param median_length:	Number of elements taken for median value search.
 * @param average_length:	Number of center elements taken for final average.
 * @return filter_out:		Output value of the median filter.
 */
unsigned char MATH_median_filter_u8(unsigned char* data, unsigned char median_length, unsigned char average_length) {
	// Local variables.
	unsigned char filter_out = 0;
	unsigned char local_buf[MATH_MEDIAN_FILTER_LENGTH_MAX];
	unsigned char temp = 0;
	unsigned char start_idx = 0;
	unsigned char end_idx = 0;
	// Compute median filter.
	MATH_median_filter(data, median_length, average_length);
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
unsigned short MATH_median_filter_u16(unsigned short* data, unsigned char median_length, unsigned char average_length) {
	// Local variables.
	unsigned short filter_out = 0;
	unsigned short local_buf[MATH_MEDIAN_FILTER_LENGTH_MAX];
	unsigned short temp = 0;
	unsigned char start_idx = 0;
	unsigned char end_idx = 0;
	// Compute median filter.
	MATH_median_filter(data, median_length, average_length);
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
unsigned int MATH_median_filter_u32(unsigned int* data, unsigned char median_length, unsigned char average_length) {
	// Local variables.
	unsigned int filter_out = 0;
	unsigned int local_buf[MATH_MEDIAN_FILTER_LENGTH_MAX];
	unsigned int temp = 0;
	unsigned char start_idx = 0;
	unsigned char end_idx = 0;
	// Compute median filter.
	MATH_median_filter(data, median_length, average_length);
	// Compute average or median value.
	filter_out = (average_length > 0)? MATH_average_u32(&(data[start_idx]), (end_idx - start_idx + 1)) : local_buf[(median_length / 2)];
	return filter_out;
}

/* COMPUTE ABSOLUTE VALUE.
 * @param x:	Parameter.
 * @return:		|x|.
 */
unsigned int MATH_abs(signed int x) {
	// Local variables.
	unsigned int result = 0;
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
MATH_status_t MATH_atan2(signed int x, signed int y, unsigned int* alpha) {
	// Local variables.
	MATH_status_t status = MATH_SUCCESS;
	signed int local_x = x;
	signed int local_y = y;
	unsigned int abs_x = 0;
	unsigned int abs_y = 0;
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
MATH_status_t MATH_two_complement(unsigned int value, unsigned char sign_bit_position, signed int* result) {
	// Local variables.
	MATH_status_t status = MATH_SUCCESS;
	unsigned char bit_idx = 0;
	unsigned int not_value = 0;
	unsigned int absolute_value = 0;
	// Check parameters.
	if (sign_bit_position > (MATH_BINARY_MAX_LENGTH - 1)) {
		status = MATH_ERROR_SIGN_BIT;
		goto errors;
	}
	// Check sign bit.
	if ((value & (0b1 << sign_bit_position)) == 0) {
		// Value is positive: nothing to do.
		(*result) = (int) value;
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
MATH_status_t MATH_one_complement(signed int value, unsigned char sign_bit_position, unsigned int* result) {
	// Local variables.
	MATH_status_t status = MATH_SUCCESS;
	unsigned int absolute_value = 0;
	unsigned int absolute_mask = ((0b1 << sign_bit_position) - 1);
	// Check parameters.
	if (sign_bit_position > (MATH_BINARY_MAX_LENGTH - 1)) {
		status = MATH_ERROR_SIGN_BIT;
		goto errors;
	}
	// Check value sign.
	if (value >= 0) {
		// Value is positive: nothing to do.
		(*result) = ((unsigned int) value) & absolute_mask;
	}
	else {
		// Set sign bit.
		absolute_value = (unsigned int) ((-1) * value);
		(*result) = (0b1 << sign_bit_position) | (absolute_value & absolute_mask);
	}
errors:
	return status;
}
