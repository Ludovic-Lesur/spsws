/*
 * math.c
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#include "math.h"

/*** MATH local macros ***/

#define MATH_MEDIAN_FILTER_LENGTH_MAX	0xFF
#define MATH_DECIMAL_MAX_DIGITS			10

/*** MATH functions ***/

/* COMPUTE A POWER A 10.
 * @param power:	Desired power.
 * @return result:	Result of computation.
 */
unsigned int MATH_pow_10(unsigned char power) {
	// Local variables.
	unsigned int result = 0;
	unsigned int pow10_buf[MATH_DECIMAL_MAX_DIGITS] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
	// Check overflow.
	if (power < MATH_DECIMAL_MAX_DIGITS) {
		result = pow10_buf[power];
	}
	return result;
}

/* COMPUTE AVERAGE VALUE.
 * @param data:			Input buffer.
 * @param data_length:	Input buffer length.
 * @return average: 	Average value of the input buffer.
 */
unsigned int MATH_average(unsigned int* data, unsigned char data_length) {
	// Local variables.
	unsigned char idx = 0;
	unsigned int average = 0;
	// Compute
	for (idx=0 ; idx<data_length ; idx++) {
		average = ((average * idx) + data[idx]) / (idx + 1);
	}
	return average;
}

/* COMPUTE AVERAGE MEDIAN VALUE
 * @param data:				Input buffer.
 * @param median_length:	Number of elements taken for median value search.
 * @param average_length:	Number of center elements taken for final average.
 * @return filter_out:		Output value of the median filter.
 */
unsigned int MATH_median_filter(unsigned int* data, unsigned char median_length, unsigned char average_length) {
	// Local variables.
	unsigned int local_buf[MATH_MEDIAN_FILTER_LENGTH_MAX];
	unsigned char buffer_sorted = 0;
	unsigned char idx1 = 0;
	unsigned char idx2 = 0;
	unsigned char start_idx = 0;
	unsigned char end_idx = 0;
	unsigned int filter_out = 0;
	unsigned int temp = 0;
	// Copy input buffer into local buffer.
	for (idx1=0 ; idx1<median_length ; idx1++) {
		local_buf[idx1] = data[idx1];
	}
	// Sort buffer in ascending order.
	for (idx1=0; idx1<median_length; ++idx1) {
		buffer_sorted = 1;
		for (idx2=1 ; idx2<(median_length-idx1) ; ++idx2) {
			if (local_buf[idx2 - 1] > local_buf[idx2]) {
				temp = local_buf[idx2 - 1];
				local_buf[idx2 - 1] = local_buf[idx2];
				local_buf[idx2] = temp;
				buffer_sorted = 0;
			}
		}
		if (buffer_sorted != 0) break;
	}
	// Compute average of center values if required.
	if (average_length > 0) {
		// Clamp value.
		if (average_length > median_length) {
			average_length = median_length;
		}
		start_idx = (median_length / 2) - (average_length / 2);
		end_idx = (median_length / 2) + (average_length / 2);
		if (end_idx >= median_length) {
			end_idx = (median_length - 1);
		}
		// Compute average.
		filter_out = MATH_average(&(data[start_idx]), (end_idx - start_idx + 1));
	}
	else {
		// Return median value.
		filter_out = local_buf[(median_length / 2)];
	}
	return filter_out;
}

#if (defined CM || defined ATM)
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
 * @param x:	x parameter.
 * @param y:	y parameter.
 * @return:		Angle of the point (x,y).
 */
unsigned int MATH_atan2(signed int x, signed int y) {
	// Local variables.
	unsigned int alpha = MATH_ERROR_VALUE;
	signed int local_x = x;
	signed int local_y = y;
	unsigned int abs_x = 0;
	unsigned int abs_y = 0;
	// Check x and y are not null.
	if ((x != 0) || (y != 0)) {
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
			alpha = (((45 * abs_y) << 10) / (abs_x)) >> 10; // Quadrant 1.
			// Add offset depending on quadrant.
			if ((x > 0) && (y < 0)) {
				// Quadrant 8.
				alpha = (360 - alpha);
			}
			if (x < 0) {
				if (y > 0) {
					// Quadrant 4.
					alpha = (180 - alpha);
				}
				else {
					// Quadrant 5.
					alpha = (180 + alpha);
				}
			}
		}
		else {
			// Use arctan approximation: arctan(z)=(pi/4)*z.
			alpha = (((45 * abs_x) << 10) / (abs_y)) >> 10;
			// Add offset depending on quadrant and arctan(1/z)=+/-90-arctan(z).
			if (x > 0) {
				if (y > 0) {
					// Quadrant 2.
					alpha = (90 - alpha);
				}
				else {
					// Quadrant 7.
					alpha = (270 + alpha);
				}
			}
			else {
				if (y > 0) {
					// Quadrant 3.
					alpha = (90 + alpha);
				}
				else {
					// Quadrant 6.
					alpha = (270 - alpha);
				}
			}
		}
		// Ensure angle is in [0,359] range.
		alpha = (alpha % 360);
	}
	return (alpha);
}
#endif

/* COMPUTE THE TWO'S COMPLEMENT OF A GIVEN VALUE.
 * @param value:				Input unsigned value.
 * @param sign_bit_position:	Position of the sign bit in the input.
 * @return result:				Result of computation.
 */
signed int MATH_two_complement(unsigned int value, unsigned char sign_bit_position) {
	// Local variables.
	signed int result = 0;
	unsigned char bit_idx = 0;
	unsigned int not_value = 0;
	unsigned int absolute_value = 0;
	// Check sign bit.
	if ((value & (0b1 << sign_bit_position)) == 0) {
		// Value is positive: nothing to do.
		result = (int) value;
	}
	else {
		// Value is negative.
		for (bit_idx=0 ; bit_idx<=sign_bit_position ; bit_idx++) {
			if ((value & (0b1 << bit_idx)) == 0) {
				not_value |= (0b1 << bit_idx);
			}
		}
		absolute_value = not_value + 1;
		result = (-1) * absolute_value;
	}
	return result;
}

/* COMPUTE THE ONE'S COMPLEMENT OF A GIVEN VALUE.
 * @param value:				Input signed value.
 * @param sign_bit_position:	Position of the sign bit in the output.
 * @return result:				Result of computation.
 */
unsigned int MATH_one_complement(signed int value, unsigned char sign_bit_position) {
	// Local variables.
	unsigned int result = 0;
	unsigned int absolute_value = 0;
	unsigned int absolute_mask = ((0b1 << sign_bit_position) - 1);
	// Check value sign.
	if (value >= 0) {
		// Value is positive: nothing to do.
		result = ((unsigned int) value) & absolute_mask;
	}
	else {
		// Set sign bit.
		absolute_value = (unsigned int) ((-1) * value);
		result = (0b1 << sign_bit_position) | (absolute_value & absolute_mask);
	}
	return result;
}
