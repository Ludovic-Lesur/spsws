/*
 * filter.c
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#include "filter.h"

/*** FILTER local macros ***/

#define FILTER_MEDIAN_LENGTH_MAX	0xFF

/*** FILTER functions ***/

/* COMPUTE AVERAGE MEDIAN VALUE
 * @param buf:				Input buffer.
 * @param median_length:	Number of elements taken for median value search.
 * @param average_length:	Number of center elements taken for final average.
 * @return filter_out:		Output value of the median filter.
 */
unsigned int FILTER_ComputeMedianFilter(unsigned int* buf, unsigned char median_length, unsigned char average_length) {
	// Local variables.
	unsigned int local_buf[FILTER_MEDIAN_LENGTH_MAX];
	unsigned char buffer_sorted = 0;
	unsigned char idx1 = 0;
	unsigned char idx2 = 0;
	unsigned char start_idx = 0;
	unsigned char end_idx = 0;
	unsigned int sum = 0;
	unsigned int filter_out = 0;
	// Copy input buffer into local buffer.
	for (idx1=0 ; idx1<median_length ; idx1++) {
		local_buf[idx1] = buf[idx1];
	}
	// Pad with zeroes.
	for (; idx1<FILTER_MEDIAN_LENGTH_MAX ; idx1++) {
		local_buf[idx1] = 0;
	}
	// Sort buffer in ascending order.
	for (idx1=0; idx1<median_length; ++idx1) {
		buffer_sorted = 1;
		for (idx2=1 ; idx2<(median_length-idx1) ; ++idx2) {
			if (local_buf[idx2 - 1] > local_buf[idx2]) {
				unsigned int temp = local_buf[idx2 - 1];
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
		for (idx1=start_idx ; idx1<(end_idx+1) ; idx1++) {
			sum += local_buf[idx1];
		}
		// Compute average.
		filter_out = ((sum) / (end_idx - start_idx + 1));
	}
	else {
		// Return median value.
		filter_out = local_buf[(median_length / 2)];
	}
	return filter_out;
}
