/*
 * error.c
 *
 *  Created on: 27 feb. 2022
 *      Author: Ludo
 */

#include "error.h"

#include "types.h"

/*** ERROR local global variables ***/

static ERROR_t error_stack[ERROR_STACK_DEPTH];
static uint32_t error_stack_idx = 0;

/*** ERROR functions ***/

/* INIT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
void ERROR_stack_init(void) {
	// Reset stack.
	for (error_stack_idx=0 ; error_stack_idx<ERROR_STACK_DEPTH ; error_stack_idx++) error_stack[error_stack_idx] = SUCCESS;
	error_stack_idx = 0;
}

/* ADD STATUS TO ERROR STACK.
 * @param status:	Error code to add.
 * @return:			None.
 */
void ERROR_stack_add(ERROR_t status) {
	// Add error code.
	error_stack[error_stack_idx] = status;
	// Increment index.
	error_stack_idx++;
	if (error_stack_idx >= ERROR_STACK_DEPTH) {
		error_stack_idx = 0;
	}
}

/* READ ERROR STACK.
 * @param error_stack_ptr:	Pointer to the error stack.
 * @return:					None.
 */
void ERROR_stack_read(ERROR_t* error_stack_ptr) {
	// Local variables.
	uint32_t idx = 0;
	uint32_t chrono_idx = error_stack_idx;
	// Fill table in chronological order.
	for (idx=0 ; idx<ERROR_STACK_DEPTH ; idx++) {
		// Compute previous index.
		chrono_idx = (chrono_idx == 0) ? (ERROR_STACK_DEPTH - 1) : (chrono_idx - 1);
		error_stack_ptr[idx] = error_stack[chrono_idx];
	}
}

/* CHECK IF ERROR STACK IS EMPTY..
 * @param:	None.
 * @return:	1 if the stack is empty (no error), 0 otherwise.
 */
uint8_t ERROR_stack_is_empty(void) {
	// Local variables.
	uint8_t is_empty = 1;
	uint8_t idx = 0;
	// Loop on stack.
	for (idx=0 ; idx<ERROR_STACK_DEPTH ; idx++) {
		if (error_stack[idx] != SUCCESS) {
			is_empty = 0;
			break;
		}
	}
	return is_empty;
}
