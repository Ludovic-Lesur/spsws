/*
 * neom8x_hw.c
 *
 *  Created on: 15 aug. 2024
 *      Author: Ludo
 */

#include "neom8x_hw.h"

#include "gpio_mapping.h"
#include "lptim.h"
#include "lpuart.h"
#include "nvic_priority.h"

/*** NEOM8X HW functions ***/

/*******************************************************************/
NEOM8X_status_t NEOM8X_HW_init(NEOM8X_HW_configuration_t* configuration) {
	// Local variables.
	NEOM8X_status_t status = NEOM8X_SUCCESS;
	// Check parameter.
	if (configuration == NULL) {
		status = NEOM8X_ERROR_NULL_PARAMETER;
		goto errors;
	}
#if !(defined HW1_0) || !(defined DEBUG)
	LPUART_status_t lpuart_status = LPUART_SUCCESS;
	LPUART_configuration_t lpuart_config;
	// Init LPUART.
	lpuart_config.baud_rate = (configuration -> uart_baud_rate);
	lpuart_config.nvic_priority = NVIC_PRIORITY_GPS_UART;
	lpuart_config.rxne_callback = (LPUART_rx_irq_cb_t) (configuration -> rx_irq_callback);
	lpuart_status = LPUART_init(&GPIO_GPS_LPUART, &lpuart_config);
	LPUART_exit_error(NEOM8X_ERROR_BASE_UART);
#endif
errors:
	return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_HW_de_init(void) {
	// Local variables.
	NEOM8X_status_t status = NEOM8X_SUCCESS;
#if !(defined HW1_0) || !(defined DEBUG)
	LPUART_status_t lpuart_status = LPUART_SUCCESS;
	// Release LPUART.
	lpuart_status = LPUART_de_init(&GPIO_GPS_LPUART);
	LPUART_exit_error(NEOM8X_ERROR_BASE_UART);
errors:
#endif
	return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_HW_send_message(uint8_t* message, uint32_t message_size_bytes) {
	// Local variables.
	NEOM8X_status_t status = NEOM8X_SUCCESS;
	LPUART_status_t lpuart_status = LPUART_SUCCESS;
	// Use LPUART.
	lpuart_status = LPUART_write(message, message_size_bytes);
	LPUART_exit_error(NEOM8X_ERROR_BASE_UART);
errors:
	return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_HW_start_rx(void) {
	// Local variables.
	NEOM8X_status_t status = NEOM8X_SUCCESS;
	// Start LPUART.
	LPUART_enable_rx();
	return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_HW_stop_rx(void) {
	// Local variables.
	NEOM8X_status_t status = NEOM8X_SUCCESS;
	// Stop LPUART.
	LPUART_disable_rx();
	return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_HW_delay_milliseconds(uint32_t delay_ms) {
	// Local variables.
	NEOM8X_status_t status = NEOM8X_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	// Perform delay.
	lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
	LPTIM_exit_error(NEOM8X_ERROR_BASE_DELAY);
errors:
	return status;
}

#ifdef NEOM8X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
NEOM8X_status_t NEOM8X_HW_set_backup_voltage(uint8_t state) {
	// Local variables.
	NEOM8X_status_t status = NEOM8X_SUCCESS;
	// Not implemented.
	UNUSED(state);
	return status;
}
#endif
