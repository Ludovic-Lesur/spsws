/*
 * terminal_hw.c
 *
 *  Created on: 29 sep. 2024
 *      Author: Ludo
 */

#include "terminal_hw.h"

#ifndef EMBEDDED_UTILS_DISABLE_FLAGS_FILE
#include "embedded_utils_flags.h"
#endif
#include "error.h"
#include "gpio_mapping.h"
#include "nvic_priority.h"
#include "types.h"
#include "usart.h"

#if (!(defined EMBEDDED_UTILS_TERMINAL_DRIVER_DISABLE) && (EMBEDDED_UTILS_TERMINAL_INSTANCES_NUMBER > 0))

#ifdef HW1_0
#define TERMINAL_HW_USART_INSTANCE      USART_INSTANCE_USART2
#endif
#ifdef HW2_0
#define TERMINAL_HW_USART_INSTANCE      USART_INSTANCE_USART1
#endif

/*** TERMINAL HW functions ***/

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_init(uint8_t instance, uint32_t baud_rate, TERMINAL_rx_irq_cb_t rx_irq_callback) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    USART_configuration_t usart_config;
    // Unused parameter.
    UNUSED(instance);
    // Init USART.
    usart_config.baud_rate = baud_rate;
    usart_config.nvic_priority = NVIC_PRIORITY_AT;
    usart_config.rxne_callback = rx_irq_callback;
    usart_status = USART_init(TERMINAL_HW_USART_INSTANCE, &GPIO_AT_USART, &usart_config);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
    // Start reception.
    usart_status = USART_enable_rx(TERMINAL_HW_USART_INSTANCE);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
errors:
    return status;
}

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_de_init(uint8_t instance) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Unused parameter.
    UNUSED(instance);
    // Release USART.
    usart_status = USART_de_init(TERMINAL_HW_USART_INSTANCE, &GPIO_AT_USART);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
errors:
    return status;
}

/*******************************************************************/
TERMINAL_status_t TERMINAL_HW_write(uint8_t instance, uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    TERMINAL_status_t status = TERMINAL_SUCCESS;
    USART_status_t usart_status = USART_SUCCESS;
    // Unused parameter.
    UNUSED(instance);
    // Write data over USART.
    usart_status = USART_write(TERMINAL_HW_USART_INSTANCE, data, data_size_bytes);
    USART_exit_error(TERMINAL_ERROR_BASE_HW_INTERFACE);
errors:
    return status;
}

#endif /* EMBEDDED_UTILS_TERMINAL_DRIVER_DISABLE */
