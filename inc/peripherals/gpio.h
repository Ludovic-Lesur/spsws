/*
 * gpio.h
 *
 *  Created on: 16 dec. 2018
 *      Author: Ludo
 */

#ifndef GPIO_H
#define GPIO_H

#include "gpio_reg.h"

/*** GPIO structures ***/

// GPIO structure.
typedef struct {
	GPIO_base_address_t* gpio_port_address; // GPIOA to GPIOC.
	unsigned char gpio_port_index; // 0 for GPIOA, 1 for GPIOB, etc.
	unsigned char gpio_num; // 0 to 15.
	unsigned char gpio_af_num; // Alternate function number if used.
} GPIO_pin_t;

typedef enum {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALTERNATE_FUNCTION,
	GPIO_MODE_ANALOG
} GPIO_mode_t;

typedef enum {
	GPIO_TYPE_PUSH_PULL,
	GPIO_TYPE_OPEN_DRAIN
} GPIO_output_type_t;

typedef enum {
	GPIO_SPEED_LOW,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_HIGH,
	GPIO_SPEED_VERY_HIGH,
} GPIO_output_speed_t;

typedef enum {
	GPIO_PULL_NONE,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN
} GPIO_pull_resistor_t;

/*** GPIO functions ***/

void GPIO_init(void);
void GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t output_speed, GPIO_pull_resistor_t pull_resistor);
void GPIO_write(const GPIO_pin_t* gpio, unsigned char state);
unsigned char GPIO_read(const GPIO_pin_t* gpio);
void GPIO_toggle(const GPIO_pin_t* gpio);

#endif /* GPIO_H */
