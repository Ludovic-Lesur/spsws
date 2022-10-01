/*
 * gpio_reg.h
 *
 *  Created on: 26 apr. 2018
 *      Author: Ludo
 */

#ifndef __GPIO_REG_H__
#define __GPIO_REG_H__

#include "types.h"

/*** GPIO registers ***/

typedef struct {
	volatile uint32_t MODER;    	// GPIO port mode register.
	volatile uint32_t OTYPER;   	// GPIO port output type register.
	volatile uint32_t OSPEEDR;  	// GPIO port output speed register.
	volatile uint32_t PUPDR;    	// GPIO port pull-up/pull-down register.
	volatile uint32_t IDR;      	// GPIO port input data register.
	volatile uint32_t ODR;      	// GPIO port output data register.
	volatile uint32_t BSRR;    		// GPIO port bit set/reset low register.
	volatile uint32_t LCKR;     	// GPIO port configuration lock register.
	volatile uint32_t AFRL;   		// GPIO alternate function low register.
	volatile uint32_t AFRH;   		// GPIO alternate function high register.
	volatile uint32_t BRR;   		// GPIO port bir reset register.
} GPIO_base_address_t;

/*** GPIO base addresses ***/

#define GPIOA	((GPIO_base_address_t*) ((uint32_t) 0x50000000))
#define GPIOB	((GPIO_base_address_t*) ((uint32_t) 0x50000400))
#define GPIOC	((GPIO_base_address_t*) ((uint32_t) 0x50000800))
#define GPIOD	((GPIO_base_address_t*) ((uint32_t) 0x50000C00))
#define GPIOE	((GPIO_base_address_t*) ((uint32_t) 0x50001000))
#define GPIOH	((GPIO_base_address_t*) ((uint32_t) 0x50001C00))

#endif /* __GPIO_REG_H__ */
