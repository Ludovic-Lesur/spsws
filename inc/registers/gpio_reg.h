/*
 * gpio_reg.h
 *
 *  Created on: 26 apr. 2018
 *      Author: Ludo
 */

#ifndef __GPIO_REG_H__
#define __GPIO_REG_H__

/*** GPIO registers ***/

typedef struct {
	volatile unsigned int MODER;    	// GPIO port mode register.
	volatile unsigned int OTYPER;   	// GPIO port output type register.
	volatile unsigned int OSPEEDR;  	// GPIO port output speed register.
	volatile unsigned int PUPDR;    	// GPIO port pull-up/pull-down register.
	volatile unsigned int IDR;      	// GPIO port input data register.
	volatile unsigned int ODR;      	// GPIO port output data register.
	volatile unsigned int BSRR;    		// GPIO port bit set/reset low register.
	volatile unsigned int LCKR;     	// GPIO port configuration lock register.
	volatile unsigned int AFRL;   		// GPIO alternate function low register.
	volatile unsigned int AFRH;   		// GPIO alternate function high register.
	volatile unsigned int BRR;   		// GPIO port bir reset register.
} GPIO_base_address_t;

/*** GPIO base addresses ***/

#define GPIOA	((GPIO_base_address_t*) ((unsigned int) 0x50000000))
#define GPIOB	((GPIO_base_address_t*) ((unsigned int) 0x50000400))
#define GPIOC	((GPIO_base_address_t*) ((unsigned int) 0x50000800))
#define GPIOD	((GPIO_base_address_t*) ((unsigned int) 0x50000C00))
#define GPIOE	((GPIO_base_address_t*) ((unsigned int) 0x50001000))
#define GPIOH	((GPIO_base_address_t*) ((unsigned int) 0x50001C00))

#endif /* __GPIO_REG_H__ */
