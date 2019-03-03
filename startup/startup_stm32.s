/**
  ******************************************************************************
  * @file      startup_stm32.s dedicated to STM32L031G6Ux device
  * @author    Ac6
  * @version   V1.0.0
  * @date      2018-04-24
  ******************************************************************************
  */

.syntax unified
.cpu cortex-m0plus
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  
/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The STM32L031G6Ux vector table.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
   .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word	0
  .word	0
  .word	0
  .word	0
  .word	0
  .word	0
  .word	0
  .word	SVC_Handler
  .word	0
  .word	0
  .word	PendSV_Handler
  .word	SysTick_Handler
  .word	0 // Watchdog
  .word	0 // PVD
  .word	RTC_IRQHandler // RTC
  .word	0 // Flash
  .word	0 // RCC
  .word	EXTI_0_1_IRQHandler // EXTI lines [1:0] interrupts
  .word	EXTI_2_3_IRQHandler // EXTI lines [3:2] interrupts
  .word	EXTI_4_15_IRQHandler // EXTI lines [15:4] interrupts
  .word	0 // Reserved
  .word	0 // DMA1 channel 1
  .word	0 // DMA1 channels 2-3
  .word	0 // DMA1 channels 4-7
  .word	0 // ADC, COMP1 and COMP2.
  .word	0 // LPTIM1 through EXTI29
  .word	0 // USART4 and USART5
  .word	TIM2_IRQHandler // TIM2
  .word	0 // TIM3
  .word	0 // TIM6 and DAC
  .word	0 // TIM7 and DAC
  .word	0 // Reserved
  .word	TIM21_IRQHandler // TIM21
  .word	0 // I2C3
  .word	0 // TIM22
  .word	0 // I2C1
  .word	0 // I2C2
  .word 0 // SPI1
  .word	0 // SPI2
  .word	USART1_IRQHandler // USART1
  .word	USART2_IRQHandler // USART2
  .word	LPUART1_IRQHandler // AES, RNG and LPUART1.

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

  	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler
	
	.weak	RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler

	.weak	EXTI_0_1_IRQHandler
	.thumb_set EXTI_0_1_IRQHandler,Default_Handler

	.weak	EXTI_2_3_IRQHandler
	.thumb_set EXTI_2_3_IRQHandler,Default_Handler

	.weak	EXTI_4_15_IRQHandler
	.thumb_set EXTI_4_15_IRQHandler,Default_Handler
	
	.weak	LPUART1_IRQHandler
	.thumb_set LPUART1_IRQHandler,Default_Handler
	
	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler

	.weak	TIM21_IRQHandler
	.thumb_set TIM21_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	SystemInit

/************************ (C) COPYRIGHT Ac6 *****END OF FILE****/
