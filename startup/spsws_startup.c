/*
 * spsws_startup.c
 *
 *  Created on: 6 mar. 2023
 *      Author: ARM
 */

#include "types.h"

/*----------------------------------------------------------------------------
 Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
#ifdef __STARTUP_COPY_MULTIPLE
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
#endif
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/*----------------------------------------------------------------------------
 Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void (*pFunc)(void);

/*----------------------------------------------------------------------------
 External References
 *----------------------------------------------------------------------------*/
#ifndef __START
extern void  _start(void)                   __attribute__((noreturn)); /* PreeMain (C library entry point) */
#else
extern int __START(void)                    __attribute__((noreturn)); /* main entry point */
#endif

#ifndef __NO_SYSTEM_INIT
extern void SystemInit (void); /* CMSIS System Initialization */
#endif

/*----------------------------------------------------------------------------
 Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void); /* Default empty handler */
void Reset_Handler(void); /* Reset Handler */

/*----------------------------------------------------------------------------
 User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
#define __STACK_SIZE    0x00000400
#endif
static uint8_t stack[__STACK_SIZE]          __attribute__ ((aligned(8), used, section(".stack")));

#ifndef __HEAP_SIZE
#define __HEAP_SIZE     0x00000C00
#endif
#if __HEAP_SIZE > 0
static uint8_t heap[__HEAP_SIZE]            __attribute__ ((aligned(8), used, section(".heap")));
#endif

/*----------------------------------------------------------------------------
 Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Cortex-M0+ Processor Exceptions */
void NMI_Handler(void)                      __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void)                __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void)                      __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void)                   __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void)                  __attribute__ ((weak, alias("Default_Handler")));

/* ARMCM0plus Specific Interrupts */
void WWDG_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)                   __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler(void)                   __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)                 __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)                   __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_1_IRQHandler(void)               __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_3_IRQHandler(void)               __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_15_IRQHandler(void)              __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void)         __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel2_3_IRQHandler(void)       __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel4_5_6_7_IRQHandler(void)   __attribute__ ((weak, alias("Default_Handler")));
void ADC1_COMP_IRQHandler(void)             __attribute__ ((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler(void)                __attribute__ ((weak, alias("Default_Handler")));
void USART4_5_IRQHandler(void)              __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void TIM6_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void TIM21_IRQHandler(void)                 __attribute__ ((weak, alias("Default_Handler")));
void I2C3_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void TIM22_IRQHandler(void)                 __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void I2C2_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)                __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)                __attribute__ ((weak, alias("Default_Handler")));
void LPUART1_IRQHandler(void)               __attribute__ ((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
 Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__ ((section(".vectors"))) = {
    /* Cortex-M0+ Exceptions Handler */
    (pFunc) ((uint32_t) &__StackTop),   /*      Initial Stack Pointer     */
    Reset_Handler,                      /*      Reset Handler             */
    NMI_Handler,                        /*      NMI Handler               */
    HardFault_Handler,                  /*      Hard Fault Handler        */
    0,                                  /*      Reserved                  */
    0,                                  /*      Reserved                  */
    0,                                  /*      Reserved                  */
    0,                                  /*      Reserved                  */
    0,                                  /*      Reserved                  */
    0,                                  /*      Reserved                  */
    0,                                  /*      Reserved                  */
    SVC_Handler,                        /*      SVCall Handler            */
    0,                                  /*      Reserved                  */
    0,                                  /*      Reserved                  */
    PendSV_Handler,                     /*      PendSV Handler            */
    SysTick_Handler,                    /*      SysTick Handler           */

    /* External interrupts */
    WWDG_IRQHandler,                    /* Window WatchDog              */
    PVD_IRQHandler,                     /* PVD through EXTI Line detection */
    RTC_IRQHandler,                     /* RTC through the EXTI line     */
    FLASH_IRQHandler,                   /* FLASH                        */
    RCC_IRQHandler,                     /* RCC                          */
    EXTI0_1_IRQHandler,                 /* EXTI Line 0 and 1            */
    EXTI2_3_IRQHandler,                 /* EXTI Line 2 and 3            */
    EXTI4_15_IRQHandler,                /* EXTI Line 4 to 15            */
    0,                                  /* Reserved                     */
    DMA1_Channel1_IRQHandler,           /* DMA1 Channel 1               */
    DMA1_Channel2_3_IRQHandler,         /* DMA1 Channel 2 and Channel 3 */
    DMA1_Channel4_5_6_7_IRQHandler,     /* DMA1 Channel 4, Channel 5, Channel 6 and Channel 7*/
    ADC1_COMP_IRQHandler,               /* ADC1, COMP1 and COMP2        */
    LPTIM1_IRQHandler,                  /* LPTIM1                       */
    USART4_5_IRQHandler,                /* USART4 and USART 5           */
    TIM2_IRQHandler,                    /* TIM2                         */
    TIM3_IRQHandler,                    /* TIM3                         */
    TIM6_IRQHandler,                    /* TIM6 and DAC                 */
    TIM7_IRQHandler,                    /* TIM7                         */
    0,                                  /* Reserved                     */
    TIM21_IRQHandler,                   /* TIM21                        */
    I2C3_IRQHandler,                    /* I2C3                         */
    TIM22_IRQHandler,                   /* TIM22                        */
    I2C1_IRQHandler,                    /* I2C1                         */
    I2C2_IRQHandler,                    /* I2C2                         */
    SPI1_IRQHandler,                    /* SPI1                         */
    SPI2_IRQHandler,                    /* SPI2                         */
    USART1_IRQHandler,                  /* USART1                       */
    USART2_IRQHandler,                  /* USART2                       */
    LPUART1_IRQHandler,                 /* LPUART1                      */
    0,                                  /* Reserved                     */
    0                                   /* Reserved                     */
};

/*----------------------------------------------------------------------------
 Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void) {
    uint32_t* pSrc, * pDest;
    uint32_t* pTable __attribute__((unused));

    /*  Firstly it copies data from read only memory to RAM. There are two schemes
     *  to copy. One can copy more than one sections. Another can only copy
     *  one section.  The former scheme needs more instructions and read-only
     *  data to implement than the latter.
     *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
    /*  Multiple sections scheme.
    *
    *  Between symbol address __copy_table_start__ and __copy_table_end__,
    *  there are array of triplets, each of which specify:
    *    offset 0: LMA of start of a section to copy from
    *    offset 4: VMA of start of a section to copy to
    *    offset 8: size of the section to copy. Must be multiply of 4
    *
    *  All addresses must be aligned to 4 bytes boundary.
    */
    pTable = &__copy_table_start__;

    for (; pTable < &__copy_table_end__; pTable = pTable + 3) {
        pSrc  = (uint32_t*)*(pTable + 0);
        pDest = (uint32_t*)*(pTable + 1);
        for (; pDest < (uint32_t*)(*(pTable + 1) + *(pTable + 2)) ; ) {
            *pDest++ = *pSrc++;
        }
    }
#else
    /*  Single section scheme.
     *
     *  The ranges of copy from/to are specified by following symbols
     *    __etext: LMA of start of the section to copy from. Usually end of text
     *    __data_start__: VMA of start of the section to copy to
     *    __data_end__: VMA of end of the section to copy to
     *
     *  All addresses must be aligned to 4 bytes boundary.
     */
    pSrc = &__etext;
    pDest = &__data_start__;

    for (; pDest < &__data_end__;) {
        *pDest++ = *pSrc++;
    }
#endif /*__STARTUP_COPY_MULTIPLE */

    /*  This part of work usually is done in C library startup code. Otherwise,
     *  define this macro to enable it in this startup.
     *
     *  There are two schemes too. One can clear multiple BSS sections. Another
     *  can only clear one section. The former is more size expensive than the
     *  latter.
     *
     *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
     *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
     */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
    /*  Multiple sections scheme.
    *
    *  Between symbol address __copy_table_start__ and __copy_table_end__,
    *  there are array of tuples specifying:
    *    offset 0: Start of a BSS section
    *    offset 4: Size of this BSS section. Must be multiply of 4
    */
    pTable = &__zero_table_start__;

    for (; pTable < &__zero_table_end__; pTable = pTable + 2) {
        pDest = (uint32_t*)*(pTable + 0);
        for (; pDest < (uint32_t*)(*(pTable + 0) + *(pTable + 1)) ; ) {
            *pDest++ = 0;
        }
    }
#elif defined (__STARTUP_CLEAR_BSS)
    /*  Single BSS section scheme.
     *
     *  The BSS section is specified by following symbols
     *    __bss_start__: start of the BSS section.
     *    __bss_end__: end of the BSS section.
     *
     *  Both addresses must be aligned to 4 bytes boundary.
     */
    pDest = &__bss_start__;

    for (; pDest < &__bss_end__;) {
        *pDest++ = 0UL;
    }
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
    SystemInit();
#endif

#ifndef __START
#define __START _start
#endif
    __START();
}

/*----------------------------------------------------------------------------
 Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) {
    // Enter sleep mode.
    while (1) {
        __asm volatile ("wfi");
    }
}
