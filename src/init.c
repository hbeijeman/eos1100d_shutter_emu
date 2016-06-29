/**
The MIT License (MIT)
Copyright (c) 2016 Hendrik Beijeman (hbeyeman@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**/

#include <stdint.h>
#include "stm32f0xx.h"

extern unsigned __c_stack_top__;
extern void main(void);
void irq_reset(void);
void irq_hardfault(void);
void irq_trigger(void);
void irq_tim2(void);
void irq_tim3(void);
void irq_systick(void);

#define CPUFREQ         48000000
#define SYSTICK_RATE    1000

static void clock_init(void)
{
    /* Set HSI48 ON bit */
    RCC->CR2 |= RCC_CR2_HSI48ON;

    /* Wait for HSI48 ready */
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    /* Enable Prefetch Buffer and set Flash Latency */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    /* HCLK,PCLK = SYSCLK, from HSI48, DIV1 */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE_DIV1;

    /* Select HSI48 as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW_HSI48));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI48;

    /* Wait until SYSCLK set from HSI48 */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48) {}
}

static void systick_init(unsigned ticks)
{
  if (ticks > SysTick_LOAD_RELOAD_Msk)            /* Reload value impossible */
      return;

  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;      /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 9);  /* set Priority for Cortex-M0 System Interrupts */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
}

__attribute__((__interrupt__, noreturn)) void irq_reset(void)
{

    extern unsigned __data_start;    /* start of .data in the linker script */
    extern unsigned __data_end__;      /* end of .data in the linker script */
    extern unsigned const __data_load;  /* initialization values for .data  */
    extern unsigned __bss_start__;    /* start of .bss in the linker script */
    extern unsigned __bss_end__;        /* end of .bss in the linker script */
    unsigned const *src;
    unsigned *dst;

    __asm volatile (
       /* Set stack pointer */
         "mov     r0, %[stack]\n\t"         /* Stack ptr */
         "msr     MSP, r0\n\t"              /* update process stack pointer */
         :                                  /* output */
         : [stack] "r" (&__c_stack_top__)   /* Input */
         :                                  /* clobber list */
     );

    IRQ_DISABLE;

    /* copy the data segment initializers from flash to RAM... */
    src = &__data_load;
    for (dst = &__data_start; dst < &__data_end__; ++dst, ++src) {
        *dst = *src;
    }

    /* zero fill the .bss segment... */
    for (dst = &__bss_start__; dst < &__bss_end__; ++dst) {
        *dst = 0;
    }

    clock_init();
    systick_init((CPUFREQ)/(SYSTICK_RATE));

    IRQ_ENABLE;

    main();

    FOR_EVER {}
}

__attribute__((__interrupt__, naked)) void irq_hardfault(void)
{
    FOR_EVER {}
}

__attribute__ ((section(".isr_vector")))
uint32_t* vectors[] = {
    (uint32_t*)&__c_stack_top__,
    (uint32_t*)&irq_reset,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_systick,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_trigger,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_tim2,
    (uint32_t*)&irq_tim3,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,
    (uint32_t*)&irq_hardfault,

};

/* EOF */ 
