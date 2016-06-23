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
#include <stddef.h>

#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_syscfg.h"

#define D0      3
#define D4      4
#define D5      5
#define D6      6
#define D7      7
#define D10     8   //PB8

#define PHASE_IDLE      0
#define PHASE_OPEN      1
#define PHASE_CLOSE     2

#define D0_SET      (volatile uint16_t*)&GPIOB->BSRR,   (1<<D0)
#define D0_CLR      (volatile uint16_t*)&GPIOB->BRR,    (1<<D0)

#define D4_SET      (volatile uint16_t*)&GPIOB->BSRR,   (1<<D4)
#define D4_CLR      (volatile uint16_t*)&GPIOB->BRR,    (1<<D4)

#define D5_SET      (volatile uint16_t*)&GPIOB->BSRR,   (1<<D5)
#define D5_CLR      (volatile uint16_t*)&GPIOB->BRR,    (1<<D5)

#define D6_SET      (volatile uint16_t*)&GPIOB->BSRR,   (1<<D6)
#define D6_CLR      (volatile uint16_t*)&GPIOB->BRR,    (1<<D6)

#define D7_SET      (volatile uint16_t*)&GPIOB->BSRR,   (1<<D7)
#define D7_CLR      (volatile uint16_t*)&GPIOB->BRR,    (1<<D7)

#define D10_SET      (volatile uint16_t*)&GPIOB->BSRR,   (1<<D10)
#define D10_CLR      (volatile uint16_t*)&GPIOB->BRR,    (1<<D10)

#define TB          (10)

typedef struct {
    uint32_t            time;
    volatile uint16_t*  io_reg;
    uint16_t            state;
} T_SHUTTER_STATE;

// Trigger: D8/digital input (110ms) (time-base in 2us (divide by /2))
static const T_SHUTTER_STATE state_shutter_open[] =
{
     {200/TB,          D7_SET},     // OK, 220u
     {25850/TB,       D10_CLR},
     {34000/TB,        D0_SET},     // OK, 34000u
     {34400/TB,        D5_SET},     // OK,
     {38800/TB,        D0_CLR},     // OK,
     {41200/TB,        D0_SET},     // OK,
     {60200/TB,        D5_CLR},     // OK,
     {68800/TB,        D6_SET},     // OK
     {110500/TB,       D0_CLR},     // OK

     {0, 0, 0},       // END
};

// Trigger: D8/digital input (18ms) (time-base in 2us (divide by /TB))
static const T_SHUTTER_STATE state_shutter_close[] =
{
     {200/TB,           D0_SET},    // OK
     {2900/TB,          D7_CLR},    // OK
     {5900/TB,          D0_CLR},    // OK
     {7200/TB,          D0_SET},    // OK
     {8700/TB,          D0_CLR},    // OK
     {34300/TB,         D6_CLR},    // OK
     {70310/TB,         D4_SET},    // OK
     {106000/TB,        D4_CLR},    // OK
     {108000/TB,       D10_SET},    // OK

     {0,0, 0},         // END
};

typedef struct {
    uint8_t                 active;
    uint8_t                 phase;
    T_SHUTTER_STATE const*  state;
} T_SHUTTER;

static T_SHUTTER shutter;

static void start_timebase(void)
{
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

static void stop_timebase(void)
{
    TIM_Cmd(TIM3, DISABLE);
}

static void shutter_init(T_SHUTTER* sh)
{
    sh->active = 0;
    sh->phase = PHASE_IDLE;
    sh->state = NULL;
    stop_timebase();
}

static void timer_base_overflow(T_SHUTTER* const sh)
{
    if (sh->active) {
        __disable_irq();
        FOR_EVER{}
    }
}

static void cam_input_do(T_SHUTTER* const sh)
{
    sh->phase++;
    start_timebase();

    switch(sh->phase)
    {
        case PHASE_OPEN:
            sh->state = state_shutter_open;
            break;
        case PHASE_CLOSE:
            sh->state = state_shutter_close;
            break;
        default:
            sh->state = NULL;
            return;
    }
    sh->active = 1;
}

// Called every 100us for state-keeping
static void timer_do(T_SHUTTER* const sh)
{
    if ((!sh->active) || (sh->phase > PHASE_CLOSE))
        return;

    uint32_t const now = TIM_GetCounter(TIM3);

    if (sh->state && (now >= sh->state->time)) {
        *sh->state->io_reg = sh->state->state;
        sh->state++;
    }

    if (!sh->state->time) {
        sh->active = 0;
        if (sh->phase >= PHASE_CLOSE)
            shutter_init(sh);
    }
}

/**
 *      PB3 -> D0
 *      PB4 -> D4
 *      PB5 -> D5
 *      PB6 -> D7
 *      PB7 -> D6
 *
 *      PB8 <- IRQ/CAM/START
 *
 */

static void init_peripherals(void)
{
    TIM_OCInitTypeDef TIM_OCStruct;
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;

    shutter.state = state_shutter_open;
    shutter_init(&shutter);

    // DCLK, MCLK Clocks / AF pins
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Enable TIM2 IRQ
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    //// Init timer_do scheduler timer w/IRQ
    TIM_TimeBaseStructInit(&TIM_BaseStruct);
    TIM_BaseStruct.TIM_Period = 819;
    TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    //// Init time-base reference (2us ticks)
    TIM_TimeBaseStructInit(&TIM_BaseStruct);
    TIM_BaseStruct.TIM_Prescaler            = 79;
    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    TIM_Cmd(TIM3, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_12 | GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // DOUT IRQ
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
    EXTI_InitStruct.EXTI_Line       = EXTI_Line2;
    EXTI_InitStruct.EXTI_LineCmd    = ENABLE;
    EXTI_InitStruct.EXTI_Mode       = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger    = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET); // Set D10
}

void main(void)
{
    init_peripherals();
    FOR_EVER {}
}

// Trigger input
__attribute__((__interrupt__)) void irq_trigger(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        cam_input_do(&shutter);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

// 100us TIMER
__attribute__((__interrupt__)) void irq_tim2(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        timer_do(&shutter);
    }
}

// 100us TIMER
__attribute__((__interrupt__)) void irq_tim3(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        // handle time-base overflow event

        timer_base_overflow(&shutter);
    }
}

/* EOF */ 
