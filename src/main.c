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

//#define UNITTEST

#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_syscfg.h"

/**
 *          Pin Mappings
 *
 *          NEW:
 *
 *          PA3 -> P1   D0
 *          PA4 -> P9   D10
 *          PA5 -> P10  D5
 *          PA6 -> P8   D4
 *          PA7 -> P12  D6
 *
 *          PB0 <- P5/Trig1
 *          PB2 <- P7/Trig2
 *
 */

typedef enum {
    TIMER,
    TRIG1_DEACTIVATE,
    TRIG1_ACTIVATE,
    TRIG2_DEACTIVATE,
} T_TRIGGER;

typedef enum {
    PHASE_IDLE,
    PHASE_OPEN,
    PHASE_CLOSE_LONG,
    PHASE_CLOSE_SHORT,

    PHASE_MAX,
} T_SHUTTER_PHASE;

#define TB                      (10)                // timebase divisor
#define TRIG2_P7_SAMPLE_WAIT    (72000/TB)  // time to wait before sampling P7
#define TRIG2_P7_SAMPLE_HOLD    (130000/TB)  // time to wait before sampling P7

#define P1_SET      (volatile uint16_t*)&GPIOA->BSRR,   (1<<3)
#define P1_CLR      (volatile uint16_t*)&GPIOA->BRR,    (1<<3)

#define P8_SET      (volatile uint16_t*)&GPIOA->BSRR,   (1<<6)
#define P8_CLR      (volatile uint16_t*)&GPIOA->BRR,    (1<<6)

#define P9_SET      (volatile uint16_t*)&GPIOA->BSRR,   (1<<4)
#define P9_CLR      (volatile uint16_t*)&GPIOA->BRR,    (1<<4)

#define P10_SET      (volatile uint16_t*)&GPIOA->BSRR,   (1<<5)
#define P10_CLR      (volatile uint16_t*)&GPIOA->BRR,    (1<<5)

#define P12_SET      (volatile uint16_t*)&GPIOA->BSRR,   (1<<7)
#define P12_CLR      (volatile uint16_t*)&GPIOA->BRR,    (1<<7)

static inline uint32_t TIMESTAMP(void) { return TIM3->CNT; }

typedef struct {
    uint32_t            trig;
    volatile uint16_t*  io_reg;
    uint16_t            state;
} T_SHUTTER_TIMING;

// Trigger: D8/digital input (110ms) (time-base in 2us (divide by /2))
static const T_SHUTTER_TIMING timing_open[] =
{
     {25850/TB,        P9_CLR},
     {34000/TB,        P1_SET},
     {35100/TB,        P10_SET},
     {39400/TB,        P1_CLR},
     {42000/TB,        P1_SET},
     {62900/TB,        P10_CLR},
     {71400/TB,        P12_SET},
     //{110500/TB,       P1_CLR},         // <<<< Problem with short mode / conflict

     {0, 0, 0},       // END
};

// Trigger: D8/digital input (18ms) (time-base in 2us (divide by /TB))
static const T_SHUTTER_TIMING timing_close_long[] =
{
     {200/TB,           P1_SET},
     {5900/TB,          P1_CLR},
     {7200/TB,          P1_SET},
     {8600/TB,          P1_CLR},
     {34800/TB,         P12_CLR},
     {71100/TB,         P8_SET},
     {107600/TB,        P8_CLR},
     {108000/TB,        P9_SET},

     {0,0, 0},         // END
};

// Trigger: D8/digital input (18ms) (time-base in 2us (divide by /TB))
static const T_SHUTTER_TIMING timing_close_short[] =
{
     {2800/TB,          P1_CLR},    // OK
     {4300/TB,          P1_SET},    // OK
     {5600/TB,          P1_CLR},    // OK
     {31600/TB,         P12_CLR},   // OK
     {67800/TB,         P8_SET},    // OK
     {104000/TB,        P8_CLR},    // OK
     {105000/TB,        P9_SET},    // OK

     {0,0, 0},         // END
};

typedef struct S_SHUTTER {
    uint8_t                 active;
    T_SHUTTER_PHASE         phase;
    T_SHUTTER_TIMING const* timing;
    void (*state)(struct S_SHUTTER* const sh, T_TRIGGER const action);
} T_SHUTTER;

static uint32_t time = 0;

static T_SHUTTER shutter;
static void handler_state_idle(T_SHUTTER* const sh, T_TRIGGER const action);

static void start_timebase(void)
{
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

static void stop_timebase(void)
{
    TIM_Cmd(TIM3, DISABLE);
}

static void timer_base_overflow(T_SHUTTER* const sh)
{
    if (sh->active) {
        __disable_irq();
        FOR_EVER{}
    }
}

static void handler_state_close(T_SHUTTER* const sh, T_TRIGGER const action)
{
    __disable_irq();
    if (action == TRIG1_ACTIVATE) {
        start_timebase();
        sh->active = 1;
    } else if (action == TIMER) {
        if (!sh->timing->trig) {
            stop_timebase();
            sh->phase = PHASE_IDLE;
            sh->state = handler_state_idle;
            sh->timing = NULL;
            sh->active = 0;
        }
    }
    __enable_irq();
}

static void handler_state_open(T_SHUTTER* const sh, T_TRIGGER const action)
{
    __disable_irq();

    if (action == TIMER) {
        // Assume we are in long mode, unless
        // Trig2/P7 signfies we are short mode instead
        if ((TIMESTAMP() >= TRIG2_P7_SAMPLE_WAIT) && (TIMESTAMP() < TRIG2_P7_SAMPLE_HOLD) &&
           (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)==Bit_SET)) {
            start_timebase();               // reset counter
            sh->phase = PHASE_CLOSE_SHORT;
            sh->state = handler_state_close;
            sh->timing = timing_close_short;
        }
    } else if (action == TRIG1_DEACTIVATE) {
        // Reset P1 (in line with falling edge of P5/Trig1
        GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
        sh->active = 0;
        stop_timebase();
        sh->phase = PHASE_CLOSE_LONG;
        sh->state = handler_state_close;
        sh->timing = timing_close_long;

    }

    __enable_irq();
}

static void handler_state_idle(T_SHUTTER* const sh, T_TRIGGER const action)
{
    __disable_irq();

    if (action == TRIG1_ACTIVATE) {
        start_timebase();
        sh->phase   = PHASE_OPEN;
        sh->state   = handler_state_open;
        sh->timing  = timing_open;
        sh->active  = 1;
    }

    __enable_irq();
}

// Called every 100us for state-keeping
static void timer_do(T_SHUTTER* const sh)
{
    if (!sh->active)
        return;

    // Handle signals
    if (sh->timing && sh->timing->trig && (TIMESTAMP() >= sh->timing->trig)) {
        *sh->timing->io_reg = sh->timing->state;
        sh->timing++;

        if (sh->timing && sh->timing->trig == 0) {
            volatile uint32_t x =0 ;
            x++;
        }
    }

    sh->state(sh, TIMER);
}

static void shutter_init(T_SHUTTER* sh)
{
    sh->active = 0;
    sh->state = handler_state_idle;
    sh->timing = NULL;
    sh->phase = 0;
    stop_timebase();
}

static void init_peripherals(void)
{
    TIM_OCInitTypeDef TIM_OCStruct;
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;

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
    TIM_BaseStruct.TIM_Period = 0x32a; // // STM32F072 DISCO/HS48 0x1300;         // 100us
    TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    //// Init time-base reference (2us ticks)
    TIM_TimeBaseStructInit(&TIM_BaseStruct);
    TIM_BaseStruct.TIM_Prescaler            = 0x4e; // STM32F072 DISCO/HS48 0x1d6;    // 10us
    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    TIM_Cmd(TIM3, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // DOUT IRQ
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
    EXTI_InitStruct.EXTI_Line       = EXTI_Line0;
    EXTI_InitStruct.EXTI_LineCmd    = ENABLE;
    EXTI_InitStruct.EXTI_Mode       = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger    = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&EXTI_InitStruct);
}

static void pins_startup(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

    GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET);
}

static void allow_events(void)
{
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void main(void)
{
    init_peripherals();
    pins_startup();

    /* Wait 'till camera is ready, avoid spurious IRQs due to PSU switching */
    while (time < 1500);

    allow_events();

    FOR_EVER {}
}

// Trigger input
__attribute__((__interrupt__)) void irq_trigger(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        uint8_t const set = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
        if (set == Bit_SET) {
            /* Rising Edge */
            shutter.state(&shutter, TRIG1_ACTIVATE);
        } else {
            shutter.state(&shutter, TRIG1_DEACTIVATE);
        }
    }

    EXTI_ClearITPendingBit(EXTI_Line0);
}

// 100us TIMER
__attribute__((__interrupt__)) void irq_tim2(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        timer_do(&shutter);
    }
}

// 10us TIMER
__attribute__((__interrupt__)) void irq_tim3(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        // handle time-base overflow event

        timer_base_overflow(&shutter);
    }
}

__attribute__((__interrupt__)) void  irq_systick(void)
{
    time++;
}

/* EOF */ 
