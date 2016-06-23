# EOS1100D Shutter Emulation

This contains the code emulate the shutter of the Canon EOS1100D DSLR.
Code is targeted to the STM32F042 processor, but any other will do as well.

## LICENSE

All source-code is provided "AS-IS", under MIT license. 

## Pin mapping

Push-pull outputs:
PA3 -> P1
PA4 -> P9
PA5 -> P10
PA6 -> P8
PA7 -> P12

Trigger input (w/internal pull-down):
PB0 <- P5/Trigger

## Peripheral use

The STM32F042 is used in "crystal-less" mode. The internal HSI48 is
configured to provide SYSCLK, HCLK, PCLK at 48MHz.
Peripherals are configured as follows:

#### TIM3
Provides the timebase for internal clock. It is set to 10us ticks.
It is started from 0 whenever the trigger is activated. 
If an overflow occurs, an error condition is presented, because
the shutter sequence should always end within this period of time.

#### TIM2
Serves as bare-bones 100us scheduler to serve monitor tasks.

#### EXTI
PB0 is used as trigger input, EXTI is used to provide an interrupt
on rising edge to signal a new shutter sequence.


## Usage

Checkout this code, and install a suitable GCC toolchain for ARM.
```
# CC=<my toolchain>-gcc make
CC     src/main.c
CC     src/init.c
CC     src/stm32f0xx_lib/stm32f0xx_tim.c
CC     src/stm32f0xx_lib/stm32f0xx_rcc.c
CC     src/stm32f0xx_lib/stm32f0xx_misc.c
CC     src/stm32f0xx_lib/stm32f0xx_gpio.c
CC     src/stm32f0xx_lib/stm32f0xx_exti.c
CC     src/stm32f0xx_lib/stm32f0xx_syscfg.c
LD     main.elf
```

Next, the flash file (main.elf) can be flashed into the target.
For reference, I included the Lauterbach-script I use. 
If you don't have a Lauterbach, then google for one of the many
different procedures for tool flashers. 

Note: this microcontroller uses SWD interface for flashing:
PA13 <-> SWDIO
PA14 <-  SWCLK
Pin7 <-  NRST

## Links

For more project information, see:
[General information about the shutter signals](http://phym.nl/wp/?p=367)
[Specific implementation details](http://phym.nl/wp/?p=427)

