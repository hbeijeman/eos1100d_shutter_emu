CFLAGS=-c -Isrc -Isrc/stm32f0xx_lib -mcpu=cortex-m0 -ffreestanding -mthumb -std=c99 -fno-delayed-branch -fno-defer-pop -fno-exceptions -ffunction-sections -fdata-sections
LDFLAGS=--static -Wl,-static,-relax,-Map=release.map,--cref,--gc-sections -Xlinker --defsym -Xlinker MK_SKIP_FILL=1 -Wl,-Tsrc/linker.ld

SOURCES=src/main.c \
		src/init.c \
		src/stm32f0xx_lib/stm32f0xx_tim.c \
		src/stm32f0xx_lib/stm32f0xx_rcc.c \
		src/stm32f0xx_lib/stm32f0xx_misc.c \
		src/stm32f0xx_lib/stm32f0xx_gpio.c \
		src/stm32f0xx_lib/stm32f0xx_exti.c \
		src/stm32f0xx_lib/stm32f0xx_syscfg.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main.elf

PRINT_INFO_CC=$(info $()  CC     $<)
PRINT_INFO_LD=$(info $()  LD     $@)

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(call PRINT_INFO_LD)
	@$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.c.o:
	$(call PRINT_INFO_CC)
	@$(CC) $(CFLAGS) $< -o $@
	
clean:
	rm -f src/*.o src/stm32f0xx_lib/*.o main.elf *.map
