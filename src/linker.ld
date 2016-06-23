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

OUTPUT_ARCH(arm)

MEMORY {
    VECTORS    (rx)  : ORIGIN = 0x08000000, LENGTH = 1024
    APPROM     (rx)  : ORIGIN = 0x08000400, LENGTH = 32K - 1024
    RAM        (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

/* Stack size to use */
STACK_SIZE = 512;

SECTIONS {
    .isr_vector : {               /* the vector table goes @ 0x08000000     */
        KEEP(*(.isr_vector))                                /* vector table */
        . = ALIGN(4);
    } >VECTORS

    .text : {                                         /* code and constants */
        . = ALIGN(4);
        *(.text)                                   /* .text sections (code) */
        *(.text*)                                 /* .text* sections (code) */
        *(.rodata)           /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)         /* .rodata* sections (constants, strings, etc.) */

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
    } >APPROM

    .data : {
        __data_load = LOADADDR (.data);
        __data_start = .;
        *(.data)                                          /* .data sections */
        *(.data*)                                        /* .data* sections */
        . = ALIGN(4);
        __data_end__ = .;
   } >RAM AT>APPROM 

    .stack : {
        /* Ensure stack size is at least STACK_SIZE */
        __stack_start__ = . ;
        . = . + STACK_SIZE;
        . = ALIGN(4);
        __c_stack_top__ = . ;
        __stack_end__ = . ;        
 
    } >RAM
 
    /* Static, 0-initialized data */   
    .bss (NOLOAD): {
        __bss_start__ = . ;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;                     /* define a global symbol at bss end */
        __bss_end__ = .;
    } >RAM

    PROVIDE ( end = _ebss );
    PROVIDE ( _end = _ebss );
    PROVIDE ( __end__ = _ebss );    

    /* Remove information from the standard libraries */
    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}

