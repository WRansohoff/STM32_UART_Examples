/*
 * Minimal linker script for an STM32F103C8 / STM32F103CB.
 * These days, most "64KB" C8 chips actually have 128KB of Flash.
 */

/* Label for the program's entry point */
ENTRY(reset_handler)

/* End of RAM/Start of stack */
/* (20KB SRAM) */
_estack = 0x20005000;

/* Set minimum size for stack and dynamic memory. */
/* (The linker will generate an error if there is
 * less than this much RAM leftover.) */
/* (1KB) */
_Min_Leftover_RAM = 0x0400;

MEMORY
{
    FLASH ( rx )      : ORIGIN = 0x08000000, LENGTH = 128K
    RAM ( rxw )       : ORIGIN = 0x20000000, LENGTH = 20K
}

INCLUDE "sections.ld"
