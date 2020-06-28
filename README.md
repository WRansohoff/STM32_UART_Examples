# STM32 Basic UART Usage

A few simple examples demonstrating some simple ways to use the UART peripherals on STM32F1 and STM32L4 chips.

* echo: Re-transmitting characters over the `TX` line as they are received on the `RX` line.

* printf: Implement the C standard library's `printf` function to "print" over UART.

* receive\_irq: Use interrupts to receive characters more reliably than occasionally polling for them.

* ringbuffer: Use a circular buffer to store a string of characters until they are ready to be processed.

All of the examples include a Makefile, and rely on the `arm-none-eabi` GCC toolchain.
