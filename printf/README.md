# STM32 "printf" UART Example

This is a minimal bare-metal application which demonstrates how to use the C standard library's `printf` method to send text strings over UART.

It implements the `_write` system call, which lets you call functions like `putchar` and `printf`.

It does the same thing as the "echo" example, but it prints a new line of text for each keystroke. Again, this is a good example of why you shouldn't receive serial data in a program's main loop; it's easy to miss incoming bytes.
