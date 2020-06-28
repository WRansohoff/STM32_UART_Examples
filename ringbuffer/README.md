# STM32 "Ring Buffer" UART Example

This is a minimal bare-metal application which demonstrates how to use hardware interrupts to receive and buffer data over UART. It uses a simple statically-allocated circular buffer to store the data.

It buffers incoming data until a `\r` character is received (i.e., the user pressed "enter" or "return"). Then it prints the entire line of buffered data.

Circular buffers are a solid way to store incoming data until it is ready to process. They don't need to be cleared out between uses, and they won't cause memory corruption if they overflow.
