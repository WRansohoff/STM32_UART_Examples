# STM32 "Echo" UART Example

This is a minimal bare-metal application which demonstrates how to set up a UART peripheral and use it in "polling" mode.

It waits for a new byte of data to be available, then sends that same byte back.

You can see why this is not the best way to process incoming data if you send new bytes very rapidly; while the program is busy running, it might miss incoming data.
