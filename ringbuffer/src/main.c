// Standard library includes.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Device header file.
#ifdef VVC_F1
  #include "stm32f1xx.h"
#elif VVC_L4
  #include "stm32l4xx.h"
#endif

#include "ringbuf.h"

#define RINGBUF_SIZE ( 128 )
volatile char rb_buf[ RINGBUF_SIZE + 1 ];
ringbuf rb = {
  len: RINGBUF_SIZE,
  buf: rb_buf,
  pos: 0,
  ext: 0
};
volatile int newline = 0;

uint32_t SystemCoreClock = 0;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

// Override the 'write' clib method to implement 'printf' over UART.
int _write( int handle, char* data, int size ) {
  int count = size;
  while ( count-- ) {
    #ifdef VVC_F1
      while ( !( USART2->SR & USART_SR_TXE ) ) {};
      USART2->DR = *data++;
    #elif VVC_L4
      while ( !( USART2->ISR & USART_ISR_TXE ) ) {};
      USART2->TDR = *data++;
    #endif
  }
  return size;
}

/**
 * Main program.
 */
int main( void ) {
  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Set the core system clock speed.
  #ifdef VVC_F1
    // Default clock source is the 8MHz internal oscillator.
    SystemCoreClock = 8000000;
  #elif VVC_L4
    // Default clock source is the "multi-speed" internal oscillator.
    // Switch to the 16MHz HSI oscillator.
    RCC->CR |=  ( RCC_CR_HSION );
    while ( !( RCC->CR & RCC_CR_HSIRDY ) ) {};
    RCC->CFGR &= ~( RCC_CFGR_SW );
    RCC->CFGR |=  ( RCC_CFGR_SW_HSI );
    while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI ) {};
    SystemCoreClock = 16000000;
  #endif

  // Enable peripheral clocks and set up GPIO pins.
  #ifdef VVC_F1
    // Enable peripheral clocks: GPIOA, USART2.
    RCC->APB1ENR  |=  ( RCC_APB1ENR_USART2EN );
    RCC->APB2ENR  |=  ( RCC_APB2ENR_IOPAEN );
    // Configure pins A2, A3 for USART2.
    GPIOA->CRL    &= ~( GPIO_CRL_MODE2 |
                        GPIO_CRL_CNF2 |
                        GPIO_CRL_MODE3 |
                        GPIO_CRL_CNF3 );
    GPIOA->CRL    |= ( ( 0x1 << GPIO_CRL_MODE2_Pos ) |
                       ( 0x2 << GPIO_CRL_CNF2_Pos ) |
                       ( 0x0 << GPIO_CRL_MODE3_Pos ) |
                       ( 0x1 << GPIO_CRL_CNF3_Pos ) );
  #elif VVC_L4
    // Enable peripheral clocks: GPIOA, USART2.
    RCC->APB1ENR1 |= ( RCC_APB1ENR1_USART2EN );
    RCC->AHB2ENR  |= ( RCC_AHB2ENR_GPIOAEN );
    // Configure pins A2, A15 for USART2 (AF7).
    GPIOA->MODER    &= ~( ( 0x3 << ( 2 * 2 ) ) |
                          ( 0x3 << ( 15 * 2 ) ) );
    GPIOA->MODER    |=  ( ( 0x2 << ( 2 * 2 ) ) |
                          ( 0x2 << ( 15 * 2 ) ) );
    GPIOA->OTYPER   &= ~( ( 0x1 << 2 ) |
                          ( 0x1 << 15 ) );
    GPIOA->OSPEEDR  &= ~( ( 0x3 << ( 2 * 2 ) ) |
                          ( 0x3 << ( 15 * 2 ) ) );
    GPIOA->OSPEEDR  |=  ( ( 0x2 << ( 2 * 2 ) ) |
                          ( 0x2 << ( 15 * 2 ) ) );
    GPIOA->AFR[ 0 ] &= ~( ( 0xF << ( 2 * 4 ) ) );
    GPIOA->AFR[ 0 ] |=  ( ( 0x7 << ( 2 * 4 ) ) );
    GPIOA->AFR[ 1 ] &= ~( ( 0xF << ( ( 15 - 8 ) * 4 ) ) );
    GPIOA->AFR[ 1 ] |=  ( ( 0x3 << ( ( 15 - 8 ) * 4 ) ) );
  #endif

  // Setup the NVIC to enable interrupts.
  // Use 4 bits for 'priority' and 0 bits for 'subpriority'.
  NVIC_SetPriorityGrouping( 0 );
  // UART receive interrupts should be high priority.
  uint32_t uart_pri_encoding = NVIC_EncodePriority( 0, 1, 0 );
  NVIC_SetPriority( USART2_IRQn, uart_pri_encoding );
  NVIC_EnableIRQ( USART2_IRQn );

  // Set the baud rate to 9600.
  uint16_t uartdiv = SystemCoreClock / 9600;
  #ifdef VVC_F1
    USART2->BRR = ( ( ( uartdiv / 16 ) << USART_BRR_DIV_Mantissa_Pos ) |
                    ( ( uartdiv % 16 ) << USART_BRR_DIV_Fraction_Pos ) );
  #elif VVC_L4
    USART2->BRR = uartdiv;
  #endif

  // Enable the USART peripheral with RX and RX timeout interrupts.
  USART2->CR1 |= ( USART_CR1_RE |
                   USART_CR1_TE |
                   USART_CR1_UE |
                   USART_CR1_RXNEIE );

  // Main loop: wait for a new byte, then echo it back.
  while ( 1 ) {
    while ( newline == 0 ) {
      __WFI();
    }
    while ( rb.pos != rb.ext ) {
      putchar( ringbuf_read( &rb ) );
    }
    printf( "\n" );
    newline = 0;
  }
}

// USART2 interrupt handler
void USART2_IRQn_handler( void ) {
  #ifdef VVC_F1
    // 'Receive register not empty' interrupt.
    if ( USART2->SR & USART_SR_RXNE ) {
      // Copy new data into the buffer.
      char c = USART2->DR;
      ringbuf_write( rb, c );
      if ( c == '\r' ) { newline = 1; }
    }
  #elif VVC_L4
    // 'Receive register not empty' interrupt.
    if ( USART2->ISR & USART_ISR_RXNE ) {
      // Copy new data into the buffer.
      char c = USART2->RDR;
      ringbuf_write( rb, c );
      if ( c == '\r' ) { newline = 1; }
    }
  #endif
}
