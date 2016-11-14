#ifndef UART0_TXRX
  #define UART0_TXRX
	

#include "MKL25Z4.h"

/* initialize UART2 to transmit and receive at 9600 Baud */
void UART0_init(void);

void UART0_TxChar(char c);
	
char UART0_RxChar(void);

void sendHelloWorld(void);
void sendStr(char *str, int n);
	
/* Delay n milliseconds */
/* The CPU core clock is set to MCGFLLCLK at 41.94 MHz in SystemInit(). */

void delayMs(int n);

#endif
