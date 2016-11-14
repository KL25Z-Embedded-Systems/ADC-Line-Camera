


#include <MKL25Z4.H>
#include "UART0_TxRx.h"

/* initialize UART2 to transmit and receive at 9600 Baud */
void UART0_init(void) {
	  SIM->SCGC4 |= 0x0400;   /* enable clock for UART0 */
    SIM->SOPT2 |= 0x04000000;    /* use FLL output for UART Baud rate generator */
    UART0->C2 = 0;          /* turn off UART0 while changing configurations */
    UART0->BDH = 0x00;
    UART0->BDL = 0x17;      /* 115200 Baud */
    UART0->C4 = 0x0F;       /* Over Sampling Ratio 16 */
    UART0->C1 = 0x00;       /* normal 8-bit, no parity  */
	  UART0->C3 = 0x00;       /* no fault interrupt */
   //  UART0->C2 = 0x08;       /* enable transmit */
   //  UART0->C2 = 0x04;       /* enable receive */
    UART0->C2 = 0x0C;       // enable Tx and Rx

    SIM->SCGC5 |= 0x0200;   /* enable clock for PORTA */
    PORTA->PCR[2] = 0x0200; /* make PTA2 UART0_Tx pin */
		PORTA->PCR[1] = 0x0200; /* make PTA1 UART0_Rx pin */
		
}

/* Delay n milliseconds */
/* The CPU core clock is set to MCGFLLCLK at 41.94 MHz in SystemInit(). */

void delayMs(int n) {
    int i;
    int j;
    for(i = 0 ; i < n; i++)
        for (j = 0; j < 7000; j++) {}
}

/*
#define CYCLES_PER_LOOP 3

inline void wait_cycles( uint32_t n ) {
    uint32_t l = n/CYCLES_PER_LOOP;
    // asm volatile( "0:" "SUBS %[count], 1;" "BNE 0b;" :[count]"+r"(l) );
}
*/

void sendHelloWorld() {

    char msg[] = "Hello World!\r\n";
    int i;
  
    for (i = 0; i < 14; i++) {
       while(!(UART0->S1 & 0x80)) {  }   /* wait for transmit buffer empty */
        UART0->D = msg[i]; /* send a char */
    }
}

void sendStr(char *str, int n) {
 int i; 
	     
    for (i = 0; i < n; i++) {
       while(!(UART0->S1 & 0x80)) {  }   /* wait for transmit buffer empty */
        UART0->D = str[i]; /* send a char */
    }
}


void UART0_TxChar(char c) {
	

        while(!(UART0->S1 & 0x80)) {}   /* wait for transmit buffer empty */
        UART0->D = c; /* send the char received */
}

char UART0_RxChar(void) {

	    char c;
	    while(!(UART0->S1 & 0x20)) { }   /* wait for receive buffer full */
      c = UART0->D ; /* read the char received */
			return c;
}
