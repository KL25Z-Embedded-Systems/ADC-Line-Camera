

#include "MKL25Z4.h"
#include "LEDDriver.h"

// Initialize the GPIO port for LED control
void LED_init(void) {
    SIM->SCGC5 |= 0x400;        /* enable clock to Port B */
    SIM->SCGC5 |= 0x1000;       /* enable clock to Port D */
    PORTB->PCR[18] = 0x100;     /* make PTB18 pin as GPIO */
    PTB->PDDR |= 0x40000;       /* make PTB18 as output pin */
    PORTB->PCR[19] = 0x100;     /* make PTB19 pin as GPIO */
    PTB->PDDR |= 0x80000;       /* make PTB19 as output pin */
    PORTD->PCR[1] = 0x100;      /* make PTD1 pin as GPIO */
    PTD->PDDR |= 0x02;          /* make PTD1 as output pin */
}

// Change the LED colour according to the voltage
void LED_set(int s) {
	  int maxV=4096;
	int levelStep=4096/5;
	int level1=levelStep;
	int level2=2*levelStep;
	int level3=3*levelStep;
	int level4=4*levelStep;
	
	  if (s<level1)
		   { 
		     PTB->PSOR = 0x80000;    /* turn off green LED */
		     PTD->PSOR = 0x02;       /* turn off blue LED */
				 PTB->PSOR = 0x40000;    /* turn off red LED */
			 }
		else if (s<level2)
			 { 
		     PTB->PCOR = 0x80000;    /* turn ON green LED */
		     PTD->PSOR = 0x02;       /* turn off blue LED */
				 PTB->PSOR = 0x40000;    /* turn off red LED */
			 }
		else if (s<level3)
			 { 
		     PTB->PSOR = 0x80000;    /* turn off green LED */
		     PTD->PCOR = 0x02;       /* turn ON blue LED */
				 PTB->PSOR = 0x40000;    /* turn off red LED */
			 }
		else if (s<level4)
    	 { 
		     PTB->PSOR = 0x80000;    /* turn off green LED */
		     PTD->PSOR = 0x02;       /* turn off blue LED */
				 PTB->PCOR = 0x40000;    /* turn ON red LED */
			 }
		else
		    	 { 
		     PTB->PCOR = 0x80000;    /* turn ON green LED */
		     PTD->PCOR = 0x02;       /* turn ON blue LED */
				 PTB->PCOR = 0x40000;    /* turn ON red LED */
			 }
}
