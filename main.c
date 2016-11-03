//***************************************************************************
// * 
// * Created on:  01 11 2016
// * Author:      Xuweu Dai  (xuewu.dai at northumbria.ac.uk)
// *
// * File:        Example Programme ADC at KL25Z
// *
// * This program converts the analog input from ADC channel 13 (PTB3) 
// * using software trigger continuously.
// * PTB3 is connect to the potentiometer POT1 at the TFC-Shield board.
// * When sdjusting POT1, the voltage of PTB3 varies from 0 volts to 3.3 volts.
// * The value of this voltage is used to control the tri-color LEDs. 
// * When the potentiometer is turned, the LEDs colour changes.
// * At the same time, the result of the each A/D conversionn is also sent to 
// * the PC and through the UART0 and displayed at the PC's terminal windows.
// *
// * Copyright:   (C) 2016 Northumbria University, Newcastle upon Tyne, UK.
// *
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU Lesser General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// 
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU Lesser General Public License for more details.
// *
// %              _
// %  \/\ /\ /   /  * _  '
// % _/\ \/\/ __/__.'(_|_|_
// **************************************************************************/


#include "MKL25Z4.h"
#include "LEDDriver.h"
#include "UART0_TxRx.h"

void ADC0_init(void);
short int readADC(short ChID);


int main (void)
{
    short int adcPOT1;  // POT one
    
	  char buf [100];   // UART buffer 
    int n;            // number of characters in buf to be sent
	
    LED_init();                     /* Configure LEDs */
    ADC0_init();                    /* Configure ADC0 */
    UART0_init();  // Initialized UART0, 57600 baud
	
		sendHelloWorld();
	  while (1) {
				adcPOT1=readADC(13);
			  LED_set(adcPOT1); /* display the voltage range on LED */
			  n = sprintf(buf, "%d\r\n", adcPOT1); // convert integer value into ASCII
			  sendStr(buf, n);
    }
}



void ADC0_init(void)
{
	
    // initialization for PORTB PTB3 (ADC0_SE13) 
    SIM->SCGC5 |= 0x0400|0x2000;  // enable clock to PORTB 
	  PORTB->PCR[3] = 0;         // PTB3.MUX[10 9 8]=000, analog input
    
	  SIM->SCGC6 |= 0x08000000;   // enable clock to ADC0 
    ADC0->SC2 &= ~0x40;         // software trigger
  	/* clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 =(0x1<<6 | 0x1<<4 |0x1<<2); //0b01010100; 
	//  ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
}

short int readADC(short ChID) 
{
	short int result;     	
	
	ADC0->SC1[0] = ChID; //software triger conversion on channel 13, SE13
	while(!(ADC0->SC1[0] & 0x80)) { } /* wait for conversion complete */
	result = ADC0->R[0];        /* read conversion result and clear COCO flag */
	return result;
}
