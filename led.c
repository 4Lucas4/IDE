/*
 * File:        led.c
 * Purpose:     Provide LED control
 *
 * Notes:		
 *
 */

// NOTE: The LEDs and the Switches on the K64 are all active LOW

#include "MK64F12.h"                    // Device header
//void delay(void)
//{
//	volatile long j = 0;
//	for ( j=0; j < 2000000; j++)
//		;
//}

void LED_Off(void)
{
	// set the bits to ONE to turn off LEDs
	// use PSOR to set a bit
    GPIOB_PSOR = (1 << 22); // R
    GPIOE_PSOR = (1 << 26); // G
    GPIOB_PSOR = (1 << 21); // B

}

void LED_Init(void)
{
	// Enable clocks on Ports B and E for LED timing
	// We use PortB for RED and BLUE LED
	// We use PortE for GREEN LED
	// 12.2.12 System Clock Gating Control Register 5
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
    
	// Port B is Bit 10
	// Port E is Bit 13
				   // 0x0400 (Bit 10)                 0x2000 (Bit 13)
	
    // Red LED is   PTB22
    // Green LED is PTE26
    // Blue LED is  PTB21
	// Configure the Signal Multiplexer for GPIO
    PORTB_PCR22 = PORT_PCR_MUX(1);
    PORTE_PCR26 = PORT_PCR_MUX(1);
    PORTB_PCR21 = PORT_PCR_MUX(1);
    
 	// Pin Control Register n  

	// Switch the GPIO pins to output mode
	// GPIOB_PDDR is the direction control for Port B
	// GPIOE_PDDR is the direction control for Port E
	// set it to ONE at BIT21, 22 on Port B for an output
	// set it to ONE at Bit26 on Port E for an output	 
    GPIOB_PDDR = (1 << 22)|(1 << 21); // set Red Blue LED to output
    GPIOE_PDDR = (1 << 26); // set Green LED to output

	// Turn off the LEDs
    LED_Off();
}

void LED_On (unsigned char color)
{
	// set the appropriate color
	// you need to drive the appropriate pin OFF to turn on the LED
	// case statement !!!
		switch(color){
			case 'R' :
				GPIOB_PCOR = (1 << 22);
			  GPIOB_PSOR = (1 << 26);
				GPIOB_PSOR = (1 << 21);
				break;
			case 'G' :
				GPIOB_PSOR = (1 << 22);
				GPIOE_PCOR = (1 << 26);
				GPIOB_PSOR = (1 << 21);
				break;
			case 'B' :
				GPIOB_PSOR = (1 << 22);
				GPIOB_PSOR = (1 << 26);
				GPIOB_PCOR = (1 << 21);
				break;
			case 'C' :
				GPIOB_PSOR = (1 << 22);
				GPIOE_PCOR = (1 << 26);
				GPIOB_PCOR = (1 << 21);
				break;
			case 'M' :
				GPIOB_PCOR = (1 << 22);
				GPIOB_PSOR = (1 << 26);
				GPIOB_PCOR = (1 << 21);
				break;
			case 'Y' :
				GPIOE_PCOR = (1 << 26);
				GPIOB_PCOR = (1 << 22);
				GPIOB_PSOR = (1 << 21);
				break;
			case 'W' :
				GPIOB_PCOR = (1 << 22);
				GPIOE_PCOR = (1 << 26);
				GPIOB_PCOR = (1 << 21);
				break;
    }

}
