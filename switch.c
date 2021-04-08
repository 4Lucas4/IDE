/*
 * File:        switch.c
 * Purpose:     Provide switch control
 *
 * Notes:		
 *
 */

// NOTE: The LEDs and the Switches on the K64 are all active LOW

#include "MK64F12.h"                    // Device header

void Switch2_Init(void)
{
	// 12.2.12 System Clock Gating Control Register 5
	// Port C is Bit 11 in SCGC5
	// Enable clock for Port C PTC6 button
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	 
	
	// Configure the Mux for the button
    PORTC_PCR6 = PORT_PCR_MUX(1);
	 
	// Set the push button as an input
	// GPIOC_PDDR is the direction control for Port C
	// set it to ZERO at BIT6 for an input
    GPIOC_PDDR = (0 << 6);
	
}
unsigned char Switch2_Pressed(void)
{
	// check if switch2 pressed ()
	// if PORT C - BIT6 is ZERO, then the switch is pressed
	// PDIR - Port Data Input Register
    if ((GPIOC_PDIR&(1<<6)) != 0){
	// return a ZERO if NOT Pressed
        return 0;
    }else{
    // return a 0xFF if Pressed
        return 0xFF;
    }
}

void Switch3_Init(void)
{
	// 12.2.12 System Clock Gating Control Register 5
	// Port A is Bit 9 in SCGC5
	// Enable clock for Port A PTC4 button
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	 
	
	// Configure the Mux for the button
    PORTA_PCR4 = PORT_PCR_MUX(1);
	 
	// Set the push button as an input
	// GPIOA_PDDR is the direction control for Port A
	// set it to ZERO at BIT4 for an input
    GPIOA_PDDR = (0 << 4);
	 
}
unsigned char Switch3_Pressed(void)
{
	// check if switch3 pressed ()
	// if PORT A - BIT4 is ZERO, then the switch is pressed
	// PDIR - Port Data Input Register
    if ((GPIOA_PDIR&(1<<4)) != 0){
	// return a ZERO if NOT Pressed
        return 0;
    }else{
    // return a 0xFF if Pressed
        return 0xFF;
    }
}

