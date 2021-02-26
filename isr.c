/*
 * isr.c
 */

#include "isr.h"
#include "MK64F12.h"
#include <stdio.h>
#include "uart.h"
#include "led.h"
#include "switch.h"

//variables global to the IRQ handlers which dictates if timer is enabled &  timer counter
int LED_Toggle = 1;
int SW2_Pressed = 0;
int counter = 0;
int Timer_Enabled = 1;

void PDB0_IRQHandler(void){ //For PDB timer
	
	 PDB0_SC &= ~0x40; // clear the interrupt in register PDB0_SC bit 6
	
	if (LED_Toggle){	// toggle the LED with use of global variable
		LED_On('Y');
		LED_Toggle = 0;
	}else{
		LED_Off();
		LED_Toggle = 1;
	}	
	return;
}
	
void FTM0_IRQHandler(void){ //For FTM timer
	
	FTM0_SC &= ~0x80;	// clear the interrupt in register FTM0_SC bit 7
	
	if (SW2_Pressed) // if SW2 is pressed, increment global counter by 1
		counter++;
	
	return;
}
	
void PORTA_IRQHandler(void){ //For switch 3
	
	PORTA_ISFR &= ~PORT_ISFR_ISF_MASK; // clear PORTA interrupt
	
	if (Timer_Enabled){
		PDB0_SC &= ~PDB_SC_PDBEN_MASK; // Disable PDB0 Timer bit 7 (since that is LED Toggle)
		Timer_Enabled = 0;
	}else{
		PDB0_SC |= PDB_SC_PDBEN_MASK; // Re-Enable PDB0 Timer bit 7
		PDB0_SC |= PDB_SC_SWTRIG_MASK; // Reset and restart counter with Software Trigger
		Timer_Enabled = 1;
	}
	
	return;
}
	
void PORTC_IRQHandler(void){ //For switch 2
	
	PORTC_ISFR &= ~PORT_ISFR_ISF_MASK; // clear PORTC interrupt
	
	if (Switch2_Pressed()){ // if SW2 is pressed, update global variable
		SW2_Pressed = 1;
	}else{
		SW2_Pressed = 0;
	}
	
	
	return;
}
