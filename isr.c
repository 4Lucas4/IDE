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
//Boolean 1=True 0=False
//int LED_Toggle = 1;
int SW2_Pressed = 0;
int Timer_Enabled = 0;

//Int counter
int counter = 0;

void PDB0_IRQHandler(void){ //For PDB timer
	
	PDB0_SC &= ~0x40; // clear the interrupt in register PDB0_SC bit 6
	GPIOE_PTOR = (1 << 26);
//	if (LED_Toggle){	// toggle the LED with use of global variable
//		LED_On('Y');
//		delay();
//		LED_Toggle = 0;
//	}else{
//		LED_Off();
//		delay();
//		LED_Toggle = 1;
	//}	
	return;
}
	
void FTM0_IRQHandler(void){ //For FTM timer
	
	FTM0_SC &= ~0x80;	// clear the interrupt in register FTM0_SC bit 7
	
	if (SW2_Pressed) // if SW3 is pressed, increment global counter by 1
		counter++;
	
	return;
}
	
void PORTA_IRQHandler(void){ //For switch 3
	
	PORTA_PCR4 |= PORT_PCR_ISF_MASK; // clear PORTA interrupt (1 is cleared)
	
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
	char intstring[255];
	PORTC_ISFR &= ~PORT_ISFR_ISF_MASK; // clear PORTC interrupt
	
	if (Switch2_Pressed()){ // if SW2 is pressed, update global variable
		SW2_Pressed = 1;
	}else{
		SW2_Pressed = 0;
		if (counter>0){
			sprintf(intstring, "%d", counter);
			put(intstring);
			put(" ms");
			put("\n\r");
			counter = 0;
		}
	}
	
	
	return;
}
