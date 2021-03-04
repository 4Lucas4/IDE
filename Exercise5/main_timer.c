/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* Spring 2016
*
* Filename: main_timer_template.c
*/

#include "MK64F12.h"
#include "uart.h"
#include "isr.h"
#include <stdio.h>

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK 20485760u /* Default System clock value */

void initPDB(void);
void initGPIO(void);
void initFTM(void);
void initInterrupts(void);

int main(void){
	//initializations
	initPDB();
	initGPIO();
	initFTM();
	uart_init();
	initInterrupts();

	for(;;){
		//To infinity and beyond
	}
}

void initPDB(void){
	
	//Enable clock for PDB module
	SIM_SCGC6 |= SIM_SCGC6_PDB_MASK;

	// Set continuous mode, prescaler of 128, multiplication factor of 20,
	// software triggering, and PDB enabled
	PDB0_SC |= PDB_SC_CONT_MASK;			// Set continuous mode
	PDB0_SC |= PDB_SC_PRESCALER(0x7); // Prescaler of 128 (b111)
	PDB0_SC |= PDB_SC_MULT(0x2);			// Multiplication 20 (b10)
	PDB0_SC |= PDB_SC_TRGSEL(0xF); 		// Software Trigger selected
	PDB0_SC |= PDB_SC_PDBEN_MASK;			// Enable PDB0

	//Set the mod field to get a 1 second period.
	//There is a division by 2 to make the LED blinking period 1 second.
	//This translates to two mod counts in one second (one for on, one for off)
	PDB0_MOD |= PDB_MOD_MOD(0x2); // modulus division by 2 TODO NOT SURE IF THIS IS RIGHT

	//Configure the Interrupt Delay register.
	PDB0_IDLY = 10;

	//Enable the interrupt mask.
	PDB0_SC |= PDB_SC_PDBIE_MASK;

	//Enable LDOK to have PDB0_SC register changes loaded.
	PDB0_SC |= PDB_SC_LDOK_MASK;

	return;
}

void initFTM(void){
	//Enable clock for FTM module (use FTM0)
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

	//turn off FTM Mode to  write protection;
	FTM0_MODE |= FTM_MODE_WPDIS_MASK; // Write 1 to Write Protection Disable bit

	//divide the input clock down by 128
	FTM0_SC |= FTM_SC_PS(0x7);	// Prescaler of 128 (b111)

	//reset the counter to zero
	FTM0_CNT = 0x0000;	// only lower 16 bits are count, rest is always 0


	//Set the overflow rate
	//(Sysclock/128)- clock after prescaler
	//(Sysclock/128)/1000- slow down by a factor of 1000 to go from
	//Mhz to Khz, then 1/KHz = msec
	//Every 1msec, the FTM counter will set the overflow flag (TOF) and
	FTM0->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/1000;

	//Select the System Clock
	FTM0_SC |= FTM_SC_CLKS(0x1);	// System Clock is b01

	//Enable the interrupt mask. Timer overflow Interrupt enable
	FTM0_SC |= FTM_SC_TOIE_MASK;

	return;
}

void initGPIO(void){
	
	//initialize push buttons and LEDs
	Switch2_Init();
	Switch3_Init();
	LED_Init();

	//initialize clocks for each different port used.
	// covered in above Init functions

	//Configure Port Control Register for Inputs with pull enable and pull up resistor
	PORTC_PCR6 |= PORT_PCR_PE_MASK; // Switch2 Pull Enable
	PORTC_PCR6 |= PORT_PCR_PS_MASK; // Switch2 Pull Select (Pull Up)
	PORTA_PCR4 |= PORT_PCR_PE_MASK; // Switch3 Pull Enable 
	PORTA_PCR4 |= PORT_PCR_PS_MASK; // Switch3 Pull Select (Pull Up)

	// Configure mux for Outputs
	// I think this is covered in all 3 inits

	// Switch the GPIO pins to output mode (Red and Blue LEDs)
	// covered in LED_Init

	// Turn off the LEDs
	// covered in LED_Init

	// Set the push buttons as an input
	// covered in Switch2_Init and Switch3_Init

	// interrupt configuration for SW3(Rising Edge) and SW2 (Either)
	PORTC_PCR6 |= PORT_PCR_IRQC(0xB); //SW2
	PORTA_PCR4 |= PORT_PCR_IRQC(0x9); //SW3
	
	return;
}

void initInterrupts(void){
	/*Can find these in MK64F12.h*/
	// Enable NVIC for portA,portC, PDB0,FTM0
	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_EnableIRQ(PDB0_IRQn);
	NVIC_EnableIRQ(FTM0_IRQn);

	return;
}
