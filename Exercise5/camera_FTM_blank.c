/*
 * Freescale Cup linescan camera code
 *
 *	This method of capturing data from the line
 *	scan cameras uses a flex timer module, periodic
 *	interrupt timer, an ADC, and some GPIOs.
 *	CLK and SI are driven with GPIO because the FTM2
 *	module used doesn't have any output pins on the
 * 	development board. The PIT timer is used to 
 *  control the integration period. When it overflows
 * 	it enables interrupts from the FTM2 module and then
 *	the FTM2 and ADC are active for 128 clock cycles to
 *	generate the camera signals and read the camera 
 *  output.
 *
 *	PTB8			- camera CLK
 *	PTB23 		- camera SI
 *  ADC0_DP1 	- camera AOut
 *
 * Author:  Alex Avery
 * Created:  11/20/15
 * Modified:  11/23/15
 */

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];

// These variables are for streaming the camera
//	 data over UART
int debugcamdata = 0;
int capcnt = 0;
char str[100];

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

int main(void)
{
	int i;
	
	uart_init();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
	
	for(;;) {

		if (debugcamdata) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (500)) {
				GPIOB_PCOR |= (1 << 22);
				// send the array over uart
				sprintf(str,"%i\n\r",-1); // start value
				put(str);
				for (i = 0; i < 127; i++) {
					sprintf(str,"%i\n", line[i]);
					put(str);
				}
				sprintf(str,"%i\n\r",-2); // end value
				put(str);
				capcnt = 0;
				GPIOB_PSOR |= (1 << 22);
			}
		}

	} //for
} //main


/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	//INSERT CODE HERE
	
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  	//INSERT CODE HERE
	
	// Toggle clk
	//INSERT CODE HERE
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		//INSERT CODE HERE
	
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	//INSERT CODE HERE
	
	// Setting mod resets the FTM counter
	//INSERT CODE HERE
	
	// Enable FTM2 interrupts (camera)
	//INSERT CODE HERE
	
	return;
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	//INSERT CODE HERE

	// Disable Write Protection
	//INSERT CODE HERE
	
	// Set output to '1' on init
	//INSERT CODE HERE
	
	// Initialize the CNT to 0 before writing to MOD
	//INSERT CODE HERE
	
	// Set the Counter Initial Value to 0
	//INSERT CODE HERE
	
	// Set the period (~10us)
	//INSERT CODE HERE
	
	// 50% duty
	//INSERT CODE HERE
	
	// Set edge-aligned mode
	//INSERT CODE HERE
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	//INSERT CODE HERE
	
	// Enable hardware trigger from FTM2
	//INSERT CODE HERE
	
	// Don't enable interrupts yet (disable)
	//INSERT CODE HERE
	
	// No prescalar, system clock
	//INSERT CODE HERE
	
	// Set up interrupt
	//INSERT CODE HERE
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	//INSERT CODE HERE
	
	// Enable timers to continue in debug mode
	//INSERT CODE HERE // In case you need to debug
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	//INSERT CODE HERE
	
	// Enable timer interrupts
	//INSERT CODE HERE
	
	// Enable the timer
	//INSERT CODE HERE

	// Clear interrupt flag
	//INSERT CODE HERE

	// Enable PIT interrupt in the interrupt controller
	//INSERT CODE HERE
	return;
}


/* Set up pins for GPIO
* 	PTB9 		- camera clk
*		PTB23		- camera SI
*		PTB22		- red LED
*/
void init_GPIO(void){
	// Enable LED and GPIO so we can see results
	//INSERT CODE HERE
	return;
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
    //INSERT CODE HERE
	
	// Single ended 16 bit conversion, no clock divider
	//INSERT CODE HERE
    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
    //INSERT CODE HERE
    
    // Set to single ended mode	
	//INSERT CODE HERE
	
	// Set up FTM2 trigger on ADC0
	//INSERT CODE HERE // FTM2 select
	//INSERT CODE HERE // Alternative trigger en.
	//INSERT CODE HERE // Pretrigger A
	
	// Enable NVIC interrupt
    //INSERT CODE HERE
}
