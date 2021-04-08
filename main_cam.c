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
#include "LED.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

void init_FTM2(void); //Lucas
void init_GPIO(void); //Lucas
void init_PIT(void); 	//Lucas
void init_ADC0(void); //Lucas
void FTM2_IRQHandler(void);  //Lucas
void PIT1_IRQHandler(void);		//Lucas
void ADC0_IRQHandler(void);  //Lucas

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
	int FirstTime = 0;
	
	uart_init();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
	
	for(;;) {

//		if (capcnt >= (500)){
//			if (FirstTime == 0) {
//				GPIOB_PCOR |= (1 << 22);
//				
//				for (i = 0; i < 127; i++) {
//					sprintf(str,"%i\n", line[i]);
//					put(str);
//				}
//				
//				capcnt = 0;
//				FirstTime = 1;
//				GPIOB_PSOR |= (1 << 22);
//			}
//			else{
//				GPIOB_PCOR |= (1 << 22);
//				
//				for (i = 0; i < 127; i++) {
//					sprintf(str,"%i\n", line[i]);
//					put(str);
//				}
//				
//				capcnt = 0;
//				GPIOB_PSOR |= (1 << 22);
//			}
//		}
		
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
	//just grabbing whats in the ADC
	//set ADC_ra, to global variable, ADC0VAL
	ADC0VAL = ADC0_RA;
	
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
	//clear by reading the SC reg., then writing it to 0
	int readint = (FTM2_SC & FTM_SC_TOF_MASK);		//read SC register
	FTM2_SC &= ~FTM_SC_TOF_MASK;	//write 0 to TOF bit
	
	// Toggle clk
	//toggle signal output being used as an output for the clock
	GPIOB_PTOR |= (1 << 9);		//Toggle PTB9
	//toggle clkval
	if (clkval == 0){
		clkval = 1;
	}
	else{
		clkval = 0;
	}
	
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
		FTM2_SC &= ~FTM_SC_TOIE_MASK;		//set to 0; disable TOF interrupts
	
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
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	FTM2->MOD = DEFAULT_SYSTEM_CLOCK/100000;
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK; // Write 1 to Write Protection Disable bit
	
	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT = 0x0;
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN = 0x0;
	
	// Set the period (~10us)
	FTM2_MOD = DEFAULT_SYSTEM_CLOCK/100000;
	
	// 50% duty
	FTM2_C0V = (DEFAULT_SYSTEM_CLOCK/100000)/2;
	
	// Set edge-aligned mode
	FTM2_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;			//QUADEN
	FTM2_COMBINE &= ~FTM_COMBINE_DECAPEN0_MASK;	//DECAPEN
	FTM2_COMBINE &= ~FTM_COMBINE_COMBINE0_MASK;	//COMBINE
	FTM2_SC &= ~FTM_SC_CPWMS_MASK;							//CPWMS
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;							//MSnB=1
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;
	
	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
	FTM2_EXTTRIG |= FTM_EXTTRIG_CH0TRIG_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC |= FTM_SC_PS(0x0);	// Prescaler of 1 (b000)
	
	// Set up interrupt
	FTM2_SC |= FTM_SC_TOIE_MASK;
	NVIC_EnableIRQ(FTM2_IRQn);
	
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	//set freeze flag, 0<< freeze flag
	// Enable timers to continue in debug mode
		
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL1 = DEFAULT_SYSTEM_CLOCK/(1/INTEGRATION_TIME);
	//systemcloclcore/(1/INTEGRATION_TIME)
	
	// Enable timer interrupts
	PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK; //1 is enabled
	
	// Enable the timer
	PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK; //1 is enabled

	// Clear interrupt flag
	PIT_TFLG1 |= PIT_TFLG_TIF_MASK; //1 clears it

	PIT_MCR &= ~PIT_MCR_MDIS_MASK; //0 is enabled
	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT1_IRQn);
	return;
}



void init_GPIO(void){
	// Enable LED and GPIO so we can see results
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	//Set all port B pins to output
	GPIOB_PDDR |= ((1 << 9) | (1 << 23) | (1 << 22));
	
	/* Set up pins for GPIO
	* 	PTB9 		- camera clk
	*		PTB23		- camera SI
	*		PTB22		- red LED
	*/
	//May need different MUX value for Camera GPIO
	PORTB_PCR9 = PORT_PCR_MUX(1);			//Enable Camera Clock
	PORTB_PCR23 = PORT_PCR_MUX(1);		//Enable Camera SI
	PORTB_PCR22 = PORT_PCR_MUX(1);		//Enable RED LED for GPIO mode
	PORTB_PCR18 = PORT_PCR_MUX(3);		//Enable FTM
	return;
}


/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	
		// Single ended 16 bit conversion, no clock divider
		ADC0_CFG1 |= ADC_CFG1_ADIV(0);
		ADC0_CFG1 |= ADC_CFG1_MODE(3);
    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
	
    //diff
		ADC1_SC1A |= ADC_SC1_DIFF_MASK;
		
	
		//Look at SIM_SOPT7
		// Set up FTM2 trigger on ADC0		
		SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0xA);	//FTM2 select 1010
		SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;	  // Alternative trigger en.
		SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK;		// Pretrigger A, 1010 FTM2
		
		//Select ADC Channel & enable interrupts
		ADC0_SC1A = 0;
		ADC0_SC1A |= (0 << 5);
		ADC0_SC1A |= ADC_SC1_ADCH(0x0);
		ADC0_SC1A |= (1 << ADC_SC1_AIEN_SHIFT);
		
		// Enable NVIC interrupt
		NVIC_EnableIRQ(ADC0_IRQn);
}
