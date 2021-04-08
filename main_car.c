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
#include "PWM.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

void DAC0_INIT(void);
void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void); 
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void); 
void delay(int del);

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
int debugcamdata = 1;
int capcnt = 0;
char str[100];

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

int main(void)
{
	int i;
	int LLLAvg = 0;
	int LLAvg = 0;
	int LAvg = 0;
	int CLAvg = 0;
	int CRAvg = 0;
	int RAvg = 0;
	int RRAvg = 0;
	int RRRAvg = 0;
	int TrackThreshold = 1000;
	int BaseSpeed = 40;
	int Differential = 5;
	int CarpetCount = 0;
	int CarpetLimit = 5;
	int PreviousMove = 0; // -3 -2 -1 0 1 2 3
	
	uart_init();
	DAC0_INIT();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	FTM0_init();
	FTM3_init();
	init_PIT();	// To trigger camera read based on integration time
	
	PORTB_PCR2 = PORT_PCR_MUX(1); 
	PORTB_PCR3 = PORT_PCR_MUX(1); 
	GPIOB_PDDR |= (1<<2|1<<3); //output
	GPIOB_PSOR |= (1<<2|1<<3); //hbridge enable high
	
	Straight();
	Forward(BaseSpeed);
	
	
	for(;;) {

		if (debugcamdata) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (1)) {
				GPIOB_PCOR |= (1 << 22);
				
				LLLAvg = 0;
				LLAvg = 0;
				LAvg = 0;
				CLAvg = 0;
				CRAvg = 0;
				RAvg = 0;
				RRAvg = 0;
				RRRAvg = 0;
				
				
				for (i = 0; i < 16; i++) {
					RRRAvg = RRRAvg + line[i];
				}
				RRRAvg = (int)(RRRAvg/16);
				sprintf(str,"RRR: %i\n\r", RRRAvg);
				put(str);
				
				for (i = 16; i < 32; i++) {
					RRAvg = RRAvg + line[i];
				}
				RRAvg = (int)(RRAvg/16);
				sprintf(str,"RR: %i\n\r", RRAvg);
				put(str);
				
				for (i = 32; i < 48; i++) {
					RAvg = RAvg + line[i];
				}
				RAvg = (int)(RAvg/16);
				sprintf(str,"R: %i\n\r", RAvg);
				put(str);
				
				for (i = 48; i < 64; i++) {
					CRAvg = CRAvg + line[i];
				}
				CRAvg = (int)(CRAvg/16);
				sprintf(str,"CR: %i\n\r", CRAvg);
				put(str);
				
				for (i = 64; i < 80; i++) {
					CLAvg = CLAvg + line[i];
				}
				CLAvg = (int)(CLAvg/16);
				sprintf(str,"CL: %i\n\r", CLAvg);
				put(str);
				
				for (i = 80; i < 96; i++) {
					LAvg = LAvg + line[i];
				}
				LAvg = (int)(LAvg/16);
				sprintf(str,"L: %i\n\r", LAvg);
				put(str);
				
				for (i = 96; i < 112; i++) {
					LLAvg = LLAvg + line[i];
				}
				LLAvg = (int)(LLAvg/16);
				sprintf(str,"LL: %i\n\r", LLAvg);
				put(str);
				
				for (i = 112; i < 128; i++) {
					LLLAvg = LLLAvg + line[i];
				}
				LLLAvg = (int)(LLLAvg/16);
				sprintf(str,"LLL: %i\n\n\n\r", LLLAvg);
				put(str);
				
			  if (CRAvg < TrackThreshold && CLAvg > TrackThreshold){ // Hard Left
					Left(3);
					LeftWheel(BaseSpeed-2*Differential);
					RightWheel(BaseSpeed+2*Differential);
					CarpetCount = 0;
					PreviousMove = -3;
				}else if (CRAvg > TrackThreshold && CLAvg < TrackThreshold){ // Hard Right
					Right(3);
					RightWheel(BaseSpeed-2*Differential);
					LeftWheel(BaseSpeed+2*Differential);
					CarpetCount = 0;
					PreviousMove = 3;
				}else if (RAvg < TrackThreshold && LAvg > TrackThreshold){ // Medium Left
					Left(2);
					LeftWheel(BaseSpeed-1*Differential);
					RightWheel(BaseSpeed+1*Differential);
					CarpetCount = 0;
					PreviousMove = -2;
				}else if (RAvg > TrackThreshold && LAvg < TrackThreshold){ // Medium Right
					Right(2);
					RightWheel(BaseSpeed-1*Differential);
					LeftWheel(BaseSpeed+1*Differential);
					CarpetCount = 0;
					PreviousMove = 2;
				}else if (RRAvg < TrackThreshold && LLAvg > TrackThreshold){ // Light Left
					Left(1);
					LeftWheel(BaseSpeed-1*Differential);
					RightWheel(BaseSpeed+1*Differential);
					CarpetCount = 0;
					PreviousMove = -1;
				}else if (RRAvg > TrackThreshold && LLAvg < TrackThreshold){ // Light Right
					Right(1);
					RightWheel(BaseSpeed-1*Differential);
					LeftWheel(BaseSpeed+1*Differential);
					CarpetCount = 0;
					PreviousMove = 1;
				}else if (CRAvg > TrackThreshold && CLAvg > TrackThreshold){ // Straight
					
					Forward(BaseSpeed);
					Straight();
					
//					if (RRAvg < TrackThreshold && LLAvg > TrackThreshold){
//						Right(2);
//						RightWheel(BaseSpeed-1*Differential);
//						LeftWheel(BaseSpeed+1*Differential);
//					}
//					else if (RRAvg > TrackThreshold && LLAvg < TrackThreshold){
//						
//						Left(2);
//						LeftWheel(BaseSpeed-1*Differential);
//						RightWheel(BaseSpeed+1*Differential);
//					}
//					else{Forward(BaseSpeed);Straight();}
					
//					switch(PreviousMove){
//						case 0 :
//							Straight();
//							break;
//						case -2 :
//							Right(1);
//							RightWheel(BaseSpeed);
//							LeftWheel(BaseSpeed+1*Differential);
//							break;
//						case 2 :
//							Left(1);
//							LeftWheel(BaseSpeed);
//							RightWheel(BaseSpeed+1*Differential);
//							break;
//						case -3 :
//							Right(2);
//							RightWheel(BaseSpeed);
//							LeftWheel(BaseSpeed+2*Differential);
//							break;
//						case 3 :
//							Left(2);
//							LeftWheel(BaseSpeed);
//							RightWheel(BaseSpeed+2*Differential);
//							break;
//					}
					
					CarpetCount = 0;
					PreviousMove = 0;
				}else{																											// Stop
					if (CarpetCount > CarpetLimit){
						Stop();
					}
					CarpetCount = CarpetCount + 1;
				}
				
					
					
					
				
//				if ((CenterAvg > TrackThreshold)){
//					Straight();
//					Forward(BaseSpeed);
//					CarpetCount = 0;
//					//put("Straight\n\r");
//					sprintf(str,"Straight\n\r");
//					put(str);
//				}else if (LeftAvg > TrackThreshold){
//					Left();
//					//Forward(BaseSpeed);
//					LeftWheel(BaseSpeed);
//					RightWheel(BaseSpeed+Differential);
//					CarpetCount = 0;
//					//put("Left\n\r");
//					sprintf(str,"Left\n\r");
//					put(str);
//				}else if (RightAvg > TrackThreshold){
//					Right();
//					Forward(BaseSpeed);
//					LeftWheel(BaseSpeed+Differential);
//					CarpetCount = 0;
//					RightWheel(BaseSpeed);
//					//put("Right\n\r");
//					sprintf(str,"Right\n\r");
//					put(str);
//				}else{
//					if (CarpetCount > CarpetLimit){
//						Stop();
//					}
//					CarpetCount = CarpetCount + 1;
//					sprintf(str,"CC: %i\r\n", CarpetCount);
//					put(str);
//					//put("STOP!!!\n\r");
//					sprintf(str,"STOP!!!\n\r");
//					put(str);
//				}
				
				capcnt = 0;
				GPIOB_PSOR |= (1 << 22);
			}
		}

	} //for
} //main

void DAC0_INIT(void) {
    //enable DAC clock
    SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK;
    DAC0_C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
    DAC0_C1 = 0;
}

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	//just grabbing whats in the ADC
	//set ADC_ra, to global variable, ADC0VAL
	ADC0VAL = (uint16_t) ADC0_RA >> 4;
	
	DAC0_DAT0L = ADC0VAL;				//take lower 8 bits of 12-bit i into low reg. 
	DAC0_DAT0H = ADC0VAL >> 8;	//take remaining upper 4-bits into high reg.
	
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
	//int readInt = (FTM2_SC & FTM_SC_TOF_MASK);		//read SC register
	FTM2_SC |= (0 << FTM_SC_TOF_SHIFT);	//write 0 to TOF bit
	
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
	PIT_TFLG0 |= (1 << PIT_TFLG_TIF_SHIFT);
	
	// Setting mod resets the FTM counter
	FTM2_MOD &= FTM_MOD_MOD_MASK;
	//FTM2_MOD = DEFAULT_SYSTEM_CLOCK/100000;
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	//FTM2_MODE |= FTM_MODE_WPDIS_MASK; // Write 1 to Write Protection Disable bit
	FTM2_MODE |= (0 << FTM_MODE_WPDIS_SHIFT);
	
	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT = 0x0;
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN = 0x0;
	
	// Set the period (~10us)
	FTM2_MOD = (uint32_t) DEFAULT_SYSTEM_CLOCK/100000;
	
	// 50% duty
	FTM2_C0V = (uint32_t) (DEFAULT_SYSTEM_CLOCK/100000)/2;
	
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
	FTM2_SC |= (1 << FTM_SC_CLKS_SHIFT);
	
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
	PIT_MCR &= ~PIT_MCR_FRZ_MASK;
		
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = (uint32_t) DEFAULT_SYSTEM_CLOCK/(1/INTEGRATION_TIME);
	//systemcloclcore/(1/INTEGRATION_TIME)
	
	// Enable timer interrupts
	PIT_TCTRL0|= PIT_TCTRL_TIE_MASK; //1 is enabled
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK; //1 is enabled

	// Clear interrupt flag
	PIT_TFLG0 |= (1 << PIT_TFLG_TIF_SHIFT); //1 clears it

	PIT_MCR &= ~PIT_MCR_MDIS_MASK; //0 is enabled
	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}

void init_GPIO(void){
	// Enable LED and GPIO so we can see results
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	/* Set up pins for GPIO
	* 	PTB9 		- camera clk
	*		PTB23		- camera SI
	*		PTB22		- red LED
	*/
	//May need different MUX value for Camera GPIO
	PORTB_PCR9 |= PORT_PCR_MUX(1);			//Enable Camera Clock
	PORTB_PCR23 |= PORT_PCR_MUX(1);		//Enable Camera SI
	PORTB_PCR22 |= PORT_PCR_MUX(1);		//Enable RED LED for GPIO mode
	PORTB_PCR18 |= PORT_PCR_MUX(3);		//FTM
	
	//Set all port B pins to output
	GPIOB_PDDR |= ((1 << 9) | (1 << 23) | (1 << 22));
	return;
}


/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	
		// Single ended 16 bit conversion, no clock divider
		ADC0_CFG1 |= (0 << ADC_CFG1_ADIV_SHIFT);
		ADC0_CFG1 |= (0x3 << ADC_CFG1_MODE_SHIFT);
    
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
		ADC0_SC1A |= (0 << ADC_SC1_DIFF_SHIFT);
		ADC0_SC1A |= ADC_SC1_AIEN_MASK;
		
	
		//Look at SIM_SOPT7
		// Set up FTM2 trigger on ADC0		
		SIM_SOPT7 &= ~SIM_SOPT7_ADC0TRGSEL_MASK;
		SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0xA);	//FTM2 select 1010
		
		SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;	  // Alternative trigger en.
		SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK;		// Pretrigger A, 1010 FTM2
		
		//Select ADC Channel & enable interrupts
		//ADC0_SC1A = 0;
		//ADC0_SC1A |= (0 << 5);
		ADC0_SC1A &= ~ADC_SC1_ADCH_MASK;
		
		
		// Enable NVIC interrupt
		NVIC_EnableIRQ(ADC0_IRQn);
}


void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

