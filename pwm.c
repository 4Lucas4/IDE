/*
 * Pulse-Width-Modulation Code for K64
 * PWM signal can be connected to output pins PC3 and PC4
 * 
 * Author: Brent Dimmig <bnd8678@rit.edu>
 * Modified by: Carson Clarke-Magrab <ctc7359@rit.edu>
 * Created: 2/20/2014
 * Modified: 2/25/2020
 */
#include "MK64F12.h"
#include "pwm.h"

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK    20485760u 
#define FTM0_MOD_VALUE          (DEFAULT_SYSTEM_CLOCK/10000)
#define FTM3_MOD_VALUE          (DEFAULT_SYSTEM_CLOCK/50)/128	//(DEFAULT_SYSTEM_CLOCK/10000)


/*
 * Change the motor duty cycle and frequency
 *
 * @param duty_cycle (0 to 100)
 * @param frequency (~1000 Hz to 20000 Hz)
 * @param dir: 1 for pin C4 active, else pin C3 active 
 */
void FTM0_C2C3_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int dir)
{
	// Calculate the new cutoff value
	uint16_t mod = (uint16_t) (((DEFAULT_SYSTEM_CLOCK / frequency) * duty_cycle) / 100);
  
	// Set outputs 
	if(dir) {
	    FTM0_C3V = mod; 
	    FTM0_C2V = 0;
	} else {
	    FTM0_C2V = mod; 
	    FTM0_C3V = 0;
	}

	// Update the clock to the new frequency
	FTM0_MOD = (DEFAULT_SYSTEM_CLOCK / frequency);
}

void FTM0_C0C1_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int dir)
{
	// Calculate the new cutoff value
	uint16_t mod = (uint16_t) (((DEFAULT_SYSTEM_CLOCK / frequency) * duty_cycle) / 100);
  
	// Set outputs 
	if(dir) {
	    FTM0_C1V = mod; 
	    FTM0_C0V = 0;
	} else {
	    FTM0_C0V = mod; 
	    FTM0_C1V = 0;
	}

	// Update the clock to the new frequency
	FTM0_MOD = (DEFAULT_SYSTEM_CLOCK / frequency);
}

void LeftWheel(signed int speed) //RM3 BM4
{
	if (speed >= 0){
		FTM0_C2C3_set_duty_cycle((unsigned)speed, 10000, 1); //////
	}else{
		FTM0_C2C3_set_duty_cycle((unsigned)(-speed),10000,0);
	}
}

void RightWheel(signed int speed) //RM1 BM2
{
	if (speed >= 0){
		FTM0_C0C1_set_duty_cycle((unsigned)speed, 10000, 0); //////
	}else{
		FTM0_C0C1_set_duty_cycle((unsigned)(-speed),10000,1);
	}
}

void Forward(unsigned int speed){
	LeftWheel(speed);
	RightWheel(speed);
}

void Reverse(unsigned int speed){
	LeftWheel(-speed);
	RightWheel(-speed);
}

void Stop(void){
	LeftWheel(0);
	RightWheel(0);
}

void Steer(unsigned int amount){
	FTM3_set_duty_cycle(amount);
}

//1ms - 2ms servo
// L 5 . 6 . (7) . 8 . 9 R
//		for (i=5; i<10; i++) {
//			FTM3_set_duty_cycle(i);
//			delay(10);
//		}

void Straight(void){
	Steer(6);
}

void Left(unsigned int amount){
	Steer(6-amount);
}

void Right(unsigned int amount){
	Steer(6+amount);
}

/*
 * Initialize the FlexTimer for PWM
 */
void FTM0_init()
{
	// 12.2.13 Enable the clock to the FTM0 Module
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	
	// Enable clock on PORT C so it can output the PWM signals
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //***Maybe change
	
	// 11.4.1 Route the output of FTM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
		PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch0
    PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch1
    PORTC_PCR3 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch2
    PORTC_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch3
	
	// 39.3.10 Disable Write Protection
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	
	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM0_CNT = 0;
	
	// 39.3.8 Set the Counter Initial Value to 0
	FTM0_CNTIN = 0;
	
	// 39.3.5 Set the Modulo resister
	FTM0_MOD = FTM0_MOD_VALUE;

	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
	FTM0_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
	
	// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;
	FTM0_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
	
	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1); 
}

/*
 * Change the motor duty cycle and frequency
 *
 * @param duty_cycle (0 to 100)
 * @param frequency (~1000 Hz to 20000 Hz)
 * @param dir: 1 for pin C4 active, else pin C3 active 
 */
void FTM3_set_duty_cycle(unsigned int duty_cycle)
{
	unsigned int frequency = ((DEFAULT_SYSTEM_CLOCK/50)/128);
	// Calculate the new cutoff value
	uint16_t mod = (uint16_t) ((frequency * duty_cycle) / 100);
  
	// Set output
	FTM3_C4V = mod;

}

/*
 * Initialize the FlexTimer for PWM
 */
void FTM3_init()
{
	// 12.2.13 Enable the clock to the FTM0 Module
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
	
	// Enable clock on PORT C so it can output the PWM signals
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// 11.4.1 Route the output of FTM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
    PORTC_PCR8 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; //Ch4
	
	// 39.3.10 Disable Write Protection
	FTM3_MODE |= FTM_MODE_WPDIS_MASK;
	
	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM3_CNT = 0;
	
	// 39.3.8 Set the Counter Initial Value to 0
	FTM3_CNTIN = 0;
	
	// 39.3.5 Set the Modulo resister
	FTM3_MOD = FTM3_MOD_VALUE; //*** CHECK HERE

	// 39.3.6 Set the Status and Control of channel
	
	// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM3_C4SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM3_C4SC &= ~FTM_CnSC_ELSA_MASK;
	
	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM3_SC = FTM_SC_PS(7) | FTM_SC_CLKS(1); 
}
