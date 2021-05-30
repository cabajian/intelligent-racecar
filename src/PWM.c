/*
 * Pulse-Width-Modulation Code for K64
 *
 * Motor shield FTM pinouts:
 *
 * K64    Module    ALT  Shield
 * -----  --------  ---  ------
 * PTC1   FTM0_CH0  4    P22
 * PTC2   FTM0_CH1  4    P23
 * PTC3   FTM0_CH2  4    P28
 * PTC4   FTM0_CH3  4    P29
 * PTC8   FTM3_CH4  3    SIG
 * 
 * Author: Chris Abajian, Becky Reich
 * Created: 11/1/2020
 */
#include "MK64F12.h"
#include "pwm.h"

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK    20485760u 
#define FTM0_MOD_VALUE          (DEFAULT_SYSTEM_CLOCK/MOTOR_FREQ)  // 10,000 Hz
#define FTM3_MOD_VALUE          (DEFAULT_SYSTEM_CLOCK/SERVO_FREQ/8)   // 50 Hz (8 prescale)

void move_servo(double duty) {
    FTM3_set_duty_cycle(duty, SERVO_FREQ);
}

void move_motor_left(unsigned int duty, int dir) {
    FTM0_set_duty_cycle(duty, MOTOR_FREQ, 0, dir);
}
void move_motor_right(unsigned int duty, int dir) {
    FTM0_set_duty_cycle(duty, MOTOR_FREQ, 1, dir);
}

/*
 * Change the motor duty cycle and frequency
 *
 * @param duty_cycle (0 to 100)
 * @param frequency (~1000 Hz to 20000 Hz)
 * @param dir: 1 for pin C2/C4 active, else pin C1/C3 active 
 */
void FTM0_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int motor, int dir)
{
    // Calculate the new cutoff value
    uint16_t mod = (uint16_t) (((DEFAULT_SYSTEM_CLOCK / frequency) * duty_cycle) / 100);
  
    // Set outputs
    if(dir) {
			  if (!motor) {
					// Motor 1
					FTM0_C1V = mod;
					FTM0_C0V = 0;
				} else {
					// Motor 2
					FTM0_C3V = mod; 
					FTM0_C2V = 0;
				}
    } else {
			  if (!motor) {
					// Motor 1
					FTM0_C0V = mod;
					FTM0_C1V = 0;
				} else {
					// Motor 2
					FTM0_C2V = mod;
					FTM0_C3V = 0;
				}
    }

    // Update the clock to the new frequency
    FTM0_MOD = (DEFAULT_SYSTEM_CLOCK / frequency);
}

/*
 * Change the servo "duty cycle" and frequency
 *
 * @param duty_cycle (0 (full left) to 100 (full right)) 
 * @param frequency
 */
void FTM3_set_duty_cycle(double duty_cycle, unsigned int frequency)
{
    // Calculate the new cutoff value
    uint16_t mod = (uint16_t) (((DEFAULT_SYSTEM_CLOCK / frequency / 8) * duty_cycle) / 100);

    // Set outputs
    FTM3_C4V = mod;

    // Update the clock to the new frequency
    FTM3_MOD = (DEFAULT_SYSTEM_CLOCK / frequency / 8);
}

/*
 * Initialize the FlexTimer for PWM
 */
void init_FTM0()
{
    // 12.2.13 Enable the clock to the FTM0 Module
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
    
    // Enable clock on PORT C so it can output the PWM signals
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    
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
    
    // Motor 1
    FTM0_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
    FTM0_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
    // Motor 2
    FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
    FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;
    
    // 39.3.3 FTM Setup
    // Set prescale value to 1 
    // Chose system clock source
    // Timer Overflow Interrupt Enable
    FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1); 
}

/*
 * Initialize the FlexTimer for PWM
 */
void init_FTM3()
{
    // 12.2.13 Enable the clock to the FTM3 Module
    SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
    
    // Enable clock on PORT C so it can output the PWM signals
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    
    // 11.4.1 Route the output of FTM channel 0 to the pins
    // Use drive strength enable flag to high drive strength
    PORTC_PCR8  = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; //Ch4
    
    // 39.3.10 Disable Write Protection
    FTM3_MODE |= FTM_MODE_WPDIS_MASK;
    
    // 39.3.4 FTM Counter Value
    // Initialize the CNT to 0 before writing to MOD
    FTM3_CNT = 0;
    
    // 39.3.8 Set the Counter Initial Value to 0
    FTM3_CNTIN = 0;
    
    // 39.3.5 Set the Modulo resister
    FTM3_MOD = FTM3_MOD_VALUE;

    // 39.3.6 Set the Status and Control of both channels
    // Used to configure mode, edge and level selection
    // See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
    FTM3_C4SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM3_C4SC &= ~FTM_CnSC_ELSA_MASK;
    
    // 39.3.3 FTM Setup
    // Set prescale value to 8
    // Chose system clock source
    // Timer Overflow Interrupt Enable
    FTM3_SC = FTM_SC_PS(3) | FTM_SC_CLKS(1);
}

void init_pwm() {
    init_FTM0();
    init_FTM3();
    
    // Enable motor shield H Bridges (ENA/PTB2, ENB/PTB3)
    // Enable Port B clocks
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    // Configure the Signal Multiplexer for the Port B GPIO Pins
    PORTB_PCR2 = PORT_PCR_MUX(1);
    PORTB_PCR3 = PORT_PCR_MUX(1);
    // Confugre as outputs
    GPIOB_PDDR |= (1 << 2) | (1 << 3);
    // Set high
    GPIOB_PSOR |= (1 << 2) | (1 << 3);
}
