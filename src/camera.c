/*
 * Freescale Cup linescan camera code
 *
 *  This method of capturing data from the line
 *  scan cameras uses a flex timer module, periodic
 *  interrupt timer, an ADC, and some GPIOs.
 *  CLK and SI are driven with GPIO because the FTM2
 *  module used doesn't have any output pins on the
 *  development board. The PIT timer is used to 
 *  control the integration period. When it overflows
 *  it enables interrupts from the FTM2 module and then
 *  the FTM2 and ADC are active for 128 clock cycles to
 *  generate the camera signals and read the camera 
 *  output.
 *
 *  Track is 24" wide w/ 1" black lines on either side
 *
 *  PTB9        - camera CLK
 *  PTB23       - camera SI
 *  ADC0_DP0    - camera AOut
 *
 * Author:  Chris Abajian, Becky Reich
 * Created:  11/1/2020
 */

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "led.h"
#include "pwm.h"
#include "camera.h"
#include "main.h"

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
// ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
int line[128] = {0};

#if DEBUG == 1
int line_temp[128] = {0};
int line_buff[128] = {0};
int line_derivative[128] = {0};
char str[100];
#endif

// thresholds (split into two since left/right light conditions can vary a lot)
int line_max_left = 0;
int line_max_right = 0;

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

#if DEBUG == 1
void convolve(const int *x, const int *h, int *y, const int xSize, const int hSize) {
    // size of y (output vector) needs to be the same size as f (input vector)
    // start hSizein so we don't index less than 0  (boundary conditio
    for (int i =(hSize-1); i<xSize; i++) {
        int sum = 0;
        for (int j=hSize; j>=0;j--) {
            sum += h[j]*x[i-j];
        }
        y[i]=sum;
    }
}

int H1[3] = {1, 1, 1};
int H2[5] = {1, 2, 1};
int H3[3] = {-1	, 1};

/* 
* Centering algorithm to align the car.
*/
void send_line() {
	
    NVIC_DisableIRQ(FTM2_IRQn);

    memcpy(line_temp, line, sizeof(int)*128);
    int lth = line_max_left;
    int rth = line_max_right;
	
    // plot raw line data
    debug_camera(line_temp, 0);
	
	  //convolve(line_temp, H1, line_buff, 128, 3);

		line_derivative[0] = 0;
		line_derivative[1] = 0;
		line_derivative[2] = 0;
		for (int i = 3; i <= 64; i++) {
			if (line_temp[i] > (lth-lth/5)) {
				line_derivative[i] = 1;
			} else {
				line_derivative[i] = 0;
			}
			// remove previous error in "101" and "1001" sequences
			if ((line_derivative[i-2] == 1 && line_derivative[i-1] == 0 && line_derivative[i] == 1) || \
				  (line_derivative[i-3] == 1 && line_derivative[i-2] == 0 && line_derivative[i-1] == 0 && line_derivative[i] == 1)) {
				line_derivative[i-2] = 1;
			  line_derivative[i-1] = 1;
			}
		}
		for (int i = 65; i <= 127; i++) {
			if (line_temp[i] > (rth-rth/5)) {
				line_derivative[i] = 1;
			} else {
				line_derivative[i] = 0;
			}
			// remove previous error in "101" and "1001" sequences
			if ((line_derivative[i-2] == 1 && line_derivative[i-1] == 0 && line_derivative[i] == 1) || \
				  (line_derivative[i-3] == 1 && line_derivative[i-2] == 0 && line_derivative[i-1] == 0 && line_derivative[i] == 1)) {
				line_derivative[i-2] = 1;
			  line_derivative[i-1] = 1;
			}
		}

    //convolve(line_temp, H3, line_derivative, 128, 2);
		
		debug_camera(line_derivative, 2);
		
		NVIC_EnableIRQ(FTM2_IRQn);
}

/*
* Debug camera function. Sends 128 bit vector over uart.
*
* camera parameter modifies the start/end values.
*   For matlab: camera = 0 --> unfiltered raw data
*                        1 --> smoothed data
*                        2 --> derivative data
*/
void debug_camera(const int *data, const short camera) {
    // send the array over uart
    sprintf(str,"%i\n\r",-1-2*camera); // start value
    uart0_put(str);
    for (int i = 0; i < 128; i++) {
        sprintf(str,"%i\n", data[i]);
        uart0_put(str);
    }
    sprintf(str,"%i\n\r",-2-2*camera); // end value
    uart0_put(str);
}
#endif

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
    // Reading ADC0_RA clears the conversion complete flag
    // Read upper 12 bits
    ADC0VAL = ADC0_RA;
}

/* 
* FTM2 handles the camera driving logic
*   This ISR gets called once every integration period
*       by the periodic interrupt timer 0 (PIT0)
*   When it is triggered it gives the SI pulse,
*       toggles clk for 128 cycles, and stores the line
*       data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
    // Clear interrupt
    FTM2_SC &= ~FTM_SC_TOF_MASK;
    
    // Toggle clk
    GPIOB_PTOR |= (1 << 9);
    clkval = !clkval;
            
    // Line capture logic
    if ((pixcnt >= 2) && (pixcnt < 256)) {
        if (!clkval) {  // check for falling edge
            // ADC read (note that integer division is 
            //  occurring here for indexing the array)
            line[127-pixcnt/2] = (int) ADC0VAL;
            if ((pixcnt/2 < 64) && (int)ADC0VAL > line_max_left) line_max_left = (int)ADC0VAL;
            if ((pixcnt/2 >= 64) && (int)ADC0VAL > line_max_right) line_max_right = (int)ADC0VAL;
        }
        pixcnt += 1;
    } else if (pixcnt < 2) {
        if (pixcnt == -1) {
            GPIOB_PSOR |= (1 << 23); // SI = 1
        } else if (pixcnt == 1) {
            GPIOB_PCOR |= (1 << 23); // SI = 0
            // ADC read
            line[127] = (int) ADC0VAL;
					  line_max_left = 0;
            line_max_right = 0;
        } 
        pixcnt += 1;
    } else {
        GPIOB_PCOR |= (1 << 9); // CLK = 0
			
        #if DEBUG != 1
        control(line, line_max_left, line_max_right); // control algorithm
        #endif

        clkval = 0; // make sure clock variable = 0
        pixcnt = -2; // reset counter
        // Disable FTM2 interrupts (until PIT0 overflows
        //   again and triggers another line capture)
        FTM2_SC &= ~FTM_SC_TOIE_MASK;
    }
}

/* PIT0 determines the integration period
*       When it overflows, it triggers the clock logic from
*       FTM2. Note the requirement to set the MOD register
*   to reset the FTM counter because the FTM counter is 
*       always counting, I am just enabling/disabling FTM2 
*       interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
    // Clear interrupt
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
    
    // Setting mod resets the FTM counter
    //FTM2_MOD = FTM_MOD_MOD(100); // camera clk (~10us period)
    //FTM2_MOD = FTM_MOD_MOD(DEFAULT_SYSTEM_CLOCK/100000/2); // camera clk (~10us period)
    FTM2_MOD = 200;
    
    // Enable FTM2 interrupts (camera)
    FTM2_SC |= FTM_SC_TOIE_MASK;
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
    // Enable clock
    SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

    // Disable Write Protection
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;
    
    // Set output to '1' on init
    FTM2_OUTINIT = FTM_OUTINIT_CH0OI_MASK;
    
    // Initialize the CNT to 0 before writing to MOD
    FTM2_CNT = 0;
    
    // Set the Counter Initial Value to 0
    FTM2_CNTIN = 0;
    
    // Set the period (~10us)
    //FTM2_MOD = FTM_MOD_MOD(DEFAULT_SYSTEM_CLOCK/100000/2); // camera clk
    FTM2_MOD = FTM_MOD_MOD(200); // camera clk
    
    // 50% duty (match value for output clock, aka half of the mod value)
    //FTM2_C0V = FTM_CnV_VAL((DEFAULT_SYSTEM_CLOCK/100000)/4);
    FTM2_C0V = FTM_CnV_VAL(100);
    
    // Set edge-aligned mode 
    FTM2_C0SC |= FTM_CnSC_MSB_MASK; // MSnB:MSnA = 1X
    FTM2_C0SC |= FTM_CnSC_MSA_MASK; // MSnB:MSnA = 1X
    
    // Enable High-true pulses
    // ELSB = 1, ELSA = 0
    FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
    FTM2_C0SC &= ~(FTM_CnSC_ELSA_MASK);
    
    // Enable hardware trigger from FTM2
    FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
    
    // Don't enable interrupts yet (disable)
    FTM2_SC &= ~(FTM_SC_TOIE_MASK);
    
    // No prescalar, system clock
    FTM2_SC |= FTM_SC_PS(0x0);
    FTM2_SC |= FTM_SC_CLKS(0x1);
    
    // Set up interrupt
    NVIC_EnableIRQ(FTM2_IRQn);
}

/* Initialization of PIT timer to control 
*       integration period
*/
void init_PIT(void){
    // Setup periodic interrupt timer (PIT)
    
    // Enable clock for timers
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT_MCR &= ~(PIT_MCR_MDIS_MASK); // 0: clock is enabled
    
    // Enable timers to continue in debug mode
    PIT_MCR &= ~(PIT_MCR_FRZ_MASK); // In case you need to debug
    
    // PIT clock frequency is the system clock
    // Load the value that the timer will count down from
    PIT_LDVAL0 = (DEFAULT_SYSTEM_CLOCK*INTEGRATION_TIME);
    
    // Enable timer interrupts
    PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
    
    // Enable the timer
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

    // Clear interrupt flag
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

    // Enable PIT interrupt in the interrupt controller
    NVIC_EnableIRQ(PIT0_IRQn);
}

/* Set up pins for GPIO
*       PTB9        - camera clk
*       PTB23       - camera SI
*       PTB22       - red LED
*/
void init_GPIO(void){    
    // Enable clk, si
    PORTB_PCR9   = PORT_PCR_MUX(1);   //clk
    PORTB_PCR9  |= PORT_PCR_DSE_MASK; //clk drive strength
    PORTB_PCR23  = PORT_PCR_MUX(1);   //si
    PORTB_PCR23 |= PORT_PCR_DSE_MASK; //si drive strength
    
    // Set to output
    GPIOB_PDDR |= (1 << 9);  // clk
    GPIOB_PDDR |= (1 << 23); // si
    
    // Turn off
    GPIOB_PDOR |= (1 << 9);  // clk
    GPIOB_PDOR |= (1 << 23); // si
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    
    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
    
    // Set up FTM2 trigger on ADC0
    SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(10); // FTM2 select
    SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
    SIM_SOPT7 &= ~(SIM_SOPT7_ADC0PRETRGSEL_MASK); // Pretrigger A
    
    // Single ended 16 bit conversion, no clock divider
    ADC0_CFG1 |= (ADC_CFG1_MODE(0x3) | ADC_CFG1_ADIV(0x0));
    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0;  calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
    
    // Set to single ended mode (DADP0 input channel, interrupts enabled, single ended)
    ADC0_SC1A = 0;
    //ADC0_SC1A &= ~(ADC_SC1_ADCH_MASK); // DADP0
    ADC0_SC1A |= ADC_SC1_ADCH(0); // DADP0
    ADC0_SC1A |= ADC_SC1_AIEN_MASK; // Enable Interrupts
    ADC0_SC1A &= ~(ADC_SC1_DIFF_MASK);
    
    // Enable NVIC interrupt
    NVIC_EnableIRQ(ADC0_IRQn);
}

/* Initialize all required components */
void init_camera() {
    init_GPIO(); // For CLK and SI output on GPIO
    init_FTM2(); // To generate CLK, SI, and trigger ADC
    init_ADC0();
    init_PIT();  // To trigger camera read based on integration time
}
