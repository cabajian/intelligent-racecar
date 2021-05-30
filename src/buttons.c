/*
 * Button initialization
 * 
 * Author: Chris Abajian, Becky Reich
 * Created: 11/2/2020
 */
#include "MK64F12.h"
#include "buttons.h"

void init_buttons(void) {
    // Enable clock for SW2 and SW3 buttons
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    // Configure the Mux for the buttons
    SW_2 = PORT_PCR_MUX(1);
    SW_3 = PORT_PCR_MUX(1);
    // Set the push buttons as inputs
    GPIOC_PDDR &= ~(1UL << SW_2_PIN);
    GPIOA_PDDR &= ~(1UL << SW_3_PIN);
}

int is_sw2_pressed() {
    return !(GPIOC_PDIR & (1UL << SW_2_PIN));
}

int is_sw3_pressed() {
    return !(GPIOA_PDIR & (1UL << SW_3_PIN));
}
