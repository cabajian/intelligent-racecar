/* 
Title:   led.c
Purpose: K64F GPIO LED Initialization and Functions
Name:    Chris Abajian (cxa6282@rit.edu)
Date:    09/09/2020
*/

#include "MK64F12.h"    // Device header
#include "led.h"        // LED header 


void led_init(void){
    // Enable clocks on Ports B and E for LED timing
    SIM_SCGC5 |= (SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK);
    // Configure the Signal Multiplexer for GPIO
    LED_RED_PORT   |= PORT_PCR_MUX(1);
    LED_GREEN_PORT |= PORT_PCR_MUX(1);
    LED_BLUE_PORT  |= PORT_PCR_MUX(1);
    // Switch the GPIO pins to output mode
    GPIOB_PDDR |= ((1UL << LED_RED_PIN) | (1UL << LED_BLUE_PIN));
    GPIOE_PDDR |= (1UL << LED_GREEN_PIN);
    // Turn off the LEDs
    GPIOB_PDOR = (1UL << LED_RED_PIN) | (1UL << LED_BLUE_PIN);
    GPIOE_PDOR = (1UL << LED_GREEN_PIN);
}

// led_on - turns specified LED pins on
//   input: int color - RGB colors to enable from MSB to LSB.
//                       eg. if color == 3 (3'b011), the Green
//                       and Blue LED's are turned on.
void led_on(int color) {
    // Turn off all LEDs
	led_off(LED_WHITE);
    // Red LED
    if (color & 4)
    GPIOB_PCOR |= (1UL << LED_RED_PIN);
    // Green LED
    if (color & 2)
    GPIOE_PCOR |= (1UL << LED_GREEN_PIN);
    // Blue LED
    if (color & 1)
    GPIOB_PCOR |= (1UL << LED_BLUE_PIN);
}
// led_off - turns specified LED pins off
//   input: int color - RGB colors to disable from MSB to LSB.
//                       eg. if color == 3 (3'b011), the Green
//                       and Blue LED's are turned on.
void led_off(int color) {
    // Red LED
    if (color & 4)
    GPIOB_PSOR |= (1UL << LED_RED_PIN);
    // Green LED
    if (color & 2)
    GPIOE_PSOR |= (1UL << LED_GREEN_PIN);
    // Blue LED
    if (color & 1)
    GPIOB_PSOR |= (1UL << LED_BLUE_PIN);
}
