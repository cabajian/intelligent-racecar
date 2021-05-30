/*
Title:       buttons.h
Description: Provide button function prototypes.
Authors:     Chris Abajian (cxa6282@rit.edu)
Date:        11/2/2020
*/

#if !defined(BUTTON_H_)
	#define BUTTON_H_ 	/**< Symbol preventing repeated inclusion */
	
    // Pin definitions
    #define SW_2       (PORTC_PCR6)     // Push Button Switch 2 Port+Pin
    #define SW_2_PIN   (6)              // Push Button Switch 2 Pin
    #define SW_3       (PORTA_PCR4)     // Push Button Switch 3 Port+Pin
    #define SW_3_PIN   (4)              // Push Button Switch 3 Pin
 
	/* Prototypes */
    void init_buttons(void);
    int is_sw2_pressed(void);
    int is_sw3_pressed(void);

#endif  /* #if !defined(BUTTON_H_) */
 
/* led.h, eof. */
