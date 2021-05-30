/*
Title:       led.h
Description: Provide LED function prototypes.
Authors:     Chris Abajian (cxa6282@rit.edu)
Date:        9/11/2020
*/

#if !defined(LED_H_)
	#define LED_H_ 	/**< Symbol preventing repeated inclusion */
	
    // RGB LED Pin Definitions
    #define LED_RED_PORT    (PORTB_PCR22)    // Red RGB LED Port+Pin
    #define LED_RED_PIN     (22)             // Red Pin
    #define LED_RED         (4)
    #define LED_GREEN_PORT  (PORTE_PCR26)    // Green RGB LED Port+Pin
    #define LED_GREEN_PIN   (26)             // Green Pin
    #define LED_GREEN       (2)
    #define LED_BLUE_PORT   (PORTB_PCR21)    // Blue RGB LED Port+Pin
    #define LED_BLUE_PIN    (21)             // Blue Pin
    #define LED_BLUE        (1)
    #define LED_CYAN        (3)
    #define LED_MAGENTA     (5)
    #define LED_YELLOW      (6)
    #define LED_WHITE       (7)

 
	/* LED Prototypes */
    void led_init(void);
    void led_on(int color);
    void led_off(int color);

#endif  /* #if !defined(LED_H_) */
 
/* led.h, eof. */
