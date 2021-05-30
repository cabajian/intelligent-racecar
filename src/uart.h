/*
Title:       uart.h
Description: Provide UART function prototypes.
Authors:     Chris Abajian (cxa6282@rit.edu)
Date:        9/09/2020
*/

#if !defined(UART_H_)
	#define UART_H_ 	/**< Symbol preventing repeated inclusion */
    
	#define BAUD_RATE    9600        //default baud rate
	#define SYS_CLOCK    20485760    //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)
	#define UART0_RX     PORTB_PCR16
	#define UART0_TX     PORTB_PCR17
	#define UART3_RX     PORTC_PCR16
	#define UART3_TX     PORTC_PCR17
	
	/* UART0 Prototypes */
	void uart0_put(char *ptr_str);
	void uart0_init(void);
	uint8_t uart0_getchar(void);
	void uart0_putchar(char ch);

	/* UART3 Prototypes */
	void uart3_put(char *ptr_str);
	void uart3_init(void);
	uint8_t uart3_getchar(void);
	void uart3_putchar(char ch);

#endif  /* #if !defined(UART_H_) */
 
/* uart.h, eof. */
