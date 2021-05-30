/*
Title:       uart.c
Description: Provide UART routines for serial IO
Authors:     Chris Abajian (cxa6282@rit.edu)
Date:        9/09/2020
*/

#include "MK64F12.h"
#include "uart.h"

void uart0_init() {
	//define variables for baud rate and baud rate fine adjust
	uint16_t ubd, brfa;

	//Enable clock for UART
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	//Configure the port control register to alternative 3 (which is UART mode for K64)
	UART0_RX   |= PORT_PCR_MUX(3);
	UART0_TX   |= PORT_PCR_MUX(3);

	/*Configure the UART for establishing serial communication*/

	//Disable transmitter and receiver until proper settings are chosen for the UART module
	UART0_C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);

	//Select default transmission/reception settings for serial communication of UART by clearing the control register 1
	UART0_C1 = 0x00;

	//UART Baud rate is calculated by: baud rate = UART module clock / (16 ? (SBR[12:0] + BRFD))
	//13 bits of SBR are shared by the 8 bits of UART0_BDL and the lower 5 bits of UART0_BDH 
	//BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
	//BRFA is defined by the lower 4 bits of control register, UART0_C4 

	//calculate baud rate settings: ubd = UART module clock/16* baud rate
	ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

	//clear SBR bits of BDH
	UART0_BDL &= ~(UART_BDL_SBR_MASK);
	UART0_BDH &= ~(UART_BDH_SBR_MASK);

	//distribute this ubd in BDH and BDL
	UART0_BDL |= UART_BDL_SBR(ubd);
	UART0_BDH |= UART_BDH_SBR(ubd >> 8);

	//BRFD = (1/32)*BRFA 
	//make the baud rate closer to the desired value by using BRFA
	brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

	//write the value of brfa in UART0_C4
	UART0_C4 &= ~(UART_C4_BRFA_MASK);
	UART0_C4 |= UART_C4_BRFA(brfa);
		
	//Enable transmitter and receiver of UART
	UART0_C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);
}
void uart3_init() {
	//define variables for baud rate and baud rate fine adjust
	uint16_t ubd, brfa;

	//Enable clock for UART
	SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

	//Configure the port control register to alternative 3 (which is UART mode for K64)
	UART3_RX   |= PORT_PCR_MUX(3);
	UART3_TX   |= PORT_PCR_MUX(3);

	/*Configure the UART for establishing serial communication*/

	//Disable transmitter and receiver until proper settings are chosen for the UART module
	UART3_C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);

	//Select default transmission/reception settings for serial communication of UART by clearing the control register 1
	UART3_C1 = 0x00;

	//UART Baud rate is calculated by: baud rate = UART module clock / (16 ? (SBR[12:0] + BRFD))
	//13 bits of SBR are shared by the 8 bits of UART0_BDL and the lower 5 bits of UART0_BDH 
	//BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
	//BRFA is defined by the lower 4 bits of control register, UART0_C4 

	//calculate baud rate settings: ubd = UART module clock/16* baud rate
	ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));

	//clear SBR bits of BDH
	UART3_BDL &= ~(UART_BDL_SBR_MASK);
	UART3_BDH &= ~(UART_BDH_SBR_MASK);

	//distribute this ubd in BDH and BDL
	UART3_BDL |= UART_BDL_SBR(ubd);
	UART3_BDH |= UART_BDH_SBR(ubd >> 8);

	//BRFD = (1/32)*BRFA 
	//make the baud rate closer to the desired value by using BRFA
	brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

	//write the value of brfa in UART3_C4
	UART3_C4 &= ~(UART_C4_BRFA_MASK);
	UART3_C4 |= UART_C4_BRFA(brfa);
		
	//Enable transmitter and receiver of UART
	UART3_C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);
}

uint8_t uart0_getchar() {
	/* Wait until there is space for more data in the receiver buffer*/
	while(!(UART0_S1 & UART_S1_RDRF_MASK)) {}

	/* Return the 8-bit data from the receiver */
	return UART0_D;
}
uint8_t uart3_getchar() {
	/* Wait until there is space for more data in the receiver buffer*/
	while(!(UART3_S1 & UART_S1_RDRF_MASK)) {}

	/* Return the 8-bit data from the receiver */
	return UART3_D;
}

void uart0_putchar(char ch) {
	/* Wait until transmission of previous bit is complete */
	while(!(UART0_S1 & UART_S1_TDRE_MASK)) {}
	
	/* Send the character */
	UART0_D = ch;
}
void uart3_putchar(char ch) {
	/* Wait until transmission of previous bit is complete */
	while(!(UART3_S1 & UART_S1_TDRE_MASK)) {}
	
	/* Send the character */
	UART3_D = ch;
}

void uart0_put(char *ptr_str) {
	/*use putchar to print string*/
	while(*ptr_str) {
		uart0_putchar(*ptr_str++);
	}
}
void uart3_put(char *ptr_str) {
	/*use putchar to print string*/
	while(*ptr_str) {
		uart3_putchar(*ptr_str++);
	}
}

