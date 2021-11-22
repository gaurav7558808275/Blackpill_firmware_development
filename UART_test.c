/*
 * UART_test.c
 *
 *  Created on: 22 Nov. 2021
 *      Author: Gaurav
 */


#include "string.h"
#include "blackpill.h"
#include "UART.h"

// handle init
USART_Handle_t Uart;
char msg[1024] = "Hello, the rest is ready";
// creation of UART init

void uart_init(void){
		Uart.pUSARTx = USART6;
		Uart.USART_Config.USART_HWcontrol = USART_HW_FLOW_CTRL_NONE;
		Uart.USART_Config.USART_baud = USART_STD_BAUD_9600;
		Uart.USART_Config.USART_mode = USART_MODE_ONLY_TX;
		Uart.USART_Config.USART_no_ofstopbits = USART_STOPBITS_1;
		Uart.USART_Config.USART_paritybit = USART_PARITY_DISABLE;
		Uart.USART_Config.USART_wordlength = USART_WORDLEN_8BITS;
	}



int main(void){


	// Main UART_INIT()
	uart_init();
	USART_Init(&Uart);



	while(1){

	}

	return 0;
}
