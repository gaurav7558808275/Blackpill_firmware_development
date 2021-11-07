/*
 * UART.c
 *
 *  Created on: 7 Nov. 2021
 *      Author: Gaurav
 */


#include "UART.h"

void USART_clock_enbale(USART_Reg_Def *pUSART , uint8_t SOR);




















void USART_Clock_Control(USART_Reg_Def *pUSART , uint8_t SOR){

	if(SOR == ENABLE){
		if(pUSART == USART1 ){
			USART1_CLOCK_ENABLE();
		}else if(pUSART == USART2){
			USART2_CLOCK_ENABLE();
		}else if(pUSART == USART6){
			USART6_CLOCK_ENABLE();
		}
		else{
			if(SOR == DISABLE){
				if(pUSART == USART1 ){
				USART1_CLOCK_DISABLE();
				}else if(pUSART == USART2){
				USART2_CLOCK_DISABLE();
				}else if(pUSART == USART6){
				USART6_CLOCK_DISABLE();
				}
			  }
		   }
	}

}
