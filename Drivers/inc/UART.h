/*
 * UART.H
 *
 *  Created on: 7 Nov. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_UART_H_
#define DRIVERS_INC_UART_H_

#include "blackpill.h"
#include "timer.h"


/*
 * Peripheral clock setup
 */
void USART_Clock_Control(USART_Reg_Def *pUSART , uint8_t SOR);






/*
 * USART Configure structure
 *
 */


typedef struct{
	uint8_t USART_mode;
	uint8_t USART_baud;
	uint8_t	USART_no_ofstopbits;
	uint8_t USART_wordlength;
	uint8_t USART_paritybit;
	uint8_t USART_HWcontrol;
}USART_Reg_Config;
/*
 *
 * USART handle structure
 *
 */
typedef struct{
	USART_Reg_Def  *pUSARTx;
	USART_Reg_Config USART_Config;

}USART_Handle_t;


#endif /* DRIVERS_INC_UART_H_ */
