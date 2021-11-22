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
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3
/*
 * Flag definitions
 *
 */
#define USART_FLAG_TXE 		(1<<7)
#define USART_FLAG_RXNE		(1<<5)
#define USART_FLAG_TC		(1<<6)
/*
 * Txrx states
 *
 */
#define USART_BUSY_IN_TX		1
#define USART_BUSY_IN_RX		2
#define USART_READY				0
/*
 * CALLBACK FUNCTONS MACRO
 *
 */
#define USART_EVENT_TX_CMPLT 0
#define USART_EVENT_RX_CMPLT 1
#define USART_EVENT_CTS		 2
#define USART_EVENT_IDLE     3
#define USART_EVENT_ORE      4
#define USART_ERREVENT_FE	 5
#define USART_ERREVENT_NE	 6
#define USART_ERREVENT_ORE	 7
/*
 * USART Configure structure
 *
 */
typedef struct{
	uint8_t USART_mode;
	uint32_t USART_baud;
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
	// for interrupt based API
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;

}USART_Handle_t;

/*
 * Peripheral clock setup
 */
/*
 * USART_Clock_Control() API
 * pUSART -> Register handle init.
 * SOR ->  ENABLE or DISABLE
 */
void USART_Clock_Control(USART_Reg_Def *pUSART , uint8_t SOR);
/*
 * puRPOSE : set and reset the UE bit ; USART_CR1 |= (1<< 13)
 * pUSARTx ->  the handle created
 * CMD -> ENABLE or DISABLE
 *
 */
void USART_PeripheralControl(USART_Reg_Def *pUSARTx, uint8_t Cmd);
/*
 * USART init and deinit
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_Deinit(USART_Reg_Def *pUSARTx);
/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_Reg_Def *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_Reg_Def *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_Reg_Def *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);









#endif /* DRIVERS_INC_UART_H_ */
