/*
 * UART.c
 *
 *  Created on: 7 Nov. 2021
 *      Author: Gaurav
 */


#include "UART.h"

void USART_Clock_Control(USART_Reg_Def *pUSART , uint8_t SOR); //DONE
void USART_PeripheralControl(USART_Reg_Def *pUSARTx, uint8_t Cmd); //DONE

void USART_Init(USART_Handle_t *pUSARTHandle); //DONE
void USART_Deinit(USART_Reg_Def *pUSARTx); // done

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); //done
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);//done
void USART_IRQHandling(USART_Handle_t pHandle);

void USART_PeripheralControl(USART_Reg_Def *pUSARTx, uint8_t EnOrDi); //done
uint8_t USART_GetFlagStatus(USART_Reg_Def *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_Reg_Def *pUSARTx, uint16_t StatusFlagName);

/*
 *
 * Declarations
 *
 *
 */
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
void USART_PeripheralControl(USART_Reg_Def *pUSARTx, uint8_t Cmd){

	if(Cmd == ENABLE){
		pUSARTx->USART_CR1 |= (1<< 13); // UE bit to set start USART
	}else if(Cmd == DISABLE){
		pUSARTx->USART_CR1 &= ~(1<< 13);
	}
}


void USART_Init(USART_Handle_t *pUSARTHandle){

		//Temporary variable
		uint32_t tempreg=0;

	/******************************** Configuration of CR1******************************************/

		//Implement the code to enable the Clock for given USART peripheral
		USART_Clock_Control(pUSARTHandle->pUSARTx , ENABLE);
		//Enable USART Tx and Rx engines according to the USART_Mode configuration item
		if (pUSARTHandle->USART_Config.USART_mode == USART_MODE_ONLY_RX)
		{
			//Implement the code to enable the Receiver bit field
			tempreg|= (1 << 2);
		}else if (pUSARTHandle->USART_Config.USART_mode == USART_MODE_ONLY_TX)
		{
			//Implement the code to enable the Transmitter bit field
			tempreg |= ( 1 <<3 );

		}else if (pUSARTHandle->USART_Config.USART_mode == USART_MODE_TXRX)
		{
			//Implement the code to enable the both Transmitter and Receiver bit fields
			tempreg |= ( ( 1 << 2) | ( 1 << 3) );
		}

	    //Implement the code to configure the Word length configuration item
		tempreg |= pUSARTHandle->USART_Config.USART_wordlength << 12 ;


	    //Configuration of parity control bit fields
		if ( pUSARTHandle->USART_Config.USART_paritybit == USART_PARITY_EN_EVEN)
		{
			//Implement the code to enable the parity control
			tempreg |= ( 1 << 10);

			//Implement the code to enable EVEN parity
			tempreg &=~(1<<9);
			//Not required because by default EVEN parity will be selected once you enable the parity control

		}else if (pUSARTHandle->USART_Config.USART_paritybit == USART_PARITY_EN_ODD )
		{
			//Implement the code to enable the parity control
		    tempreg |= ( 1 << 10);

		    //Implement the code to enable ODD parity
		    tempreg |= ( 1 << 9);

		}

	   //Program the CR1 register
		pUSARTHandle->pUSARTx->USART_CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

		tempreg=0;

		//Implement the code to configure the number of stop bits inserted during USART frame transmission
		tempreg |= pUSARTHandle->USART_Config.USART_no_ofstopbits << 12;

		//Program the CR2 register
		pUSARTHandle->pUSARTx->USART_CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

		tempreg=0;

		//Configuration of USART hardware flow control
		if ( pUSARTHandle->USART_Config.USART_HWcontrol == USART_HW_FLOW_CTRL_CTS)
		{
			//Implement the code to enable CTS flow control
			tempreg |= ( 1 << 9);


		}else if (pUSARTHandle->USART_Config.USART_HWcontrol == USART_HW_FLOW_CTRL_RTS)
		{
			//Implement the code to enable RTS flow control
			tempreg |= ( 1 << 8);

		}else if (pUSARTHandle->USART_Config.USART_HWcontrol == USART_HW_FLOW_CTRL_CTS_RTS)
		{
			//Implement the code to enable both CTS and RTS Flow control
			tempreg |= ((1<<8)|(1<<9));
		}


		pUSARTHandle->pUSARTx->USART_CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

		//Implement the code to configure the baud rate
		//We will cover this in the lecture. No action required here

	}
/*
 *
 *
 *
 */
void USART_Deinit(USART_Reg_Def *pUSARTx){
	if(pUSARTx == USART1){
	USART_Clock_Control(USART1, DISABLE);
	}
	else if(pUSARTx == USART2){
	USART_Clock_Control(USART1, DISABLE);
	}
	else if(pUSARTx == USART6){
	USART_Clock_Control(USART1, DISABLE);
	}
}
/*
 *
 *
 *
 *
 */
void USART_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t SOR){
	if ( SOR == ENABLE)
			{
				if(IRQ_Number <= 31)
				{
				*NVIC_ISER0 |= (1 << IRQ_Number);
				}
				else if(IRQ_Number >31 && IRQ_Number <64) // 32-63
				{
				*NVIC_ISER1 |= (1 << (IRQ_Number % 32));
				}
				else if(IRQ_Number >64 && IRQ_Number <96)
				{
				*NVIC_ISER2 |= (1 << (IRQ_Number % 64));
				}
			}
			else
			{
				if(IRQ_Number <= 31)
				{
				*NVIC_ICER0 &= ~(1 << IRQ_Number);
				}
				else if(IRQ_Number >31 && IRQ_Number <64) // 32-63
				{
				*NVIC_ICER1 &= ~(1 << (IRQ_Number % 32));
				}
				else if(IRQ_Number >64 && IRQ_Number <96)
				{
				*NVIC_ICER2 &= ~(1 << (IRQ_Number % 64));
				}
			}
}

void USART_IRQPriorityConfig(uint8_t IRQ_number, uint32_t priority){
	uint8_t iprx = IRQ_number/4;   	// TO FIND THE WHICH REGISTER OF PRI
	uint8_t iprx_section = IRQ_number % 4;		// THIS IS USED TO FIND THE SECTION THAT THE DTAA WILL BE UPDATED
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_BIT_IMPLEMENTED);	// IN THE IPR LAST 4 BITS ARE INGNORED
	*(NVIC_IPR0 + iprx) |= (priority << (shift_amount)); // SETS ALL THE 8 BIT REGISTER TO THE REQUIRED VALUE(PRIORITY)
}


/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_wordlength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_paritybit == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->USART_DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE)))

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_wordlength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_paritybit == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_paritybit == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
/*
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TODO;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TODO = Len;
		pUSARTHandle->pTxBuffer = TODO;
		pUSARTHandle->TxBusyState = TODO;

		//Implement the code to enable interrupt for TXE
		TODO


		//Implement the code to enable interrupt for TC
		TODO


	}

	return txstate;

}*/


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
/*
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->TODO;

	if(rxstate != TODO)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = TODO;
		pUSARTHandle->RxBusyState = TODO;

		//Implement the code to enable interrupt for RXNE
		TODO

	}

	return rxstate;

} */
/*
 *
 */
uint8_t USART_GetFlagStatus(USART_Reg_Def *pUSARTx , uint32_t FlagName){
	if(pUSARTx->USART_SR & FlagName){
		return SET;
	}
	else{
		return RESET;
	}
}
