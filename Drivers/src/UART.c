/*
 * UART.c
 *
 *  Created on: 7 Nov. 2021
 *      Author: Gaurav
 */


#include "UART.h"
#include "blackpill_rcc.h"

void USART_Clock_Control(USART_Reg_Def *pUSART , uint8_t SOR); //DONE
void USART_PeripheralControl(USART_Reg_Def *pUSARTx, uint8_t Cmd); //DONE

void USART_Init(USART_Handle_t *pUSARTHandle); //DONE
void USART_Deinit(USART_Reg_Def *pUSARTx); // done

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len); // DONE
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len); //DONE
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len); //DONE
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);//DONE

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); //done
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);//done
void USART_IRQHandling(USART_Handle_t pHandle);

void USART_PeripheralControl(USART_Reg_Def *pUSARTx, uint8_t EnOrDi); //done
uint8_t USART_GetFlagStatus(USART_Reg_Def *pUSARTx , uint32_t FlagName); //DONE
void USART_ClearFlag(USART_Reg_Def *pUSARTx, uint16_t StatusFlagName);//done

void USART_SetBaudRate(USART_Reg_Def *pUSARTx, uint32_t BaudRate); //done


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
		USART_SetBaudRate(pUSARTHandle->pUSARTx ,pUSARTHandle->USART_Config.USART_baud);

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

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->RxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->USART_CR3 |=(1<< 7); //TXEIE BIT SET


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->USART_CR3 |=(1<< 6);


	}

	return txstate;

}


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

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->USART_CR3 |=(1<< 5 ); //RXNEIE BIT IS SET

	}

	return rxstate;

}
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
/*
 * flag clear API
 */
void USART_ClearFlag(USART_Reg_Def *pUSARTx, uint16_t StatusFlagName){
	pUSARTx->USART_SR &= ~(1<< StatusFlagName);
}

/*
 * @ BAUD RATE FUNCTION
 *
 *
 */
void USART_SetBaudRate(USART_Reg_Def *pUSARTx, uint32_t BaudRate){

	// to store the system clock
	uint32_t fclk =0;
	//final value
	uint32_t usartval =0;
	//mantissa part and floating part
	uint32_t M,F;
	//
	uint32_t tempreg =0;
	//usarts available on APB2
	if((pUSARTx == USART1) & (pUSARTx == USART6)){

		fclk = APB2_CLK_Freq_Calculate();
	} else {
		fclk = APB1_CLK_Freq_Calculate();
	}
	//  check for OVER8 bit
	if(pUSARTx->USART_CR1 & (1<<15)){
		usartval = (25*fclk)/(2*BaudRate); // formulae is reference manual
	}else{
		usartval = (25*fclk)/(4*BaudRate);
	}

	M = usartval/100; // we get the mantissa
	tempreg |= (M << 4);
	F = usartval - (M * 100);
	// calculate the fractional value
	if(pUSARTx->USART_CR1 & (1<<15)){
		// ovesampling 8
		 F = ((( F * 8)+ 50) / 100)& ((uint8_t)0x07);
	}else{
		 F = ((( F * 16)+ 50) / 100)& ((uint8_t)0x07);
	}
	tempreg |= F;
	pUSARTx->USART_BRR |=(tempreg);

}
/*
 * Interrupt handling for UART.
 *
 */
void USART_IRQHandling(USART_Handle_t pHandle){
	 //Implement the code to check the state of TC bit in the SR
	uint32_t temp1=0,temp2=0;
		temp1 = pHandle.pUSARTx->USART_SR & ( 1 << 6);

		 //Implement the code to check the state of TCIE bit
		temp2 = pHandle.pUSARTx->USART_CR1 & ( 1 << 6);

		if(temp1 && temp2 )
		{
			//this interrupt is because of TC

			//close transmission and call application callback if TxLen is zero
		if ( pHandle.TxBusyState == USART_BUSY_IN_TX)
			{
				//Check the TxLen . If it is zero then close the data transmission
				if(!(pHandle.TxLen) )
				{
					//Implement the code to clear the TC flag
					pHandle.pUSARTx->USART_SR &= ~( 1 << 6);

					//Implement the code to clear the TCIE control bit

					//Reset the application state
					pHandle.TxBusyState = USART_READY;

					//Reset Buffer address to NULL
					pHandle.pTxBuffer = 0;

					//Reset the length to zero
					pHandle.TxLen =0;

					//Call the applicaton call back with event USART_EVENT_TX_CMPLT
	/* TODO:this*/	USART_ApplicationEventCallback(&pHandle,USART_EVENT_TX_CMPLT);
				}
			}
		}

	/*************************Check for TXE flag ********************************************/

		//Implement the code to check the state of TXE bit in the SR
		uint16_t *pdata=0;
		temp1 = pHandle.pUSARTx->USART_SR & (1 << 7);

		//Implement the code to check the state of TXEIE bit in CR1
		temp2 = pHandle.pUSARTx->USART_CR1 & (1<< 7);

		if(temp1 && temp2 )
		{
			//this interrupt is because of TXE

			if(pHandle.TxBusyState == USART_BUSY_IN_TX)
			{
				//Keep sending data until Txlen reaches to zero
				if(pHandle.TxLen > 0)
				{
					//Check the USART_WordLength item for 9BIT or 8BIT in a frame
					if(pHandle.USART_Config.USART_wordlength == USART_WORDLEN_9BITS)
					{
						//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
						pdata = (uint16_t*) pHandle.pTxBuffer;

						//loading only first 9 bits , so we have to mask with the value 0x01FF
						pHandle.pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

						//check for USART_ParityControl
						if(pHandle.USART_Config.USART_paritybit == USART_PARITY_DISABLE)
						{
							//No parity is used in this transfer , so, 9bits of user data will be sent
							//Implement the code to increment pTxBuffer twice
							pHandle.pTxBuffer++;
							pHandle.pTxBuffer++;

							//Implement the code to decrement the length
							pHandle.TxLen-= 2;
						}
						else
						{
							//Parity bit is used in this transfer . so , 8bits of user data will be sent
							//The 9th bit will be replaced by parity bit by the hardware
							pHandle.pTxBuffer++;

							//Implement the code to decrement the length
							pHandle.TxLen-= 1;
						}
					}
					else
					{
						//This is 8bit data transfer
						pHandle.pUSARTx->USART_DR = (*pdata & (uint8_t)0xFF);

						//Implement the code to increment the buffer address
						pHandle.pTxBuffer++;

						//Implement the code to decrement the length
						pHandle.TxLen --;
					}

				}
				if (pHandle.TxLen == 0 )
				{
					//TxLen is zero
					//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
					pHandle.pUSARTx->USART_CR1 &=  ~(1<< 7);
				}
			}
		}

	/*************************Check for RXNE flag ********************************************/

		temp1 = pHandle.pUSARTx->USART_SR & ( 1 << 5); // RXNE CHECK BIT
		temp2 = pHandle.pUSARTx->USART_CR1 & ( 1 << 5);//USART_CR1_RXNEIE BIT

		if(temp1 && temp2 )
		{
			//this interrupt is because of rxne
			//this interrupt is because of txe
			if(pHandle.RxBusyState == USART_BUSY_IN_RX)
			{
				//TXE is set so send data
				if(pHandle.RxLen > 0)
				{
					//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
					if(pHandle.USART_Config.USART_wordlength == USART_WORDLEN_9BITS)
					{
						//We are going to receive 9bit data in a frame

						//Now, check are we using USART_ParityControl control or not
						if(pHandle.USART_Config.USART_paritybit == USART_PARITY_DISABLE)
						{
							//No parity is used. so, all 9bits will be of user data

							//read only first 9 bits so mask the DR with 0x01FF
							*((uint16_t*) pHandle.pRxBuffer) = (pHandle.pUSARTx->USART_DR  & (uint16_t)0x01FF);

							//Now increment the pRxBuffer two times
							pHandle.pRxBuffer++;
							pHandle.pRxBuffer++;

							//Implement the code to decrement the length
							pHandle.RxLen =- 2;
						}
						else
						{
							//Parity is used. so, 8bits will be of user data and 1 bit is parity
							 *pHandle.pRxBuffer = (pHandle.pUSARTx->USART_DR  & (uint8_t)0xFF);

							 //Now increment the pRxBuffer
							 pHandle.pRxBuffer++;

							 //Implement the code to decrement the length
							 pHandle.RxLen --;
						}
					}
					else
					{
						//We are going to receive 8bit data in a frame

						//Now, check are we using USART_ParityControl control or not
						if(pHandle.USART_Config.USART_paritybit == USART_PARITY_DISABLE)
						{
							//No parity is used , so all 8bits will be of user data

							//read 8 bits from DR
							 *pHandle.pRxBuffer = (uint8_t) (pHandle.pUSARTx->USART_DR  & (uint8_t)0xFF);
						}

						else
						{
							//Parity is used, so , 7 bits will be of user data and 1 bit is parity

							//read only 7 bits , hence mask the DR with 0X7F
							 *pHandle.pRxBuffer = (uint8_t) (pHandle.pUSARTx->USART_DR  & (uint8_t)0x7F);

						}

						//Now , increment the pRxBuffer
						pHandle.pRxBuffer++;

						//Implement the code to decrement the length
						pHandle.RxLen --;
					}


				}//if of >0

				if(! pHandle.RxLen)
				{
					//disable the rxne
					pHandle.pUSARTx->USART_CR1 &= ~( 1 << 5 );
					pHandle.RxBusyState = USART_READY;
/* TODO : THIS*/	USART_ApplicationEventCallback(&pHandle,USART_EVENT_RX_CMPLT);
				}
			}
		}


	/*************************Check for CTS flag ********************************************/
	//Note : CTS feature is not applicable for UART4 and UART5

		//Implement the code to check the status of CTS bit in the SR

		//Implement the code to check the state of CTSE bit in CR1
		temp1 = pHandle.pUSARTx->USART_CR3 & ( 1 << 9);

		//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
		temp2 = pHandle.pUSARTx->USART_CR3 & ( 1 << 9);


		if(temp1  && temp2 )
		{
			//Implement the code to clear the CTS flag in SR
			pHandle.pUSARTx->USART_CR3 &= ~( 1 << 9);

			//this interrupt is because of cts
/*todo :this*/USART_ApplicationEventCallback(&pHandle,USART_EVENT_CTS);
		}

	/*************************Check for IDLE detection flag ********************************************/

		//Implement the code to check the status of IDLE flag bit in the SR
		pHandle.pUSARTx->USART_SR &= ~(1<<4);

		//Implement the code to check the state of IDLEIE bit in CR1
		temp2 = pHandle.pUSARTx->USART_CR1 & ( 1 << 4);


		if(temp1 && temp2)
		{
			//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
			pHandle.pUSARTx->USART_CR1 &= ~( 1 << 4);
			//this interrupt is because of idle
			USART_ApplicationEventCallback(&pHandle,USART_EVENT_IDLE);/*TODO: THIS*/
		}

	/*************************Check for Overrun detection flag ********************************************/

		//Implement the code to check the status of ORE flag  in the SR
		temp1 = pHandle.pUSARTx->USART_SR & (1<<3);

		//Implement the code to check the status of RXNEIE  bit in the CR1
		temp2 = pHandle.pUSARTx->USART_CR1 & (1<<5);


		if(temp1  && temp2 )
		{
			//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

			//this interrupt is because of Overrun error
			USART_ApplicationEventCallback(&pHandle,USART_EVENT_ORE); /*TODO: THIS*/
		}



	/*************************Check for Error Flag ********************************************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used.

		temp2 =  pHandle.pUSARTx->USART_CR3 & ( 1 << 0) ;

		if(temp2)
		{
			temp1 = pHandle.pUSARTx->USART_SR;
			if(temp1 & ( 1 << 1))
			{
				/*
					This bit is set by hardware when a de-synchronization, excessive noise or a break character
					is detected. It is cleared by a software sequence (an read to the USART_SR register
					followed by a read to the USART_DR register).
				*/
				USART_ApplicationEventCallback(&pHandle,USART_ERREVENT_FE); /*TODO:THIS*/
			}

			if(temp1 & ( 1 << 2) ) // CHECK THE NF FLAG
			{
				/*
					This bit is set by hardware when noise is detected on a received frame. It is cleared by a
					software sequence (an read to the USART_SR register followed by a read to the
					USART_DR register).
				*/
				USART_ApplicationEventCallback(&pHandle,USART_ERREVENT_NE); /*TODO: THIS */
			}

			if(temp1 & ( 1 << 3) ) // ORE BIT IN SR
			{
				USART_ApplicationEventCallback(&pHandle,USART_ERREVENT_ORE); /*TODO : THIS*/
			}
		}


	}

