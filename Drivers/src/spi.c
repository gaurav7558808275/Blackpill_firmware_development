/*
 * spi.c
 *
 *  Created on: 24 Sep. 2021
 *      Author: Gaurav
 */

#include "spi.h"
#include "stddef.h"

/*
 *
 *
 *
 */


void SPI_Clock_EN(SPI_RegDef_t *pSPIx);	// SPI Clock Initialize
void SPI_Clock_DE(SPI_RegDef_t *pSPIx);	// SPI clock deinit

void SPI_Init(SPI_Handle_t *SPI_Handle);
void SPI_Deinit(SPI_Handle_t *SPI_Handle);

void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *tx_buff , uint32_t length);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *rx_buff , uint32_t length);


			//register control API's
void SPI_Peripheral_Control(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R); // function for activating the SPE register in CR1 register
void SPI_SSI_Enable(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R); // ssi bit handling Function
void SPI_SSOE_Enable(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R);	// SSO

		// API's required for Interrupt mode

uint8_t SPI_Send_IT(SPI_Handle_t * pSPI_Handle_t, uint8_t *tx_buff , uint32_t length);
uint8_t SPI_Receive_IT(SPI_Handle_t *pSPI_Handle_t, uint8_t *rx_buff , uint32_t length);
void SPI_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);  // SET OR RESET
void SPI_IRQ_Handling(SPI_Handle_t *pSPI_Handle_t );
void SPI_Priority_Config(uint8_t IRQ_number , uint32_t priority);
void Close_Transmission(SPI_Handle_t *pSPI_Handle_t);
void Close_Reception(SPI_Handle_t *pSPI_Handle_t);
void Clear_OVRflag(SPI_Handle_t *SPI_Handle);
static void spi_handle_rx_(SPI_Handle_t *pSPI_Handle_t);
static void spi_handle_tx_(SPI_Handle_t *pSPI_Handle_t);
static void spi_ovr_error_handle(SPI_Handle_t *pSPI_Handle_t);

/*
 * // function declaration
 *
 */


void SPI_Clock_EN(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_Clock_Init();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_Clock_Init();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_Clock_Init();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_Clock_Init();
	}
	else if(pSPIx== SPI5)
	{
		SPI5_Clock_Init();
	}

}
void SPI_Clock_DE(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_Clock_De();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_Clock_De();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_Clock_De();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_Clock_De();
	}
	else if(pSPIx== SPI5)
	{
		SPI5_Clock_De();
	}

}

void SPI_Init(SPI_Handle_t *SPI_Handle)

{
	SPI_Clock_EN(SPI_Handle->pSPIx);
			// SPI_Mode check
	if(SPI_Handle->SPI_Config.SPI_MODE == SPI_MODE_MASTER )
		{
		SPI_Handle->pSPIx->SPI_CR1 |= (1<<2);
		}
	else if(SPI_Handle->SPI_Config.SPI_MODE == SPI_MODE_SLAVE) // optional condition as the register will be in-reset condition
		{
		SPI_Handle->pSPIx->SPI_CR1 &= ~(1<<2);
		}
				// SPI_Bus_config check

	if(SPI_Handle->SPI_Config.SPI_Bus_Config == SPI_BUS_CONFIG_FD)
		{
		SPI_Handle->pSPIx->SPI_CR1 &= ~(1<<15);
		}
	else if(SPI_Handle->SPI_Config.SPI_Bus_Config == SPI_BUS_CONFIG_HD)
		{
		SPI_Handle->pSPIx->SPI_CR1 |= (1<<15);
		}
	else if(SPI_Handle->SPI_Config.SPI_Bus_Config == SPI_BUS_CONFIG_SIMPLEX_RX)
		{
		//SPI_Handle->pSPIx->SPI_CR1 |= (1<<14);
		SPI_Handle->pSPIx->SPI_CR1 &= ~(1<<15);
		SPI_Handle->pSPIx->SPI_CR1 |= (1<<10);
		}
	else if(SPI_Handle->SPI_Config.SPI_Bus_Config == SPI_BUS_CONFIG_SIMPLEX_TX)
		{
		SPI_Handle->pSPIx->SPI_CR1 |= (1<<15);
		SPI_Handle->pSPIx->SPI_CR1 |= (1<<14);
		}
				//SPI_Clk_Speed check
	if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV2)
		{
		SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV2 << 3);
		}
	else if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV4)
		{
		SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV4 << 3);
		}
	else if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV8)
		{
		SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV8 << 3);
		}
	else if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV16)
		{
		SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV16 << 3);
		}
	else if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV32)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV32 << 3);
		}
	else if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV64)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV64 << 3);
		}
	else if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV128)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV128 << 3);
		}
	else if(SPI_Handle->SPI_Config.SPI_Clk_Speed == SPI_CLOCK_SPEED_DIV256)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (SPI_CLOCK_SPEED_DIV256 << 3);
		}
					// SPI_Dff check
	if(SPI_Handle->SPI_Config.SPI_Dff == SPI_DFF_8BIT)
		{
		SPI_Handle->pSPIx->SPI_CR1 &= ~(1<<11);
		}
	else if(SPI_Handle->SPI_Config.SPI_Dff == SPI_DFF_16BIT)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (1<<11);
		}
					// SPI_CPOL check
	if(SPI_Handle->SPI_Config.SPI_CPOL == SPI_CPOL_0)
		{
		SPI_Handle->pSPIx->SPI_CR1 &= ~(1<<1);
		}
	else if(SPI_Handle->SPI_Config.SPI_CPOL == SPI_CPOL_1)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (1<<1);
		}
					// SPI_CPHA check
	if(SPI_Handle->SPI_Config.SPI_CPHA == SPI_CPHA_0)
		{
		SPI_Handle->pSPIx->SPI_CR1 &= ~(1<<0);
		}
	else if(SPI_Handle->SPI_Config.SPI_CPHA == SPI_CPHA_1)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (1<<0);
		}
					//SPI_SSM check
	if(SPI_Handle->SPI_Config.SPI_SSM == SPI_SSM_DI)
		{
		SPI_Handle->pSPIx->SPI_CR1 &= ~(1<<9);
		}
	else if(SPI_Handle->SPI_Config.SPI_SSM == SPI_SSM_EN)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (1<<9);
		}
}

 void SPI_Deinit(SPI_Handle_t *SPI_Handle)
{
	 if(SPI_Handle->pSPIx == SPI1)
	 	 {
		 SPI1_Clock_Reset();
	 	 }
	 else if(SPI_Handle->pSPIx == SPI2)
		 {
			 SPI2_Clock_Reset();
		 }
	 else if(SPI_Handle->pSPIx == SPI3)
	 	 {
	 		SPI3_Clock_Reset();
	 	 }
	 else if(SPI_Handle->pSPIx == SPI4)
	 	 {
	 		SPI4_Clock_Reset();
	 	 }
	 else if(SPI_Handle->pSPIx == SPI5)
	 	 {
	 		SPI5_Clock_Reset();
	 	 }
}
/********************************************************************
 *  	Send data with polling method.
 *
 ********************************************************************/
 void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *tx_buff , uint32_t length)
 {
	 //uint32_t TX_mask = 0x0001;
	 while(length > 0)
	 {
		 // check the tx status
		 while(!(pSPIx->SPI_SR & (1<<1)));   //TODO implement using function
		 // check the dff bit field
		 if(pSPIx->SPI_CR1 & (1<11))
		 {
			 // load 16 bit data
			 pSPIx->SPI_DR = *((uint16_t *)tx_buff);
			 length--;
			 length--;
			 (uint16_t*)tx_buff++;
		 }
		 else
		 {
			 	 // load 8 bit data
			 pSPIx->SPI_DR = *tx_buff;
			 length --;
			 tx_buff++;
		 }
	 }
 }

 /****************************************************
  *  Data receive using polling method.
  *
  ****************************************************/

 void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *rx_buff , uint32_t length)
 {
	 while(length < 0)
	 {
		 /*
		  * Here 0 tells RX buffer empty , 1 rx buffer not empty. get out
		 	 of while loop when 1
		  */
		 while(!(pSPIx->SPI_SR & (1<<0)));
		 if(pSPIx->SPI_CR1 & (1<11))
		 {
			*((uint16_t *)rx_buff) = pSPIx->SPI_DR;
			length--;
			length--;
			(uint16_t *) rx_buff++ ;   // typecasting required.
		 }
		 else
		 {
			*((uint16_t *)rx_buff) = pSPIx->SPI_DR;
			length--;
			rx_buff++;
		 }

	 }
 }

 /********************************************************************
  * Sets the SPE bit SPI enable bit in CR1.
  * This bit is very necessary to start the SPI Bus
  *
  ********************************************************************/
 void SPI_Peripheral_Control(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R)
 {
	 if(S_O_R == ENABLE)
	 {
		 SPI_Handle->SPI_CR1 |= (1 << 6);
	 }
	 else
	 {
		 SPI_Handle->SPI_CR1 &= ~(1 << 6);
	 }

 }

 /*******************************************************************************************************************
 	 *  ENABLING THE SSI BIT IN CR1 REGISTER TELLS THAT INTERNAL SLAVE SELECT HAS BEEN SET OR NOT
 	 *
 	 *  this is because if we have selected the SSM bit as 1 and when SSM bit is set to 1,
 	 *  it has an effect on SSI bit so We have to set SSI bit 0/1. This value will be pushed to
 	 *  the NSS pin
 	 *
 	 *  When we set the SSI pin to 1, we set the NSS pin to 1 thus saying there is no slave.
 	 *
 	 *This function san be used for saying whether there is a slave or not
 	 ***************************************************************************************************************/

 void SPI_SSI_Enable(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R)
 {
	 if(S_O_R == ENABLE)
	 	 {
		 	 SPI_Handle->SPI_CR1 |= (1<<8);
	 	 }
	 else
	 	 {
	 		 SPI_Handle->SPI_CR1 &= ~(1<<8);
	 	 }
 }
 /***********************************************************************************************
  * 0: SS output is disabled in master mode and the cell can work in multimaster configuration
	1: SS output is enabled in master mode and when the cell is enabled. The cell cannot work
	in a multimaster environment.

  ***********************************************************************************************/
void SPI_SSOE_Enable(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R)
{
	if(S_O_R == ENABLE)
	{
		SPI_Handle->SPI_CR2 |= (1<<2);
	}
	else
	{
		SPI_Handle->SPI_CR2 &= ~(1<<2);
	}
}
/************************************************************************************************
	 *  checks whether the SPI is busy or not. busy in the sense whether the data
	 *  is been sent or not
*************************************************************************************************/
uint8_t SPI_BusyFlag(SPI_RegDef_t *SPI_Handle)
{

	return(SPI_Handle->SPI_SR & (1<<7));
}
/************************************************************************************************
 * 	normal function to activate the NVIC
 *
 *
 ************************************************************************************************/

void SPI_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R)
{

	if (S_O_R == ENABLE)
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
/*
 * Function to send data using interrupt. it takes parameters like
 * @handle 		the handle for connecting with SPI structure
 * @Tx_buff		TXbuffer to push data
 * @length		1 or 2 That shows bytes.
 *
 * -> Checks whether the status of TX in structure is ready or not.
 * -> data is pushed to handler
 * -> Sets the interrupt bit
 * -> returns the value of the function and this will be utilised in a while loop in
 * application function. till then it waits.
 *
 */
uint8_t SPI_Send_IT(SPI_Handle_t * pSPI_Handle_t, uint8_t *tx_buff , uint32_t length)
{

	uint8_t state = pSPI_Handle_t->txrdy;
	if( state != SPI_BUSY_IN_TX)
		// 1. < set the Txt address and length to the handle
	{
		pSPI_Handle_t->tx_buff = tx_buff;
		pSPI_Handle_t->txlen= length;

		// 2. We will have to actiavte the Tx busy signal so no other application uses it
		pSPI_Handle_t->txrdy = SPI_BUSY_IN_TX;

		// 3. activate the CR2 -> TXEIE to activate the interrupt
		pSPI_Handle_t->pSPIx->SPI_CR2 |= (1<<7);  // TXEIE BIT SET

		// 4. data transmisiion done by IRQ. Control goes to SPI_IRQ_Handling() function

	}
	return state;
}
/*
 * parameters
 *
 */
uint8_t SPI_Receive_IT(SPI_Handle_t *pSPI_Handle_t, uint8_t *rx_buff , uint32_t length)
{
	uint8_t state = pSPI_Handle_t->rxrdy;
	if(state != SPI_BUSY_IN_RX)
	{
		 // 1. WE WILL UPDATE The address and length
		pSPI_Handle_t->rx_buff = rx_buff;
		pSPI_Handle_t->rxlen = length;
		// 2.we will initiate a rx busy
		pSPI_Handle_t->rxrdy = SPI_BUSY_IN_RX;
		// 3.we will activate the rx interrupt
		pSPI_Handle_t->pSPIx->SPI_CR2 |= (1<<6);
		// 4.IRQ for data movement is called.

	}
	return state;
}



void SPI_IRQ_Handling(SPI_Handle_t *pSPI_Handle_t )
{
	// check the Tx bit
	uint8_t temp1=0;
	uint8_t temp2=0;
		// first check TXE flag
	temp1 = (pSPI_Handle_t->pSPIx->SPI_SR) & (1<<1);  // checking the tx empty or not bit
	temp2 = (pSPI_Handle_t->pSPIx->SPI_CR2) & (1<<7); // Tx interrupt activated or not check

	if(temp2 && temp1)
	{
		spi_handle_tx_(pSPI_Handle_t);
	}

			// first check RXE flag
	temp1 = (pSPI_Handle_t->pSPIx->SPI_SR) & (1<<0);  // checking the rx not_empty or empty bit
	temp2 = (pSPI_Handle_t->pSPIx->SPI_CR2) & (1<<6); // rx interrupt activated or not check
	if(temp2 && temp1)
	{
			spi_handle_rx_(pSPI_Handle_t);
	}
		// check for OVR flag
	temp1 = (pSPI_Handle_t->pSPIx->SPI_SR) & (1<<6);
	temp2 = (pSPI_Handle_t->pSPIx->SPI_CR2) & (1<<5);
	if(temp1 && temp2)
	{
			spi_ovr_error_handle(pSPI_Handle_t);
	}

}
void Close_Transmission(SPI_Handle_t *pSPI_Handle_t)
{
		pSPI_Handle_t->pSPIx->SPI_CR2 &= ~(1<<7);   // interrupt cleared
		pSPI_Handle_t->tx_buff = NULL;
		pSPI_Handle_t->txlen = 0;
		pSPI_Handle_t->txrdy = SPI_READY;
}

void Close_Reception(SPI_Handle_t *pSPI_Handle_t)
{
	pSPI_Handle_t->pSPIx->SPI_CR2 &= ~(1<<6);
	pSPI_Handle_t->rx_buff = NULL;
	pSPI_Handle_t->rxlen = 0;
	pSPI_Handle_t->rxrdy = SPI_READY;
}

void Clear_OVRflag(SPI_Handle_t *SPI_Handle)
{
	uint16_t temp = 0;
	temp = SPI_Handle->pSPIx->SPI_DR;
	temp = SPI_Handle->pSPIx->SPI_SR;
	(void)temp;
}

__weak__ void SPI_EVENTCALLBACK_FUNCTION(SPI_Handle_t *pSPI_Handle_t, uint8_t Appev) // only activated when called in the main function
{
	/*
	 *  ANY INFORMATION TO BR PASSD TO THEMAIN FUNCTION
	 */

}

static void spi_handle_tx_(SPI_Handle_t *pSPI_Handle_t)
{


			 // check the dff bit field
		if(pSPI_Handle_t->pSPIx->SPI_CR1 & (1<11))
		{
				 // load 16 bit data
			pSPI_Handle_t->pSPIx->SPI_DR = *(uint16_t*)pSPI_Handle_t->tx_buff;
			pSPI_Handle_t->txlen--;
			pSPI_Handle_t->txlen--;
			(uint16_t*)pSPI_Handle_t->tx_buff++;
		}
		else
		{
				 	 // load 8 bit data
			 pSPI_Handle_t->pSPIx->SPI_DR =*(uint8_t*) pSPI_Handle_t->tx_buff;
			 pSPI_Handle_t->txlen--;
			 pSPI_Handle_t->tx_buff++;
		}
		if(!pSPI_Handle_t->txlen)
		{

			Close_Transmission(pSPI_Handle_t);
			// small callback function
			SPI_EVENTCALLBACK_FUNCTION(pSPI_Handle_t,SPI_EVENTTX_COMPLETED);


		}

}


static void spi_handle_rx_(SPI_Handle_t *pSPI_Handle_t)
{


	 if(pSPI_Handle_t->pSPIx->SPI_CR1 & (1<11))
		 {
			*((uint16_t *)pSPI_Handle_t->rx_buff) = pSPI_Handle_t->pSPIx->SPI_DR;
			pSPI_Handle_t->rxlen--;
			pSPI_Handle_t->rxlen--;
			(uint16_t *)pSPI_Handle_t->rx_buff++ ;   // typecasting required.
		 }
	else
		 {
			*(pSPI_Handle_t->rx_buff) = pSPI_Handle_t->pSPIx->SPI_DR;
			pSPI_Handle_t->rxlen--;
			pSPI_Handle_t->rx_buff++;
		 }
	 if(!(pSPI_Handle_t->rxlen))
	 	 {

		 Close_Reception(pSPI_Handle_t);
		 // SMALL CALLBACK FUNCTION
		 SPI_EVENTCALLBACK_FUNCTION(pSPI_Handle_t,SPI_EVENTRX_COMPLETED);

	 	 }

}

static void spi_ovr_error_handle(SPI_Handle_t *pSPI_Handle_t)
{
	// clear ovr flag
	/*
	 * to clear the ovr flag the data from DR should be fetched.
	 */
	uint8_t temp = 0;
	if(pSPI_Handle_t->txrdy !=SPI_BUSY_IN_RX)
	{
		temp = pSPI_Handle_t->pSPIx->SPI_DR;  // ovr clear procedures as per reference manual
		temp = pSPI_Handle_t->pSPIx->SPI_SR;
	}
	(void)temp;
		// Inform the application

}




