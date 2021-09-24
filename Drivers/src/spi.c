/*
 * spi.c
 *
 *  Created on: 24 Sep. 2021
 *      Author: Gaurav
 */

#include "spi.h"

void SPI_Clock_EN(SPI_RegDef_t *pSPIx);	// SPI Clock Initialize
void SPI_Clock_DE(SPI_RegDef_t *pSPIx);	// SPI clock deinit

void SPI_Init(SPI_Handle_t *SPI_Handle);
void SPI_Deinit(SPI_Handle_t *SPI_Handle);

void SPI_Send(SPI_RegDef_t pSPIx, uint8_t *tx_buff , uint32_t length);
void SPI_Receive(SPI_RegDef_t pSPIx, uint8_t *rx_buff , uint32_t length);

void SPI_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);  // SET OR RESET
void SPI_IRQ_Handling(SPI_Handle_t *P_handle);
void SPI_Priority_Config(uint8_t IRQ_number , uint32_t priority);


 	 	 // function declaration

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
	else if(SPI_Handle->SPI_Config.SPI_SSM == SPI_SSM_DI)
		{
			SPI_Handle->pSPIx->SPI_CR1 |= (1<<9);
		}
}




