/*
 * spi.h
 *
 *  Created on: 24 Sep. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_SPI_H_
#define DRIVERS_INC_SPI_H_

#include "blackpill.h"



typedef struct
{
	uint32_t	SPI_MODE;
	uint32_t	SPI_Bus_Config;
	uint32_t	SPI_Clk_Speed;
	uint32_t	SPI_Dff;
	uint32_t	SPI_CPOL;
	uint32_t 	SPI_CPHA;
	uint32_t	SPI_SSM;

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;

}SPI_Handle_t;

		// Function Prototypes

void SPI_Clock_EN(SPI_RegDef_t *pSPIx);	// SPI Clock Initialize
void SPI_Clock_DE(SPI_RegDef_t *pSPIx);	// SPI clock deinit

void SPI_Init(SPI_Handle_t *SPI_Handle);
void SPI_Deinit(SPI_Handle_t *SPI_Handle);

void SPI_Send(SPI_RegDef_t * pSPIx, uint8_t *tx_buff , uint32_t length);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *rx_buff , uint32_t length);

void SPI_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);  // SET OR RESET
void SPI_IRQ_Handling(SPI_Handle_t *P_handle);
void SPI_Priority_Config(uint8_t IRQ_number , uint32_t priority);

void SPI_Peripheral_Control(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R);


#endif /* DRIVERS_INC_SPI_H_ */
