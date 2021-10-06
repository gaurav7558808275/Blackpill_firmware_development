/*
 * spi.h
 *
 *  Created on: 24 Sep. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_SPI_H_
#define DRIVERS_INC_SPI_H_

#include "blackpill.h"


	// Possible Application states
#define SPI_READY			0
#define	SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

#define SPI_EVENTTX_COMPLETED	3
#define SPI_EVENTRX_COMPLETED	4
#define SPI_ERROR_EVENT_OCCURED	5

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
	uint8_t	 	 *tx_buff;  // needed for pointing tx buffer in interrupt mode
	uint8_t 	 *rx_buff; // needed for pointing rx buffer during interrupt mode
	uint32_t	 txlen;		// length of the data  that is going ton be send
	uint32_t	 rxlen;		// length of the data received
	uint8_t		 txrdy;		// status of tx state
	uint8_t		 rxrdy;		// !< to store the state of rx >

}SPI_Handle_t;

		// Function Prototypes

void SPI_Clock_EN(SPI_RegDef_t *pSPIx);	// SPI Clock Initialize
void SPI_Clock_DE(SPI_RegDef_t *pSPIx);	// SPI clock deinit

void SPI_Init(SPI_Handle_t *SPI_Handle);
void SPI_Deinit(SPI_Handle_t *SPI_Handle);

void SPI_Send(SPI_RegDef_t * pSPIx, uint8_t *tx_buff , uint32_t length);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *rx_buff , uint32_t length);

uint8_t SPI_Send_IT(SPI_Handle_t * pSPI_Handle_t, uint8_t *tx_buff , uint32_t length);
uint8_t SPI_Receive_IT(SPI_Handle_t *pSPI_Handle_t, uint8_t *rx_buff , uint32_t length);

void SPI_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);  // SET OR RESET
void SPI_IRQ_Handling(SPI_Handle_t *pSPI_Handle_t );
void SPI_Priority_Config(uint8_t IRQ_number , uint32_t priority);

void SPI_Peripheral_Control(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R);
void SPI_SSI_Enable(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R);
void SPI_SSOE_Enable(SPI_RegDef_t *SPI_Handle, uint8_t S_O_R);
uint8_t SPI_BusyFlag(SPI_RegDef_t *SPI_Handle);

void Clear_OVRflag(SPI_Handle_t *SPI_Handle);
void Close_Transmission(SPI_Handle_t *SPI_Handle);
void Close_Reception(SPI_Handle_t *SPI_Handle);
__weak__ void SPI_EVENTCALLBACK_FUNCTION(SPI_Handle_t *pSPI_Handle_t, uint8_t Appev); // only activated when called in the main function




#endif /* DRIVERS_INC_SPI_H_ */
