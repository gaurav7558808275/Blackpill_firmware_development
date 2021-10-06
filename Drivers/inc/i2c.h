/*
 * i2c.h
 *
 *  Created on: 28 Sep. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_I2C_H_
#define DRIVERS_INC_I2C_H_

#include "blackpill.h"

/*
 *		SCL Speed Setup.
 *		Its 100KHz for std mode
 *		Its 400kHz for speed mode
 *
 */
#define I2C_SCL_SM_SPEED	100000
#define I2C_SCL_PM_SPEED	400000
/*
 * 	ACK CONTROL
 *
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * 	Duty control
 * 	DUTY: Fm mode duty cycle
	0: Fm mode tlow/thigh = 2
	1: Fm mode tlow/thigh = 16/9 (see CCR)
 *
 */
#define I2C_FMDUTY_2		0
#define I2C_FMDUTY16_9		1

/*
 * API PROTOTYPES.
 *
 */

typedef struct
{
	uint32_t	I2C_SCL_Speed;
	uint32_t	I2C_Device_Addr;
	uint32_t	I2C_ACKControl;
	uint32_t	I2C_FMduty;

}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;

}I2C_Handle_t;


void I2C_Clock_EN(I2C_RegDef_t *pI2Cx);	// I2C Clock Initialize
void I2C_Clock_DE(I2C_RegDef_t *pI2Cx);	// I2C clock deinit

void I2C_Init(I2C_Handle_t *I2C_Handle);
void I2C_Deinit(I2C_Handle_t *I2C_Handle);

void I2C_Send(I2C_RegDef_t * pI2Cx, uint8_t *tx_buff , uint32_t length);
void I2C_Receive(I2C_RegDef_t *pI2Cx, uint8_t *rx_buff , uint32_t length);

uint8_t I2C_Send_IT(I2C_Handle_t * pI2C_Handle_t, uint8_t *tx_buff , uint32_t length);
uint8_t I2C_Receive_IT(I2C_Handle_t *pI2C_Handle_t, uint8_t *rx_buff , uint32_t length);

void I2C_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);  // SET OR RESET
void I2C_IRQ_Handling(I2C_Handle_t *pI2C_Handle_t );
void I2C_Priority_Config(uint8_t IRQ_number , uint32_t priority);

void I2C_Peripheral_Control(I2C_RegDef_t *I2C_Handle, uint8_t S_O_R);
void I2C_SSI_Enable(I2C_RegDef_t *I2C_Handle, uint8_t S_O_R);
void I2C_SSOE_Enable(I2C_RegDef_t *I2C_Handle, uint8_t S_O_R);
uint8_t I2C_BusyFlag(I2C_RegDef_t *I2C_Handle);

void Clear_OVRflag(I2C_Handle_t *I2C_Handle);
void Close_Transmission(I2C_Handle_t *I2C_Handle);
void Close_Reception(I2C_Handle_t *I2C_Handle);

// Sub-functions
uint32_t CLK_Freq_calculate(void);

/*
 * DEFINE REQUIRED FOR @CLK_Freq_calculate 	FUNCTION
 */
#define CLK_HSI		16000000
#define CLK_HSE		8000000
#define CLK_PLL		2;  //TODO a fuunction can be created to calculate the PLL if selected






#endif /* DRIVERS_INC_I2C_H_ */
