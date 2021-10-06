/*
 * i2c.c
 *
 *  Created on: 28 Sep. 2021
 *      Author: Gaurav
 */

#include "i2c.h"
#include "blackpill.h"

uint32_t AHB_Prescalar_arr[9] = {2,4,8,16,32,64,128,256,512};
uint32_t APB1_Prescalar_arr[4] = {1,4,8,16};



void I2C_Clock_EN(I2C_RegDef_t *pI2Cx);	// I2C Clock Initialize
void I2C_Clock_DE(I2C_RegDef_t *pI2Cx);
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



void I2C_Clock_EN(I2C_RegDef_t *pI2Cx )	// I2C Clock Initialize
{
	if(pI2Cx == I2C1)
	{
		I2C1_CLOCK_EN();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_CLOCK_EN();
	}
	else if(pI2Cx == I2C2)
	{
		I2C3_CLOCK_EN();
	}
	else
	{
		I2C1_CLOCK_DI();
		I2C2_CLOCK_DI();
		I2C3_CLOCK_DI();
	}

}
void I2C_Clock_DE(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
		{
			I2C1_CLOCK_DI();
		}
	else if(pI2Cx == I2C2)
		{
			I2C2_CLOCK_DI();
		}
	else if(pI2Cx == I2C2)
		{
			I2C3_CLOCK_DI();
		}

}

uint32_t CLK_Freq_Calculate(void)
{
	uint8_t pcklk = 0,prescalarahb =0 ,prescalarapb1 =0;
	uint32_t clksrc = 0,temp = 0;
	temp = ((RCC->RCC_CFGR >> 2) & 0x03);
	if(temp == 0)
		{
		clksrc = CLK_HSI;
		}
	else if(temp == 1)
		{
		clksrc = CLK_HSE;
		}
	else if(temp == 2)
		{
		clksrc = CLK_PLL;
		}

	temp = ((RCC->RCC_CFGR >>7) & 0XF); // prescalar for AHB
	if(temp <8)
		{
		prescalarahb = 1;
		}
	else if(temp >= 8)
		{
		prescalarahb = AHB_Prescalar_arr[temp-8];
		}
	temp = ((RCC->RCC_CFGR >>12) & 0XF);// prescalar for APB
	if(temp <4)
		{
			prescalarapb1 = 1;
		}
	else if(temp >= 4)
		{
			prescalarapb1 = APB1_Prescalar_arr[temp-4];
		}


	pcklk =  (clksrc/prescalarahb) / prescalarapb1; // formulae for calculation of clock.
	return pcklk;
}

void I2C_Init(I2C_Handle_t *I2C_Handle)
{
	// 1) taking the ACK enable.from CR1
	uint32_t tempreg = 0;
	tempreg |= I2C_Handle->I2C_Config.I2C_ACKControl << 10;  // ACK bit read

	// configure the FREQ assign
	tempreg = 0;
	tempreg = CLK_Freq_Calculate() / 1000000UL;  // need only the value
	I2C_Handle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);


}
