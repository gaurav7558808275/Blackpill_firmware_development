/*
 * gpio.c
 *
 *  Created on: 19 Sep. 2021
 *      Author: Gaurav
 */

#include "gpio.h"
/*
 *  CLOCK ENABLE FUNCTION PROTOYPE
 */

void GPIO_Clock_EN(GPIO_Reg_Def *GPIO , uint8_t S_O_R);

/*
 * GPIO SET AND RESET FUCNTION PROTOTYPE
 */

void GPIO_Init(GPIOx_Handle *GPIO_Handle);
void GPIO_DeInit(GPIO_Reg_Def *GPIO);

/*
 *  GPIO PORT WRITE AND READ FUNCTION PROTOTYPE
 */

void GPIO_Read_Pin(GPIO_Reg_Def *GPIO , uint8_t PinNumber);
void GPIO_Read_Port(GPIO_Reg_Def *GPIO);
void GPIO_Write_Pin(GPIO_Reg_Def *GPIO , uint16_t Value);
void GPIO_Write_Port(GPIO_Reg_Def *GPIO, uint16_t Value);
void Toggle_Pin(GPIO_Reg_Def *GPIO, uint8_t PinNumber);

/*
 *  INTERUPT HANDLING FUNCTION PROTOTYPE FOR GPIO
 */

void GPIO_IRQ_config(GPIO_Reg_Def *GPIO, uint8_t IRQPriority,uint8_t S_O_R);   // S_O_R IS SET OR RESET SHORTHAND USAGE
void GPIO_IRQ_Handling(uint8_t PinNumber);


/*
 * ALL FUNCTION DEFINITIONS
 */



void GPIO_Clock_EN(GPIO_Reg_Def *GPIO , uint8_t S_O_R)
{
	if(S_O_R == ENABLE)
	{
		if(GPIO == GPIOA)
		{
		GPIOA_CLK_EN();
		}
		else if(GPIO == GPIOB)
		{
		GPIOB_CLK_EN();
		}
		else if(GPIO == GPIOC)
		{
		GPIOC_CLK_EN();
		}
		else if(GPIO == GPIOD)
		{
		GPIOD_CLK_EN();
		}
		else if(GPIO == GPIOE)
		{
		GPIOE_CLK_EN();
		}
		else if(GPIO == GPIOH)
		{
		GPIOH_CLK_EN();
		}
	}
	else
	{
		if(GPIO == GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if(GPIO == GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if(GPIO == GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if(GPIO == GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if(GPIO == GPIOE)
		{
			GPIOE_CLK_DI();
		}
		else if(GPIO == GPIOH)
		{
			GPIOH_CLK_DI();
		}

	}
}

void GPIO_Init(GPIOx_Handle *GPIO_Handle)
{
		// configuration of gpio pin config
		if(GPIO_Handle->GPIO_Pin_config.GPIO_PinMode <= 3)
		{


		}

}
