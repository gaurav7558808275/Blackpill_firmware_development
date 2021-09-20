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

uint8_t GPIO_Read_Pin(GPIO_Reg_Def *GPIO , uint8_t PinNumber);
uint16_t GPIO_Read_Port(GPIO_Reg_Def *GPIO);
void GPIO_Write_Pin(GPIO_Reg_Def *GPIO , uint8_t Value, uint8_t Pin);
void GPIO_Write_Port(GPIO_Reg_Def *GPIO, uint16_t Value);
void GPIO_Toggle_Pin(GPIO_Reg_Def *GPIO, uint8_t PinNumber);

/*
 *  INTERUPT HANDLING FUNCTION PROTOTYPE FOR GPIO
 */

void GPIO_IRQ_config(GPIO_Reg_Def *GPIO, uint8_t IRQPriority,uint8_t S_O_R);   // S_O_R IS SET OR RESET SHORTHAND USAGE
void GPIO_IRQ_Handling(uint8_t PinNumber);

void GPIO_Delay(uint8_t value);


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

}

void GPIO_Init(GPIOx_Handle *GPIO_Handle)
{

		// uint32_t temp;
		// configuration of gpio pin config

		if(GPIO_Handle->GPIO_Pin_config.GPIO_PinMode <= 3)
		{
			/* ANOTHER WAY BY USING THE TEMP FUNCTION */
			/*
			 * 	temp = GPIO_Handle->GPIO_Pin_config.GPIO_PinMode << ( 2 * GPIO_Handle->GPIO_Pin_config.GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
				GPIOx_Handle->pGPIOx.MODER = temp;
				temp=0;
			 */
			GPIO_Handle->pGPIOx->MODER &= ~(0X03 << GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber); // CLASSIC CLEAR
			GPIO_Handle->pGPIOx->MODER |= (GPIO_Handle->GPIO_Pin_config.GPIO_PinMode << ( 2 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber)); // LOGICAL SET
		}
		else
		{
				// interrupt pin initialisation

		}


		// speed of the pin initiated
		//GPIO_Handle->pGPIOx->OSPEEDR &= ~(0X03 << GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber ); // classic clear
		GPIO_Handle->pGPIOx->OSPEEDR |= (GPIO_Handle->GPIO_Pin_config.GPIO_PinSpeed << (2 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber));
		//  pull-up or down is necessary
		//GPIO_Handle->pGPIOx->PUPDR &= ~(0x03<<GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber); //classic clear
		GPIO_Handle->pGPIOx->PUPDR |=  (GPIO_Handle->GPIO_Pin_config.GPIO_PinPuPdcontrol <<(2 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber));
		// the outputtype
		//GPIO_Handle->pGPIOx->OTYPER = ~(0x01<<GPIO_Pin_config.GPIO_PinNumber ); classic clear
		GPIO_Handle->pGPIOx->OTYPER |= (GPIO_Handle->GPIO_Pin_config.GPIO_PinOutputType << (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber));
		// alternate function
		if(GPIO_Handle->GPIO_Pin_config.GPIO_AltFunctionMode == GPIO_MODE_ALT)
		{
			//ALTERNATE PIN SETUP
			// INCASE YOU FORGET THE MULTIPLICATION FACTOR DEPENDS ON THE REGISTER MAPPING PER PIN
			if(GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber <= 7)
			{
				GPIO_Handle->pGPIOx->AFRL = ~(0x0F<<GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber );
				GPIO_Handle->pGPIOx->AFRL |= GPIO_Handle->GPIO_Pin_config.GPIO_AltFunctionMode << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
			}
			else if(GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber > 7)
			{
				GPIO_Handle->pGPIOx->AFRH = ~(0x0F<<GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber );
				GPIO_Handle->pGPIOx->AFRH |= GPIO_Handle->GPIO_Pin_config.GPIO_AltFunctionMode <<(4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
			}
		}

}
void GPIO_DeInit(GPIO_Reg_Def *GPIO)
{
	if(GPIO == GPIOA)
	{
		GPIOA_CLK_RESET();
	}
	else if(GPIO == GPIOB)
	{
		GPIOB_CLK_RESET();
	}
	else if(GPIO == GPIOC)
	{
		GPIOC_CLK_RESET();
	}
	else if(GPIO == GPIOD)
	{
		GPIOD_CLK_RESET();
	}
	else if(GPIO == GPIOE)
	{
		GPIOE_CLK_RESET();
	}
	else if(GPIO == GPIOH)
	{
		GPIOH_CLK_RESET();
	}
}

uint8_t GPIO_Read_Pin(GPIO_Reg_Def *GPIO , uint8_t PinNumber)
{
	uint8_t data = 0;
	data = (GPIO->IDR>>PinNumber) & (0x00000001);
	return data;
}

uint16_t GPIO_Read_Port(GPIO_Reg_Def *GPIO)
{
		uint16_t data = 0;
		data = (GPIO->IDR);
		return data;
}
void GPIO_Write_Pin(GPIO_Reg_Def *GPIO , uint8_t Value, uint8_t Pin)
{
	if(Value ==GPIO_PIN_SET)
		{GPIO->ODR |= 1 << Pin;}
	else if(Value == GPIO_PIN_RESET)
		{GPIO->ODR &=(1 << Pin);}
}
void GPIO_Write_Port(GPIO_Reg_Def *GPIO, uint16_t Value)
{
	GPIO->ODR |= Value;
}
void GPIO_Toggle_Pin(GPIO_Reg_Def *GPIO, uint8_t PinNumber)
{
	GPIO->ODR ^= (1 <<PinNumber);
}
void GPIO_Delay(uint8_t value)
{
	value = value*1000;
	for(volatile int i=0;i<= value;i++)
	{

	}
}
void GPIO_IRQ_config(GPIO_Reg_Def *GPIO, uint8_t IRQPriority,uint8_t S_O_R)
{
	//TODO  : IMPLEMETATION
}
void GPIO_IRQ_Handling(uint8_t PinNumber)
{
	//TODO  : IMPLEMETATION
}

