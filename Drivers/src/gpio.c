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

void GPIO_IRQ_IT_config(uint8_t IRQ_Number,uint8_t S_O_R);   // S_O_R IS SET OR RESET SHORTHAND USAGE
void GPIO_IRQ_Handling(uint8_t PinNumber);
void GPIO_Priority_Config(uint8_t IRQ_number , uint32_t priority);


void GPIO_Delay(uint32_t value);


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
	else if(S_O_R == DISABLE)
	{
		if(GPIO == GPIOA)
			{
				GPIOA_CLK_DI();
			}
		else if(GPIO == GPIOB)
			{
				GPIOA_CLK_DI();
			}
	}
}

void GPIO_Init(GPIOx_Handle *GPIO_Handle)
{
	// clock enable for respective peripheral
	GPIO_Clock_EN(GPIO_Handle->pGPIOx,ENABLE);

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
		if(GPIO_Handle->GPIO_Pin_config.GPIO_PinMode  == GPIO_MODE_IT_FT)
	{
				// interrupt pin initialisation FALLING TRIGGER
		EXTI->EXTI_FTSR |= (1<< GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
		EXTI->EXTI_RTSR &= ~(1<< GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);

	}
	else if(GPIO_Handle->GPIO_Pin_config.GPIO_PinMode  == GPIO_MODE_IT_RT)
	{
				//GPIO_PIN SETUP FOR RISING TRIGGER
		EXTI->EXTI_FTSR &= ~(1<< GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
		EXTI->EXTI_RTSR |= (1<< GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
	}
	else if(GPIO_Handle->GPIO_Pin_config.GPIO_PinMode  == GPIO_MODE_IT_RFT)
	{
				// GPIO PIN SETUP FOR RISING AND FALLING TRIGGER
		EXTI->EXTI_FTSR |= (1<< GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
		EXTI->EXTI_RTSR |= (1<< GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
	}

	SYSCFG_CLK_EN();			//SYSCONFIG CLOCK INITIALISATION MACRO

/*____________________________________// CONFIGURE THE SYSCONFIG PORT SELECTOR__________________________________________________________________________________*/
	if(GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber <=3)
		{
			if(GPIO_Handle->pGPIOx == GPIOA)
				{
					SYSCFG->SYSCFG_EXTICR1 |= (GPIO_SYSCFG_MODE_A << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber));
				}
				else if(GPIO_Handle->pGPIOx == GPIOB)
				{
					SYSCFG->SYSCFG_EXTICR1 |= (GPIO_SYSCFG_MODE_B << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber)) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOC)
				{
					SYSCFG->SYSCFG_EXTICR1 |= (GPIO_SYSCFG_MODE_C << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber)) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOD)
				{
					SYSCFG->SYSCFG_EXTICR1 |= (GPIO_SYSCFG_MODE_D << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber)) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOE)
				{
					SYSCFG->SYSCFG_EXTICR1 |= (GPIO_SYSCFG_MODE_E << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber)) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOH)
				{
					SYSCFG->SYSCFG_EXTICR1 |= (GPIO_SYSCFG_MODE_H << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber)) ;
				}
		}

			else if(GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber>3 && GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber <=7 )

			{
				if(GPIO_Handle->pGPIOx == GPIOA)
				{
					SYSCFG->SYSCFG_EXTICR2 |= (GPIO_SYSCFG_MODE_A << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber % 4)));
				}
				else if(GPIO_Handle->pGPIOx == GPIOB)
				{
					SYSCFG->SYSCFG_EXTICR2 |= (GPIO_SYSCFG_MODE_B << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%4))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOC)
				{
					SYSCFG->SYSCFG_EXTICR2 |= (GPIO_SYSCFG_MODE_C << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%4))) ;
				}
					else if(GPIO_Handle->pGPIOx == GPIOD)
				{
					SYSCFG->SYSCFG_EXTICR2 |= (GPIO_SYSCFG_MODE_D << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%4))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOE)
				{
					SYSCFG->SYSCFG_EXTICR2 |= (GPIO_SYSCFG_MODE_E << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%4))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOH)
				{
					SYSCFG->SYSCFG_EXTICR2 |= (GPIO_SYSCFG_MODE_H << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%4))) ;
				}
		  }

			else if(GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber>7 && GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber <=11)

			{
				if(GPIO_Handle->pGPIOx == GPIOA)
				{
					SYSCFG->SYSCFG_EXTICR3 |= (GPIO_SYSCFG_MODE_A << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%8))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOB)
				{
					SYSCFG->SYSCFG_EXTICR3 |= (GPIO_SYSCFG_MODE_B << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%8))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOC)
				{
					SYSCFG->SYSCFG_EXTICR3 |= (GPIO_SYSCFG_MODE_C << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%8))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOD)
				{
					SYSCFG->SYSCFG_EXTICR3 |= (GPIO_SYSCFG_MODE_D << ((4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%8))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOE)
				{
					SYSCFG->SYSCFG_EXTICR3 |= (GPIO_SYSCFG_MODE_E << ((4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%8))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOH)
				{
					SYSCFG->SYSCFG_EXTICR3 |= (GPIO_SYSCFG_MODE_H << ((4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%8))) ;
				}

			}

			else if(GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber>11 && GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber <=15)
			{
				if(GPIO_Handle->pGPIOx == GPIOA)
				{
					SYSCFG->SYSCFG_EXTICR4 |= (GPIO_SYSCFG_MODE_A << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%12))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOB)
				{
					SYSCFG->SYSCFG_EXTICR4 |= (GPIO_SYSCFG_MODE_B << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%12))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOC)
				{
					SYSCFG->SYSCFG_EXTICR4 |= (GPIO_SYSCFG_MODE_C << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%12))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOD)
				{
					SYSCFG->SYSCFG_EXTICR4 |= (GPIO_SYSCFG_MODE_D << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%12))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOE)
				{
					SYSCFG->SYSCFG_EXTICR4 |= (GPIO_SYSCFG_MODE_E << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%12))) ;
				}
				else if(GPIO_Handle->pGPIOx == GPIOH)
				{
					SYSCFG->SYSCFG_EXTICR4 |= (GPIO_SYSCFG_MODE_H << (4 * (GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber%12))) ;
				}
			}
	/*_______________________// INITLISAZE THE INTERRUPT OF EXTI ACCORDING TO PIN________________________________________________*/

			EXTI->EXTI_IMR |= (1<< GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);		// pushed that bit to interrupt mask register
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
				GPIO_Handle->pGPIOx->AFRL &= ~(0x0F<<GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber );
				GPIO_Handle->pGPIOx->AFRL |= GPIO_Handle->GPIO_Pin_config.GPIO_AltFunctionMode << (4 * GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber);
			}
			else if(GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber > 7)
			{
				GPIO_Handle->pGPIOx->AFRH &= ~(0x0F<<GPIO_Handle->GPIO_Pin_config.GPIO_PinNumber );
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
	data = (GPIO->IDR >> PinNumber) & (0x00000001);
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
	if(Value == GPIO_PIN_SET)
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

void GPIO_Delay(uint32_t value)
{
	value = value*1000;
	for(volatile int i=0;i<= value;i++)
	{

	}
}

void GPIO_IRQ_IT_config(uint8_t IRQ_Number,uint8_t S_O_R)
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
		*NVIC_ICER0 |= (1 << IRQ_Number);
		}
		else if(IRQ_Number >31 && IRQ_Number <64) // 32-63
		{
		*NVIC_ICER1 |= (1 << (IRQ_Number % 32));
		}
		else if(IRQ_Number >64 && IRQ_Number <96)
		{
		*NVIC_ICER2 |= (1 << (IRQ_Number % 64));
		}
	}


}

void GPIO_Priority_Config(uint8_t IRQ_number , uint32_t priority)
{
	uint8_t iprx = IRQ_number/8;   	// TO FIND THE WHICH REGISTER OF PRI
	uint8_t iprx_section = IRQ_number % 4;		// THIS IS USED TO FIND THE SECTION THAT THE DTAA WILL BE UPDATED
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_BIT_IMPLEMENTED);	// IN THE IPR LAST 4 BITS ARE INGNORED

	*(NVIC_IPR0 + iprx) |= (priority << (shift_amount)); // SETS ALL THE 8 BIT REGISTER TO THE REQUIRED VALUE(PRIORITY)


}
void GPIO_IRQ_Handling(uint8_t PinNumber)
{
	if(EXTI->EXTI_PR & (1<< PinNumber))
	{
		EXTI->EXTI_PR |=(1<< PinNumber);
	}
}

