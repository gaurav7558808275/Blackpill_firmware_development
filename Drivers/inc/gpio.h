/*
 * gpio.h
 *
 *  Created on: 18 Sep. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_GPIO_H_
#define DRIVERS_INC_GPIO_H_

#include <blackpill.h>

// GPIO AVILABLE MODES
#define GPIO_MODE_IN		0			// INPUT MODE
#define GPIO_MODE_OUT		1			// OUTPUT MODE
#define GPIO_MODE_ALT		2			// ALTERNATE FUNCTION MODE
#define GPIO_MODE_ANL		3			// ANALOG MODE
// INPUT INTERRUPT MODE
#define GPIO_MODE_IT_RT		13			// INTERRUPTIN INPUT MODE RISING TRIGGER
#define GPIO_MODE_IT_FT		14			// INTERRUPT MODE INPUT MODE FALLIN TRIGGER
#define GPIO_MODE_IT_RF		15			// INTERUPT MODE INUT MODE RISING AND FALLIN TRIGGER
//GPIO AVAILABLE OUTPUT MODES
#define GPIO_MODE_OUT_PP	4			// OUTPUT PUHSPULL MODE
#define GPIO_MODE_OUT_OD	5			// OUTPUT OPEN DRAIN MODE
//GPIO AVAILABLE SPEED
#define GPIO_SPEED_LOW		6
#define GPIO_SPEED_MED		7
#define GPIO_SPEED_FAST		8
#define GPIO_SPEED_HIGH		9
//GPIO PORT PUSH PULL MACROS
#define GPIO_PORT_N_PP		10
#define GPIO_PORT_PP		11
#define GPIO_PORT_PPPD		12


/*
 * GPIO CONFIGURATION STRUCTURE
 */

typedef struct
{
	uint8_t		GPIO_PinNumber;
	uint8_t		GPIO_PinMode;
	uint8_t		GPIO_PinSpeed;
	uint8_t		GPIO_PinPuPdcontrol;
	uint8_t		GPIO_PinOutputType;
	uint8_t		GPIO_AltFunctionMode;

}GPIO_Config_t;

/*
 * GPIO HANDLER STRUCTURE
 */

typedef struct
{

	GPIO_Reg_Def	*pGPIOx;				// base address of the gpio according to usage
	GPIO_Config_t	GPIO_Pin_config;

}GPIOx_Handle;

/*
 * FUNCTON PROTOTYPES FOR GPIO
 */

void GPIO_Clock_EN(GPIO_Reg_Def *GPIO , uint8_t S_O_R);

void GPIO_Init(GPIOx_Handle *GPIO_Handle);
void GPIO_DeInit(GPIO_Reg_Def *GPIO);

void GPIO_Read_Pin(GPIO_Reg_Def *GPIO , uint8_t PinNumber);
void GPIO_Read_Port(GPIO_Reg_Def *GPIO);
void GPIO_Write_Pin(GPIO_Reg_Def *GPIO , uint16_t Value);
void GPIO_Write_Port(GPIO_Reg_Def *GPIO, uint16_t Value);
void Toggle_Pin(GPIO_Reg_Def *GPIO, uint8_t PinNumber);

void GPIO_IRQ_config(GPIO_Reg_Def *GPIO, uint8_t IRQPriority,uint8_t S_O_R);   // S_O_R IS SET OR RESET SHORTHAND USAGE
void GPIO_IRQ_Handling(uint8_t PinNumber);

#endif /* DRIVERS_INC_GPIO_H_ */
