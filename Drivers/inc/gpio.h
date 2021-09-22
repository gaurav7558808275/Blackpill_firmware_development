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
#define GPIO_MODE_IT_RFT	15			// INTERUPT MODE INUT MODE RISING AND FALLIN TRIGGER
//GPIO AVAILABLE OUTPUT MODES
#define GPIO_MODE_OUT_PP	0			// OUTPUT PUHSPULL MODE
#define GPIO_MODE_OUT_OD	1			// OUTPUT OPEN DRAIN MODE
//GPIO AVAILABLE SPEED
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3
//GPIO PORT PUSH PULL MACROS
#define GPIO_PORT_N_PP		0			// GPIO PORT NO PUSH PULL
#define GPIO_PORT_UP		1			// GPIO PORT WITH PULL UP
#define GPIO_PORT_DN		2			// GPIO PORT WITH PULL DOWN



#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3 		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13 	13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15
#define GPIO_PIN_16		16

#define GPIO_SYSCFG_MODE_A		0
#define GPIO_SYSCFG_MODE_B		1
#define GPIO_SYSCFG_MODE_C		2
#define GPIO_SYSCFG_MODE_D		3
#define GPIO_SYSCFG_MODE_E		4
#define GPIO_SYSCFG_MODE_H		7
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

uint8_t GPIO_Read_Pin(GPIO_Reg_Def *GPIO , uint8_t PinNumber);
uint16_t GPIO_Read_Port(GPIO_Reg_Def *GPIO);
void GPIO_Write_Pin(GPIO_Reg_Def *GPIO , uint8_t Value, uint8_t Pin);
void GPIO_Write_Port(GPIO_Reg_Def *GPIO, uint16_t Value);
void GPIO_Toggle_Pin(GPIO_Reg_Def *GPIO, uint8_t PinNumber);

void GPIO_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);   // S_O_R IS SET OR RESET SHORTHAND USAGE
void GPIO_IRQ_Handling(uint8_t PinNumber);
void GPIO_Priority_Config(uint8_t number , uint32_t priority);
void GPIO_Delay(uint32_t value); // delay Function

#endif /* DRIVERS_INC_GPIO_H_ */
