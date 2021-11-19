/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdlib.h>
#include "blackpill.h"
#include "gpio.h"
#include "spi.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
    /* Loop forever */
/*
	GPIOx_Handle gpio_led, button;
	gpio_led.pGPIOx = GPIOC;
	gpio_led.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_13;
	gpio_led.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_Pin_config.GPIO_PinSpeed = GPIO_SPEED_MED;
	gpio_led.GPIO_Pin_config.GPIO_PinOutputType = GPIO_MODE_OUT_PP;

	GPIO_Clock_EN(gpio_led.pGPIOx ,ENABLE);
	GPIO_Init(&gpio_led);
*/
	GPIOx_Handle button;
	button.pGPIOx = GPIOA;
	button.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_12;
	button.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_IT_FT;
	button.GPIO_Pin_config.GPIO_PinPuPdcontrol = GPIO_PORT_UP;
	button.GPIO_Pin_config.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_Clock_EN(button.pGPIOx , ENABLE);
	GPIO_Init(&button);

	GPIO_IRQ_IT_config(IRQ_N_EXTI15_10, ENABLE);
	//GPIO_IRQ_Handling(12);
	GPIO_Priority_Config(IRQ_N_EXTI15_10, 3);

	while(1)
	{
		//GPIO_Toggle_Pin(GPIOC, GPIO_PIN_13);
		//GPIO_Delay(200);
	}


}
void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQ_Handling(12); // calling the handling function to set the pending register
	GPIO_Toggle_Pin(GPIOC, GPIO_PIN_13); // THE ACTIVITY WHEN INTERRUPT IS FIRED.
}
