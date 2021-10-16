/*
 * adc.c
 *
 *  Created on: 16 Oct. 2021
 *      Author: Gaurav
 */


#include "adc.h"
#include "gpio.h"

void adc_init();
void adc_start();
uint16_t adc_convert();


void adc_init()
{
	GPIOx_Handle gpio;
	RCC->RCC_APB2ENR |= (1<<8);   //CLOCK FOR ADC1

	//GPIO_Clock_EN(GPIOA, ENABLE);
	gpio.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_5;
	gpio.GPIO_Pin_config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_ANL; // ADC1 PIN IS GPIOA
	gpio.pGPIOx = GPIOA;
	GPIO_Init();

	ADC1->ADC_CR1 &=~(3<<12); // ADC RESOLUTION  IN ADC1_CR1
	ADC1->ADC_CR1 &= ~(1<<8);  // SCAN MODE CONVERTED IN ADC1_CR1 DISABLED
	ADC1->ADC_CR1 &=~(1<<12);  // DISCONTNIOUS MODE FOR INJECTED CHANNELS DISBLAED
	ADC1->ADC_CR1 &=~(1<<)

	ADC1->ADC_CR2 &= ~(1<<1); // ADC BIT[1] 0-SINGLE CONVERSION 1- CONTINIOUS CONVERSION
	ADC1->ADC_CR2 &= ~(1<<11| (1<<12)); //data alignment bit[11]
	ADC1->ADC_CR2 |=(0<<24 | 1<<24 | 2<<24); // EXTERNAL TRIGGER TODO: IDK WHAT THIS IS








	// SET PRESCALAR FOR ADC



}
