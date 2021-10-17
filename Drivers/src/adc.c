/*
 * adc.c
 *
 *  Created on: 16 Oct. 2021
 *      Author: Gaurav
 */


#include "adc.h"



void adc_init();
void adc_start();
uint16_t adc_convert();



void adc_init()
{
	GPIOx_Handle gpio;
	//ADC_Handle g
	RCC->RCC_APB2ENR |= (1<<8);   //CLOCK FOR ADC1

	//GPIO_Clock_EN(GPIOA, ENABLE);
	gpio.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_5;
	gpio.GPIO_Pin_config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_ANL; // ADC1 PIN IS GPIOA
	gpio.pGPIOx = GPIOA;
	GPIO_Init(&gpio);

	ADC1->ADC_CR1 &=~(3<<24); // ADC RESOLUTION  IN ADC1_CR1
	ADC1->ADC_CR1 &= ~(1<<8);  // SCAN MODE CONVERTED IN ADC1_CR1 DISABLED
	ADC1->ADC_CR1 &=~(1<<12);  // DISCONTNIOUS MODE FOR INJECTED CHANNELS DISBLAED
	ADC1->ADC_CR1 |=0x00; // DISCONTINOUS CAHNNELS SELECTED 1 channel
	ADC1->ADC_CR1 &=~(1<<11); //Discontinuous mode on injected channels DISABLED

	ADC1->ADC_CR2 &= ~(1<<1); // ADC BIT[1] 0-SINGLE CONVERSION 1- CONTINIOUS CONVERSION CONT_BIT
	ADC1->ADC_CR2 &= ~(1<<11| (1<<12)); //data alignment bit[11]
	ADC1->ADC_CR2 &=~(1<<28); // EXTERNAL TRIGGER disabled
	ADC1->ADC_CR2 |=(1<<10); //SET END OF CONVERSION BIT

	// CHANNEL SAMPLE TIME
	ADC1->ADC_SMRP2 &= ~(3<<0);  // 3 cycles sample time
	// SETUP THE CHANNEL
	ADC1->ADC_SQR1 &= ~(15<<20);// NUMBER OF CONVERSION 1
}

void adc_start()
{


	ADC1->ADC_CR2 &= (1<<0); // OFF ADON BIT
	ADC1->ADC_CR2 |= (1<<0); // ADON IS 1
	us_delay(2);  // delay to stabilise

	// CLEAR FLAGS
	ADC1->ADC_SR &= ~(1<<1); // CLEAR EOC BIT
	ADC1->ADC_CR1 |=(1<<1);



}

uint16_t adc_convert()
{
	uint16_t data;
	ADC1->ADC_CR2 |= (1<<30); // START CONVERSION

	while(!((ADC1->ADC_SR) & (1<<1)));
	data = ADC1->ADC_DR;
	return data;
}





