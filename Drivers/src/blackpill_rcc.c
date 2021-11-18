/*
 * backpill_rcc.c
 *
 *  Created on: 8 Nov. 2021
 *      Author: Gaurav
 */


#include "blackpill_rcc.h"

 int AHB_Prescalar_arr[9] = {2,4,8,16,32,64,128,256,512};
 int APB1_Prescalar_arr[4] = {1,4,8,16};

uint32_t APB1_CLK_Freq_Calculate(void); // usart2
uint32_t APB2_CLK_Freq_Calculate(void); // usart1,6


uint32_t APB1_CLK_Freq_Calculate(void){
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

		temp = ((RCC->RCC_CFGR >> 4) & 0XF); // prescalar for AHB
		if(temp <8)
			{
			prescalarahb = 1;
			}
		else if(temp >= 8)
			{
			prescalarahb = AHB_Prescalar_arr[temp-8];
			}
		temp = ((RCC->RCC_CFGR >> 10) & 0XF);// prescalar for APB
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
/*
 *
 *
 */
uint32_t APB2_CLK_Freq_Calculate(void){
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

		temp = ((RCC->RCC_CFGR >> 4) & 0XF); // prescalar for AHB
		if(temp <8)
			{
			prescalarahb = 1;
			}
		else if(temp >= 8)
			{
			prescalarahb = AHB_Prescalar_arr[temp-8];
			}
		temp = ((RCC->RCC_CFGR >> 13) & 0X7);// prescalar for APB
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
