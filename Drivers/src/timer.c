/*
 * timer.c
 *
 *  Created on: 6 Oct. 2021
 *      Author: Gaurav
 */


#include "timer.h"

void timer_init();
void us_delay(uint16_t us);
void ms_delay(uint16_t ms);
void s_delay(uint16_t s);

timer_init()
{
	TIMER_2->TIM2_CR1 &= ~(1<<0); //CEN disabled
	TIMER_2->TIM2_SR &=~(1<0);	  // UIF BIT disabled

	// Timer clock reset
	RCC->RCC_APB1RSTR |= (1<<0);
	RCC->RCC_APB1RSTR &= ~(1<<0);

	RCC->RCC_APB1ENR |= (1<<0); // Clock enable

	TIMER_2->TIM2_CR1 |= (1<<1);  // UDIS SET, UPDATE EVENT NOT GENERATED
/************************ SETUP FOR PSC AND ARR *********/
	TIMER_2->TIM2_PSC |=0;   // ACORDING TO CALCULATION
	TIMER_2->TIM2_ARR |=32;	 // ACCORDING TO CALCULATION

	TIMER_2->TIM2_CR1 &= ~(1<<1); // UDIS RESET, UPDATE EVENT GENERATED
	TIMER_2->TIM2_EGR |= (1<<0);  // reload back the counter check the EGR register
	TIMER_2->TIM2_SR &= ~(1<<0); // no update flag

}

us_delay(uint16_t us)
{
	TIMER_2->TIM2_CR1 |=(1<<0); // TIMER ON
	TIMER_2->TIM2_CNT |= 0X00000000; // RESET COUNTER
	while(TIMER_2->TIM2_CNT < us)
	{
		;
	}
}

void ms_delay(uint16_t ms)
{
	volatile int i=0;
	for(i=0;i<ms;i++)
	{
		us_delay(1000);
	}

}
void s_delay(uint16_t s)
{
	volatile int i=0;
		for(i=0;i<s;i++)
		{
			ms_delay(1000);
		}
}


