/*
 * timer.c
 *
 *  Created on: 6 Oct. 2021
 *      Author: Gaurav
 */


#include "timer.h"
#include "blackpill.h"

/*
 * For Configuring the clock settings in STM32 blue chip board we need to do following things
 * 1) Enable the HSE and wait for HSE to become ready.
 * 2) Configure Flash PreFetch and the latency related settings.
 * 3) Configure the Clock for Buses(AHB,APB1,APB2).
 * 4) Configure the PLL(multiplier,HSE divider).
 * 5) Enable the PLL source and wait for it to become stable
 * 6) Enable the Clock source and wait for it to be set
 */


void system_clk();

void system_clk()
{
	RCC->RCC_CR |= 1<<16; // We will use HSE. HSE bit set
	while(!(RCC->RCC_CR & (1 << 17))); 	// HSE ready bit check condition
	// flash memory settings
	//1. latency is set to 1-BIT [0:4]
	//2.PRFTEN is set to 1 bit[8]
	FLASH->FLASH_ACR |= ((1<<8)	| (1<<0));

	/* activating PLL
	 * BIT[22] is set to 1 for HSE clock
	 * BIT[22] is set to zero for HSI clock
	 */
	RCC->RCC_PLLCFGR |= (1<<22);
	/*
	 * setting PLLM according to the clock in CUBEmx
	 * according to the prescalar /25
	 */
	RCC->RCC_PLLCFGR |= (25 << 0);
	/*
	 *  PLLN setting
	 *  /128 IN PUSHED
	 *
	 */
	RCC->RCC_PLLCFGR |= (128<<6);
	/*
	 *PLLP SETTING
	 *PLLP /2 IS PUSHED
	 */
	RCC->RCC_PLLCFGR &= ~(1<<16);
	// TURN ON PLL
	RCC->RCC_CR |= (1<<24);
	while(RCC->RCC_CR &(1<<25)); // WAIT TILL PLLRDY BIT IS 1
	// activating sysCLK
	RCC->RCC_CFGR |= (2<<0);
	while(!(RCC->RCC_CFGR & (2<<1)));




}
