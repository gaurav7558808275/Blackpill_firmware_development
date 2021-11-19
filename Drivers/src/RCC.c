/*
 * RCC.c
 *
 *  Created on: 14 Oct. 2021
 *      Author: Gaurav
 */


#include "blackpill.h"
#include "RCC.h"

     // clock can be set equal to 100Mhz
/*1. Set the HSE bit[16] is set to 1 for external HSE clock on.
 *2. Check the HSERDY bit[17] is set or not. in while loop.
 *3. Set the bit PWR_EN bit[28]
 *4. Set the PWR bit[15] to 1
 *5. Set the ICEN bit[9], PRFTEN Bit[8] , DCEN bit[10] to 1 and Latency to 3
 *6. Set respective prescalar of PLL_CFGR register
 *7. PLLM - /8 - RCC_PLLCFGR bit[0] set to 8
 *8. PLLN - x84 - RCC_PLLCFGR bit[6] set to 84
 *9. PLLP - /2	 - RCC_PLLCFGR bit[16] set to 0
 *10. Configure the AHB APB1 and APB2 buss prescalar
 *11. Configuration in RCC_CFGR - APB1 bit[10] to 100 for /2 prescalar
 *11. configuration in RCC_CFGR - APB2 bit[13] to 100 for /2 prescalar
 *12. configuration in RCC_CFGR - AHB bit[4]  to 0 as no prescalar.
 *13. turn on PLL - RCC _CR bit[24] SET TO 1
 *14.  wait for PLLRDY - RCC_CR bit[25] in while
 *15. turn on the sysclk as PLL by setting bit[0] in RCC_CFGR to 2
 *16. Check bit [2] in RCC_CFGR if ready.
 *17. Activate the MCO1 clock as HSE and corresponding Prescalar
 */


void system_clk_init();
void  MCO_pin_config();

void  MCO_pin_config()
{
		// From reference manual MCO1 Pin is PA8 setting PA8 now.
	GPIOA_CLK_EN();
	GPIOA->MODER &= ~(0XFFFF);
	GPIOA->MODER |= (2<<16);    // ALTERNATE FUNCTION
	GPIOA->OTYPER &=~(0XFFFF);
	GPIOA->OTYPER &= ~(1<<8);	// SET AS PUSH PULL
	GPIOA->OSPEEDR |= (3<<16);  // SPEED SET TO FAST
	GPIOA->AFRH &= ~(0X0F<< 0); // ALTERNATE FUNCTION SET TO AF0 FOR MCO1, INFO FROM DATASHEET
}

void system_clk_init()
{
	RCC->RCC_CR |= 1<<16; // We will use HSE. HSE bit set
	while(!(RCC->RCC_CR & (1 << 17))); 	// HSE ready bit check condition
	RCC->RCC_APB1ENR |= (1<<28); // bit[28] of RCC_APB1ENR
	PWR->PWR_CR |= (1<<15); // scale 2 mode <= 84Mhz;
	FLASH->FLASH_ACR |= ((1<<8)	| (3<<0) |(1<<9) | (1<<10) ); //

	/* activating PLL
	 * BIT[22] is set to 1 for HSE clock
	 * BIT[22] is set to zero for HSI clock
	 */
	RCC->RCC_PLLCFGR |= (1<<22);
	/*
	 * setting PLLM according to the clock in CUBEmx ( for easiness)
	 * according to the prescalar /12 - Push value 12
	 */
	RCC->RCC_PLLCFGR |= (12 << 0);
	/*
	 *  PLLN settings
	 *  /96 IN PUSHED
	 *
	 */
	RCC->RCC_PLLCFGR |= (96<<6);
	/*
	 *PLLP SETTING
	 *PLLP /2  so  000 IS PUSHED
	 */
	RCC->RCC_PLLCFGR &= ~(1<<16); // set to zero
	/*
	 *  Setting PPRE1 WITH 100 AS /2
	 */
	RCC->RCC_CFGR |=(4<<10); // 100 IS 2 APB1
	RCC->RCC_CFGR &=(1<<13); // 000 IS 1 APB2
	RCC->RCC_CFGR &=(1<<4);  // 000 IS /1 AHB

	// TURN ON PLL
	RCC->RCC_CR |= (1<<24);
	while(!(RCC->RCC_CR &(1<<25))); // WAIT TILL PLLRDY BIT IS 1
	// activating sysCLK as HSE
	RCC->RCC_CFGR |= (2<<0);
	while(!(RCC->RCC_CFGR & (2<<0)));

	// Clock to microcontroller
	/*
	 * Setup the MCO1 pin in alternate functions
	 * // MCO1 Prescalar setup. /1 so push
	 *
	 */

	MCO_pin_config(); // Function for Pin setup.
	RCC->RCC_CFGR &= ~(3<<24); // Prescalar is set to /1 no presclar on bit[26:24]
	RCC->RCC_CFGR |= (3<<21); // PLL selected 11 (3) pushed
	// MICROCONTROLLER WORKS AT 25MHZ.
}
