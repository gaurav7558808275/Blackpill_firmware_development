/*
 * blackpill_rcc.h
 *
 *  Created on: 8 Nov. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_BLACKPILL_RCC_H_
#define DRIVERS_INC_BLACKPILL_RCC_H_

#include "blackpill.h"
#include "I2C.h"

uint32_t APB1_CLK_Freq_Calculate(void); // usart2
uint32_t APB2_CLK_Freq_Calculate(void); // usart1,6

#endif /* DRIVERS_INC_BLACKPILL_RCC_H_ */
