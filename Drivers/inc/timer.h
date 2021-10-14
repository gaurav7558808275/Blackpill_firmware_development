/*
 * timer.h
 *
 *  Created on: 6 Oct. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_TIMER_H_
#define DRIVERS_INC_TIMER_H_

#include "blackpill.h"


/*
 * We will create funtions for delay function
 *  like microsecond delay
 *  millisecond delay
 *  and second delay.
 */


/*
 * Now we know the clock is 100Mhz. Refer RCC clock init function
 * timer prescalar is 1
 *
 * Timer frequency (TF) = Fsysclk / (Prescalar + 1) => 100Mhz
 *
 * Timer Time Period (TTP) = 1/ TF = 1/100Mhz = .01 * 10^-6 = .01us
 * if we need 1us delay
 * 1us = x * .01us So, calculating x = 100us
 *
 * so, time delay = TP* ARR  = .01 * 100
 *
 * TTP    =  1us = 1 count
 * 1000us = 1000 counts
 * 1ms 	  = 1000us
 *
 *
 */
void timer_init();
void us_delay(uint16_t us);
void ms_delay(uint16_t ms);
void s_delay(uint16_t s);



#endif /* DRIVERS_INC_TIMER_H_ */
