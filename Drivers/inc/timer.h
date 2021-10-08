/*
 * timer.h
 *
 *  Created on: 6 Oct. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_TIMER_H_
#define DRIVERS_INC_TIMER_H_

#include "blackpill.h"


/*****************************************************************
 *  Number determines wait states
 *  0- 0 wait state
 *  1 - 1 wait state
 *  In the .c, Here we take 1 wait state;
 *****************************************************************/
#define latency_wait_sate_0 		0
#define latency_wait_sate_1 		1
#define latency_wait_sate_2 		2
#define latency_wait_sate_3 		3
#define latency_wait_sate_4 		4
#define latency_wait_sate_5 		5
#define latency_wait_sate_6 		6
#define latency_wait_sate_7 		7
#define latency_wait_sate_8 		8
#define latency_wait_sate_9 		9
#define latency_wait_sate_10 		10
#define latency_wait_sate_11 		11
#define latency_wait_sate_12 		12
#define latency_wait_sate_13 		13
#define latency_wait_sate_14 		14




void  MCO_pin_config();
void  system_clk_init();






#endif /* DRIVERS_INC_TIMER_H_ */
