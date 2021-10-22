/*
 * test.h
 *
 *  Created on: 22 Oct. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_TEST_H_
#define DRIVERS_INC_TEST_H_




#include "blackpill.h"
#include "gpio.h"


void search();
void PWM1_Init(GPIOx_Handle *GPIO_Handle);// didnt create as i am yet to play around with timer for PWM generation
void Test_reset();


#endif /* DRIVERS_INC_TEST_H_ */
