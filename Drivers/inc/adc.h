/*
 * adc.h
 *
 *  Created on: 16 Oct. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_ADC_H_
#define DRIVERS_INC_ADC_H_

#include "blackpill.h"

void adc_init();
void adc_start();
uint16_t adc_convert();

#endif /* DRIVERS_INC_ADC_H_ */
