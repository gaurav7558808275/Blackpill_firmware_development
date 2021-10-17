/*
 * adc.h
 *
 *  Created on: 16 Oct. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_ADC_H_
#define DRIVERS_INC_ADC_H_

#include "blackpill.h"
#include "gpio.h"
#include "timer.h"

/*
typedef struct
{
	uint8_t adc_resolution; TODO: MOSLTY BY STRUCTURE APPROACH.
	uint8_t
 	 // TODO THE CONFIG STRUCT FOR ADC
}
*/

typedef struct
{
	uint8_t channel;
	uint8_t size;
  // TODO THE HANDLE STRUCT
}ADC_Handle;


void adc_init();
void adc_start();
uint16_t adc_convert();

#endif /* DRIVERS_INC_ADC_H_ */
