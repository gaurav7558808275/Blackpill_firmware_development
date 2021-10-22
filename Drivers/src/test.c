/*
 * test.c
 *
 *  Created on: 22 Oct. 2021
 *      Author: Gaurav
 */


#include "test.h"

const char Bytes[] = "S,E,T,O,A,I,P,O,O,O,O,O,O,P,o,{,1,0,1,},d,2,100,=,o,{,4,0,100,},R,S,T"; //
GPIOx_Handle gpio; // check gpio.h header file.

void Test_reset()
{
	if((Bytes[29] == 'R')&&(Bytes[30] == 'S') && (Bytes[31]=='T')){
	 search();
		}
}


void PWM1_Init(GPIOx_Handle *GPIO_Handle)
{
	/*if(GPIO_Handle->pGPIOx == GPIOA){
		GPIOA_PWN_INIT();
	}*/
}
void search()
{
	if((Bytes[0]=='S') && (Bytes[1]=='E') && (Bytes[2]=='T'))
	{
		for(volatile int i=0; i<14 ;i++)
		{ // gpio inits till 14 bit

		if(Bytes[i] == 'O'){
			gpio.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_OUT;
			}
		else if(Bytes[i] == 'A'){
			gpio.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_ANL;
			}
		else if(Bytes[i] == 'I'){
			gpio.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_IN;
		}
		else if(Bytes[i] == 'P'){
			gpio.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_OUT;
		}
	   }
	if(Bytes[14] == 'o'){
		// CONTREOL BITS FOR{,1,0,1,} SECTION
		for(volatile int i = 16 ; i< 19 ;i++){
			if(Bytes[i] == '1'){
				gpio.pGPIOx = GPIOA;  // CUSTOM SELECTED FOR MY MICROCONTROLLER
				GPIO_Init(GPIOA);
			}
			else if(Bytes[i] == '0'){
				gpio.pGPIOx = GPIOB;
				GPIO_Init(GPIOB);
			}
		}
	 }


	// Dint understand the 8th and 9th point.

	if(Bytes[20] == 'd'){
		if((Bytes[21] == '2') && (Bytes[23] == '100')){
			//PWM1_Init(GPIOA);
			}
		else if((Bytes[21] == '2') && (Bytes[23] == '=')){
			//PWM1_Init(GPIOB);  not initiated yet
			}
		}
	}

}
