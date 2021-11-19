/*
 * RCC_test.c
 *
 *  Created on: 18 Nov. 2021
 *      Author: Gaurav
 */


#include "blackpill.h"
#include "RCC.h"

int main(void){


	system_clk_init();
	//MCO_pin_config(); not required



	while(1){
		;
	}

	return 0;
}
