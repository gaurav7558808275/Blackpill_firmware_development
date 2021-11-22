/*
 * UART_test.c
 *
 *  Created on: 22 Nov. 2021
 *      Author: Gaurav
 */


#include "string.h"
#include "blackpill.h"
#include "UART.h"
#include "gpio.h"
#include "timer.h"
#include "string.h"

// handle init
USART_Handle_t Uart;
GPIOx_Handle GPIO;
char msg[1024] = "Hello, the UART is ready";
// creation of UART init

void uart_init(void){
		Uart.pUSARTx = USART6;
		Uart.USART_Config.USART_HWcontrol = USART_HW_FLOW_CTRL_NONE;
		Uart.USART_Config.USART_baud = USART_STD_BAUD_9600;
		Uart.USART_Config.USART_mode = USART_MODE_ONLY_TX;
		Uart.USART_Config.USART_no_ofstopbits = USART_STOPBITS_1;
		Uart.USART_Config.USART_paritybit = USART_PARITY_DISABLE;
		Uart.USART_Config.USART_wordlength = USART_WORDLEN_8BITS;
	}
void gpio_init(void){
	GPIO.pGPIOx = GPIOD;
	GPIO.GPIO_Pin_config.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_12;
	GPIO.GPIO_Pin_config.GPIO_PinOutputType = GPIO_MODE_OUT_OD;
	GPIO.GPIO_Pin_config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO.GPIO_Pin_config.GPIO_PinPuPdcontrol = GPIO_PORT_N_PP;
}



int main(void){


	// config UART_INIT()
	uart_init();
	// main uart_init
	USART_Init(&Uart);
	//config gpio_init()
	gpio_init();
	//main gpio_init()
	GPIO_Init(&GPIO);
	//UART start
	USART_PeripheralControl(Uart.pUSARTx,ENABLE);
	while(1){
			while(!GPIO_Read_Pin(GPIOD, GPIO_PIN_12));
			s_delay(1);
			USART_SendData(&Uart, (uint8_t*)msg,strlen(msg));

	}

	return 0;
}
