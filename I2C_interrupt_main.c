/*
 * I2C_ingterrupt_main.c
 *
 *  Created on: 6 Nov. 2021
 *      Author: Gaurav
 */


#include "blackpill.h"
#include "gpio.h"
#include "i2c.h"
#include "string.h"
#include "timer.h"
#include "stdio.h"


//extern void initialise_monitor_handles();
uint8_t rx= RESET;

I2C_Handle_t I2C_Handle;  // I2C handle for configuration
GPIOx_Handle I2C_gpio;	  // gpio pin config for I2C
GPIOx_Handle button;// Button initilaised.
uint32_t buff[32];
/*
 * PB6 - SCL
 * PB7 - SDA
 */

void I2C_Gpio_Pins();
void I2C_Init_main();
void  Button_init();

/*
 * Config function
 */

void I2C_Gpio_Pins(){
	I2C_gpio.pGPIOx = GPIOB;
	I2C_gpio.GPIO_Pin_config.GPIO_PinMode |= GPIO_MODE_ALT;
	I2C_gpio.GPIO_Pin_config.GPIO_PinOutputType|= GPIO_MODE_OUT_OD;
	I2C_gpio.GPIO_Pin_config.GPIO_PinPuPdcontrol |= GPIO_PORT_UP;
	I2C_gpio.GPIO_Pin_config.GPIO_PinSpeed |= GPIO_SPEED_FAST;
	I2C_gpio.GPIO_Pin_config.GPIO_AltFunctionMode |= GPIO_ALT_MODE_4;
	//SCL
	I2C_gpio.GPIO_Pin_config.GPIO_PinNumber |= GPIO_PIN_6;
	GPIO_Init(&I2C_gpio);
	//SDA
	I2C_gpio.GPIO_Pin_config.GPIO_PinNumber |= GPIO_PIN_7;
	GPIO_Init(&I2C_gpio);
	Button_init();
}
/*
 * I2C configuration init()
 *
 */
#define SLAVE_ADDR 0x12
#define MY_ADDR 0x68

void I2C_Init_main(){

	I2C_Handle.pI2Cx = I2C1;
	I2C_Handle.I2C_Config.I2C_ACKControl |= I2C_ACK_ENABLE;
	I2C_Handle.I2C_Config.I2C_Device_Addr |= SLAVE_ADDR; // 0001 0010 check the NXP um10204 for allowed address
	I2C_Handle.I2C_Config.I2C_FMduty |= I2C_FMDUTY_2; // 50% duty cycle
	I2C_Handle.I2C_Config.I2C_SCL_Speed |= I2C_SCL_SM_SPEED; // Sm mode
}
/*
 * button in initialised
 */

void Button_init(){
	 // called inside the I2C_Gpio_Pins()
	button.pGPIOx = GPIOA;
	button.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_OUT;
	button.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_13;  // SOME RANDOM NUMBER
	button.GPIO_Pin_config.GPIO_PinPuPdcontrol = GPIO_PORT_N_PP;
	button.GPIO_Pin_config.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	button.GPIO_Pin_config.GPIO_PinOutputType = GPIO_MODE_OUT_PP;
	GPIO_Init(&button);

}
/*
 *
 * When button pressed the ,master send to the Data
 */

int main(void)
{
	//initialise_monitor_handles();
	timer_init();  // for delay function
	I2C_Gpio_Pins();
	I2C_Init_main();
	I2C_Clock_EN(I2C_Handle.pI2Cx);

	I2C_IRQ_IT_config(I2C1_EV, ENABLE);
	I2C_IRQ_IT_config(I2C1_ER, ENABLE);

	I2C_Manage_ACK(I2C_Handle.pI2Cx,ENABLE); // a complication had happened regarding the ACK bit intialization
	uint8_t command = 0;
	uint8_t length =0;
	while(1){
		if(GPIO_Read_Pin(GPIOB,GPIO_PIN_13) == 1){
			ms_delay(200); // 200ms delay
			command = 0x51;
			// Data write 1 byte to send command
			while(I2C_MasterSend_IT(&I2C_Handle,&command,length, SLAVE_ADDR, I2C_ENABLE_SR)!= I2C_READY);
			// API to receive master
			while(I2C_MasterReceive_IT(&I2C_Handle,&length,length, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
			// Send the command code 0x52
			command = 0x52;
			while(I2C_MasterSend_IT(&I2C_Handle,&command,length, SLAVE_ADDR, I2C_ENABLE_SR)!= I2C_READY);
			// receive the data
			while(I2C_MasterReceive_IT(&I2C_Handle,(uint8_t *)buff,length, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

			rx = RESET; // reseting back
			while(rx != SET){

			}
			buff[length +1] = '\0'; // used for semihosting
			printf("Data:  %s",(char *)buff); // semihosting print
			rx = RESET;

		}
	}
}

/*
 * IRQ handlers from startup code
 */
void I2C1_EV_IRQHandler(void){
	I2CEV_IRQHandling(&I2C_Handle);
}
/*
 * IRQ handler function from startup codes
 */
void I2C1_ER_IRQHandler(void){
	I2CEV_IRQHandling(&I2C_Handle);
}

void I2CEventCallBack(I2C_Handle_t *pI2CHandle,uint8_t SOR){

	if(SOR == I2C_EV_TX_CMPLT){
	printf("The TX  event was complete\n");
	}else if(SOR == I2C_EV_RX_CMPLT){
	printf("The RX  event was complete\n");
		rx =SET;
	}else if(SOR == I2C_EV_RX_CMPLT){
	printf("The RX  event was complete\n");
	}else if(SOR == I2C_EV_AF_CMPL){
	printf(" ACK not received\n");

	I2C_CloseSendData(pI2CHandle);
	// GENERETE THE STOP CONDITION
	if(pI2CHandle->sr == I2C_DISABLE_SR){  // Generate the stop condition
			pI2CHandle->pI2Cx->I2C_CR1 |= (1<< 9);// writing into the bit[9] in CR1b
	// infinite loop for
			while(1);

		}
	}
}
