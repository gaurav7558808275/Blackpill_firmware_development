/*
 * main_I2C_Receive.c
 *
 *  Created on: 2 Nov. 2021
 *      Author: Gaurav
 */

#include "blackpill.h"
#include "gpio.h"
#include "i2c.h"
#include "string.h"
#include "timer.h"

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
	timer_init();  // for delay function
	I2C_Gpio_Pins();
	I2C_Init_main();
	I2C_Clock_EN(I2C_Handle.pI2Cx);
	uint8_t command = 0;
	uint8_t length =0;
	while(1){
		if(GPIO_Read_Pin(GPIOB,GPIO_PIN_13) == 1){
			ms_delay(200); // 200ms delay
			command = 0x51;
			// Data write 1 byte to send command
			I2C_MasterSend(&I2C_Handle,&command,1, SLAVE_ADDR);
			// API to receive master
			I2C_MasterReceive(&I2C_Handle,&length,1,SLAVE_ADDR);
			// Send the command code 0x52
			command = 0x52;
			I2C_MasterSend(&I2C_Handle, &command,1,SLAVE_ADDR);
			// receive the data
			I2C_MasterReceive(&I2C_Handle,(uint8_t*)buff,length,SLAVE_ADDR);
		}
	}
}
