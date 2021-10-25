/*
 * I2C_main.c
 *
 *  Created on: 25 Oct. 2021
 *      Author: Gaurav
 */


#include "blackpill.h"
#include "gpio.h"
#include "i2c.h"


I2C_Handle_t I2C_Handle;
GPIOx_Handle I2C_gpio;
/*
 * PB6 - SCL
 * PB9 - SDA
 */
void I2C_Gpio_Pins();
void I2C_Init_main();
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
	I2C_gpio.GPIO_Pin_config.GPIO_PinNumber |= GPIO_PIN_9;
	GPIO_Init(&I2C_gpio);
}
/*
 * I2C configuration init()
 *
 */
void I2C_Init_main(){

	I2C_Handle.I2C_Config.I2C_ACKControl |= I2C_ACK_ENABLE;
	I2C_Handle.I2C_Config.I2C_Device_Addr |= 0x12; // check the NXP um10204 for allowed address
	I2C_Handle.I2C_Config.I2C_FMduty |= I2C_FMDUTY_2; // 50% duty cycle
	I2C_Handle.I2C_Config.I2C_SCL_Speed |= I2C_SCL_SM_SPEED; // Sm mode
}



int main(void)
{
	while(1){

	}

}
