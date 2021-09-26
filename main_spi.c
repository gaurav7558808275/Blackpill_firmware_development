/*
 * main_spi.c
 *
 *  Created on: 25 Sep. 2021
 *      Author: Gaurav
 */

#include "blackpill.h"
#include "gpio.h"
#include "spi.h"
#include "string.h"

// gpio pins for SPI 2 connection from datasheet pinouts
	//PB13 SPI2_SCK
	//PB12 SPI2_NSS
	//PB14 SPI2_MISO
	//PB15 SPI2_MOSI

	// Application specific functions for spi inits prototype
void SPI2_GPIO_inits(void);
void SPI2_inits(void);
void SPI2_GPIO_inits(void)
{

	GPIOx_Handle pins;
	pins.pGPIOx = GPIOB;
	pins.GPIO_Pin_config.GPIO_PinMode = GPIO_MODE_ALT;
	pins.GPIO_Pin_config.GPIO_AltFunctionMode = 5;
	pins.GPIO_Pin_config.GPIO_PinOutputType = GPIO_MODE_OUT_PP;
	pins.GPIO_Pin_config.GPIO_PinPuPdcontrol = GPIO_PORT_N_PP;
	pins.GPIO_Pin_config.GPIO_PinSpeed = GPIO_SPEED_FAST;

	pins.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_13;  // SCK init
	GPIO_Init(&pins);

	pins.GPIO_Pin_config.GPIO_PinNumber= GPIO_PIN_12;  // NSS init
	GPIO_Init(&pins);

	pins.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_14;  // MOSI init
	GPIO_Init(&pins);

	pins.GPIO_Pin_config.GPIO_PinNumber = GPIO_PIN_15;  // MISO init
	GPIO_Init(&pins);
}

void SPI2_inits(void)
{
	SPI_Handle_t spi;
	spi.pSPIx = SPI2;
	spi.SPI_Config.SPI_Bus_Config = SPI_BUS_CONFIG_FD;
	spi.SPI_Config.SPI_MODE = SPI_MODE_MASTER;
	spi.SPI_Config.SPI_Clk_Speed = SPI_CLOCK_SPEED_DIV2; // generated speed of 8Mhz
	spi.SPI_Config.SPI_Dff = SPI_DFF_8BIT;
	spi.SPI_Config.SPI_CPHA = SPI_CPHA_0;
	spi.SPI_Config.SPI_CPOL = SPI_CPOL_0;
	spi.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&spi);

}


int main(void)
{
	SPI2_GPIO_inits();
	SPI2_inits();

	SPI_SSI_Enable(SPI2,ENABLE);
	/*
	 * // ENABLING THE SPI
	 */
	SPI_Peripheral_Control(SPI2, ENABLE);
	/*
	 * // ENABLING SSOE BIT IN CR2 SAYS ITS IN MASTER MODE
	 */
	SPI_SSOE_Enable(SPI2,ENABLE);

	char data[] = "Hello World";
	SPI_Send(SPI2 , (uint8_t*) data,strlen(data));

	while(SPI_BusyFlag(SPI1));
	SPI_Peripheral_Control(SPI2, DISABLE);


	while(1)
	{

	}
	return 0;
}

