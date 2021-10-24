/*
 * i2c.c
 *
 *  Created on: 28 Sep. 2021
 *      Author: Gaurav
 */

#include "i2c.h"
#include "blackpill.h"

uint32_t AHB_Prescalar_arr[9] = {2,4,8,16,32,64,128,256,512};
uint32_t APB1_Prescalar_arr[4] = {1,4,8,16};



void I2C_Clock_EN(I2C_RegDef_t *pI2Cx);	// I2C Clock Initialize
void I2C_Clock_DE(I2C_RegDef_t *pI2Cx);
void I2C_Init(I2C_Handle_t *I2C_Handle);
void I2C_Deinit(I2C_Handle_t *I2C_Handle);

void I2C_MasterSend(I2C_Handle_t *pI2CHandle, uint8_t *ptx_buff , uint32_t length,uint8_t Sadd);
void I2C_Receive(I2C_RegDef_t *pI2Cx, uint8_t *rx_buff , uint32_t length);

uint8_t I2C_Send_IT(I2C_Handle_t * pI2C_Handle_t, uint8_t *tx_buff , uint32_t length);
uint8_t I2C_Receive_IT(I2C_Handle_t *pI2C_Handle_t, uint8_t *rx_buff , uint32_t length);

void I2C_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);  // SET OR RESET
void I2C_IRQ_Handling(I2C_Handle_t *pI2C_Handle_t );
void I2C_Priority_Config(uint8_t IRQ_number , uint32_t priority);

void I2C_Peripheral_Control(I2C_RegDef_t *I2C_Handle, uint8_t S_O_R);
void I2C_SSI_Enable(I2C_RegDef_t *I2C_Handle, uint8_t S_O_R);
void I2C_SSOE_Enable(I2C_RegDef_t *I2C_Handle, uint8_t S_O_R);
uint8_t I2C_BusyFlag(I2C_RegDef_t *I2C_Handle);

void Clear_OVRflag(I2C_Handle_t *I2C_Handle);
void Close_Transmission(I2C_Handle_t *I2C_Handle);
void Close_Reception(I2C_Handle_t *I2C_Handle);

// Sub-functions
uint32_t CLK_Freq_calculate(void);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2C_Handle);
static void ExecuteAddress(I2C_RegDef_t * pI2CHandle,uint8_t Saddr);
static void Read_clear_ADDR(I2C_RegDef_t *pI2CHandle);

void I2C_Clock_EN(I2C_RegDef_t *pI2Cx )	// I2C Clock Initialize
{
	if(pI2Cx == I2C1)
	{
		I2C1_CLOCK_EN();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_CLOCK_EN();
	}
	else if(pI2Cx == I2C2)
	{
		I2C3_CLOCK_EN();
	}
	else
	{
		I2C1_CLOCK_DI();
		I2C2_CLOCK_DI();
		I2C3_CLOCK_DI();
	}

}
void I2C_Clock_DE(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
		{
			I2C1_CLOCK_DI();
		}
	else if(pI2Cx == I2C2)
		{
			I2C2_CLOCK_DI();
		}
	else if(pI2Cx == I2C2)
		{
			I2C3_CLOCK_DI();
		}

}
/*
 * LOCAL FUNCTION FOR CLOCK GENERATION FOR I2C_Init()
 */
uint32_t CLK_Freq_Calculate(void)
{
	uint8_t pcklk = 0,prescalarahb =0 ,prescalarapb1 =0;
	uint32_t clksrc = 0,temp = 0;
	temp = ((RCC->RCC_CFGR >> 2) & 0x03);
	if(temp == 0)
		{
		clksrc = CLK_HSI;
		}
	else if(temp == 1)
		{
		clksrc = CLK_HSE;
		}
	else if(temp == 2)
		{
		clksrc = CLK_PLL;
		}

	temp = ((RCC->RCC_CFGR >> 4) & 0XF); // prescalar for AHB
	if(temp <8)
		{
		prescalarahb = 1;
		}
	else if(temp >= 8)
		{
		prescalarahb = AHB_Prescalar_arr[temp-8];
		}
	temp = ((RCC->RCC_CFGR >> 10) & 0XF);// prescalar for APB
	if(temp <4)
		{
			prescalarapb1 = 1;
		}
	else if(temp >= 4)
		{
			prescalarapb1 = APB1_Prescalar_arr[temp-4];
		}


	pcklk =  (clksrc/prescalarahb) / prescalarapb1; // formulae for calculation of clock.
	return pcklk;
}

void I2C_Init(I2C_Handle_t *I2C_Handle){

	// 1) taking the ACK enable.from CR1
	uint32_t tempreg = 0;
	tempreg |= I2C_Handle->I2C_Config.I2C_ACKControl << 10;  // ACK bit read

	// configure the FREQ assign in CR2 register
	tempreg = 0;
	tempreg = CLK_Freq_Calculate() / 1000000UL;  // need only the value(Hz-Mhz conversion)
	I2C_Handle->pI2Cx->I2C_CR2 |= (tempreg & 0x3F);  //  I2C_Handle->pI2Cx->I2C_CR2 |= (tempreg & 0x3F)

	// register I2C_OAR2 Program device own address
	tempreg = I2C_Handle->I2C_Config.I2C_Device_Addr << 1; //ADD[0] is Ignored
	I2C_Handle->pI2Cx->I2C_OAR1 |= (tempreg & 0xFE); // FROM BIT[1]
	I2C_Handle->pI2Cx->I2C_OAR1 &= ~(1<<15);  // 7 bit addressing
	tempreg |=(1<<4);
	I2C_Handle->pI2Cx->I2C_OAR1 |= tempreg;
	uint16_t CCR_value =0;
	tempreg =0;

	/*________________________________________*/

	// CCR - Setting up speed
	if(I2C_Handle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SM_SPEED ){
		// it is standard mode
		I2C_Handle->pI2Cx->I2C_CCR &=(1<<15); // to setup sm mode
		CCR_value = ((CLK_Freq_Calculate() / 2 * I2C_Handle->I2C_Config.I2C_SCL_Speed));
		tempreg |= (CCR_value & 0xFFF);
	}

	else {
		// fast mode setup
		I2C_Handle->pI2Cx->I2C_CCR |= (1<<15); // set the fast mode
		tempreg |= (I2C_Handle->I2C_Config.I2C_FMduty << 14); // TODO: check the function
		if(I2C_Handle->I2C_Config.I2C_FMduty == I2C_FMDUTY_2 ){
			CCR_value |= ((CLK_Freq_Calculate() / 3 * I2C_Handle->I2C_Config.I2C_SCL_Speed)); // check the reference manual and converted to frequency domain
		}
		else{
			CCR_value |= ((CLK_Freq_Calculate() / 25 * I2C_Handle->I2C_Config.I2C_SCL_Speed));
		}
		tempreg |= (CCR_value & 0xFFF);
	}
	I2C_Handle->pI2Cx->I2C_CCR |=tempreg;

	// todo : Setup the T_rise register
	/*
	 * 	If, in the I2C_CR2 register, the value of FREQ[5:0] bits is equal to 0x08 and TPCLK1 = 125 ns
		therefore the TRISE[5:0] bits must be programmed with 09h.
		(1000 ns / 125 ns = 8 + 1)
	 */
	if(I2C_Handle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SM_SPEED ){
		// set up the trise for SM mode
		tempreg |= (CLK_Freq_Calculate()/ 1000000U) +1; //  1Mhz max for standard mode
	}
	else{
		// set the trisde for FM mode
		tempreg |= (CLK_Freq_Calculate()*300U / 1000000000U) +1;
	}
	I2C_Handle->pI2Cx->I2C_TRISE |= (tempreg & 0x1F);
}

/*
 *	LOCAL FUNCTION FOR MASTER SEND and Main Master Send available here!
 *
 */


void I2C_MasterSend(I2C_Handle_t *pI2CHandle, uint8_t *ptx_buff , uint32_t length,uint8_t Saddr){

	// 1. initiate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//  2 .CHECK FLAG - confirm the start condition by checking the SB bit[0] in	SR1 register
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1<<0)));
	// 3.EXECUTE THE ADDRESS PHASE send the address with 7 bit and r/w bit in the end
	ExecuteAddress(pI2CHandle->pI2Cx,Saddr);
	// 4.confirm that the address phase is completed by checking the SR1 register bit[1] ADDR
	// need to clear the ADDR flag by reading from it.
	Read_clear_ADDR(pI2CHandle->pI2Cx);
	// 5.Send data till  length is empty. That can be done by checking the TXE
	while(length < 0){
		while(!(pI2CHandle->pI2Cx->I2C_SR1 &(1<<7)));  // txe check
		pI2CHandle->pI2Cx->I2C_DR = *ptx_buff;
		ptx_buff ++;
		length --;
		}
	// when the length is empty generate the end set.
	while(!(pI2CHandle->pI2Cx->I2C_SR1 &(1<<7)));   // TXE bit check
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1<<2)));  // BTF bit check
	// Generate the stop condition
	pI2CHandle->pI2Cx->I2C_CR1 |= (1<< 9); // writing into the bit[9] in CR1b
}


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2C_Handle){
	pI2C_Handle->I2C_CR1 |= (1<<8);  //BIT[8] = START BIT CHECK REFERENCE MANUAL
}
static void ExecuteAddress(I2C_RegDef_t *pI2CHandle,uint8_t Saddr){
	Saddr = Saddr << 1 ;
	Saddr &= ~(1<<0);    // adding address and r/w bit in the end
	Saddr |= pI2CHandle->I2C_DR;
}
static void Read_clear_ADDR(I2C_RegDef_t *pI2C_Handle){
	uint32_t Dummy =0;
	Dummy |= pI2C_Handle->I2C_SR1;
	Dummy |= pI2C_Handle->I2C_SR2;
	(void)Dummy; // typecast to void.
}
