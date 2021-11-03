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

void I2C_MasterSend(I2C_Handle_t *pI2CHandle, uint8_t *ptx_buff , uint32_t length,uint8_t Sadd,uint8_t SR); /*Used*/
void I2C_MasterReceive(I2C_Handle_t *pI2CHandle, uint8_t *prx_buff , uint32_t length,uint8_t Sadd,uint8_t SR);/*Used*/

uint8_t I2C_MasterSend_IT(I2C_Handle_t *pI2CHandle, uint8_t *ptx_buff , uint32_t length,uint8_t Sadd,uint8_t SR); /*used*/
uint8_t I2C_MasterReceive_IT(I2C_Handle_t *pI2CHandle, uint8_t *ptx_buff , uint32_t length,uint8_t Sadd,uint8_t SR);/*used*/

void I2C_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R);  // SET OR RESET /* used*/
void I2C_Priority_Config(uint8_t IRQ_number , uint32_t priority); /* used*/
void I2CEV_IRQ_Handling(I2C_Handle_t *pI2CHandle); /* used */
void I2CER_IRQ_Handling(I2C_Handle_t *pI2CHandle); /*used */

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
static void ExecuteAddress_Mastersend(I2C_RegDef_t * pI2CHandle,uint8_t Saddr);
static void ExecuteAddress_MasterReceive(I2C_RegDef_t *pI2CHandle,uint8_t Saddr);
static void Read_clear_ADDR(I2C_RegDef_t *pI2CHandle);
void I2C_Manage_ACK(I2C_RegDef_t *pI2C, uint8_t S_O_R);
/*
 *
 * Clock enable and disable API's
 *
 */
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
/*
 * I2C clock disable API
 *
 */
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
 *
 *  LOCAL FUNCTION FOR CLOCK GENERATION FOR I2C_Init()
 *
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
/*
 *
 * I2c init function with clock frequency calculator static function
 *
 */
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
void I2C_MasterSend(I2C_Handle_t *pI2CHandle, uint8_t *ptx_buff , uint32_t length,uint8_t Saddr,uint8_t SR){

	// 1. initiate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//  2 .CHECK FLAG - confirm the start condition by checking the SB bit[0] in	SR1 register
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1<<0)));
	// 3.EXECUTE THE ADDRESS PHASE send the address with 7 bit and r/w bit in the end in this case its R(0)
	ExecuteAddress_Mastersend(pI2CHandle->pI2Cx,Saddr);
	// 4.confirm that the address phase is completed by checking the SR1 register bit[1] ADDR
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1<<1)));
	// need to clear the ADDR flag by reading from it from SR1 and SR2 registers.
	Read_clear_ADDR(pI2CHandle->pI2Cx);
	// 5.Send data till  length is empty. That can be done by checking the TXE
	while(length < 0){
		while(!(pI2CHandle->pI2Cx->I2C_SR1 &(1<<7)));  // txe bit check
		pI2CHandle->pI2Cx->I2C_DR = *ptx_buff;
		ptx_buff ++;
		length --;
		}
	// when the length is empty generate the end set.
	while(!(pI2CHandle->pI2Cx->I2C_SR1 &(1<<7)));   // TXE bit check
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1<<2)));  // BTF bit check

	if(SR==I2C_DISABLE_SR){  // Generate the stop condition
		pI2CHandle->pI2Cx->I2C_CR1 |= (1<< 9);// writing into the bit[9] in CR1b
	}
}
/*
 *  Sub function declaration from here
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2C_Handle){
	pI2C_Handle->I2C_CR1 |= (1<<8);  //BIT[8] = START BIT CHECK REFERENCE MANUAL
}
static void ExecuteAddress_Mastersend(I2C_RegDef_t *pI2CHandle,uint8_t Saddr){
	Saddr = Saddr << 1 ;
	Saddr &= ~(1<<0);    // adding address and r/w bit in the end to 0 because its read mode
	Saddr |= pI2CHandle->I2C_DR;
}
static void ExecuteAddress_MasterReceive(I2C_RegDef_t *pI2CHandle,uint8_t Saddr){
		Saddr = Saddr << 1 ;
		Saddr |= (1<<0);    // adding address and r/w bit in the end to 0 because its read mode
		pI2CHandle->I2C_DR |= Saddr;
}
static void Read_clear_ADDR(I2C_RegDef_t *pI2C_Handle){
	uint32_t Dummy =0;
	Dummy |= pI2C_Handle->I2C_SR1;
	Dummy |= pI2C_Handle->I2C_SR2;
	(void)Dummy; // typecast to void.
}

/*
 *I2C master receive API
 *
 * # Generate the start bit
 * # Confirm that the start bit by checking the SB flag
 * # send the address of the slave with R/W bit in this case R(1) (total 8 bit)
 * # Wait until the address phase is completed by checking the ADDR register
 */
void I2C_MasterReceive(I2C_Handle_t *pI2CHandle, uint8_t *prx_buff , uint32_t length, uint8_t Sadd,uint8_t SR){
	// Generating the start condition with the sub functions created
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// checking whether the start condition is initiated or not
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1<<0)));
	// adding the last bit as 0 in this case as we are receive in master mode so separate function created
	ExecuteAddress_MasterReceive(pI2CHandle->pI2Cx,Sadd);
	// check the ADDr flag if set or not,Wait until the address phase is over
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1<<1)));
	// Now clear the SR1 and SR2 read with the help of this function
	Read_clear_ADDR(pI2CHandle->pI2Cx);
	// Receive the data has two modes 1) 1byte mode from slave 2) multiple byes from slave
	// condition if 1 byte is asked from slave
	if(length == 1){

		// disable the ACK
		I2C_Manage_ACK(pI2CHandle->pI2Cx,DISABLE);
		// clear the ADDr bit by calling this static API
		Read_clear_ADDR(pI2CHandle->pI2Cx);
		// wait until RXNE becomes 1 in SR1 register (Data register not empty)
		 /* 0: Data register empty
			1: Data register not empty*/
		while(!(pI2CHandle->pI2Cx->I2C_CR1 & (1<<6)));
		// Generate the stop condition
		if(SR==I2C_DISABLE_SR)
			pI2CHandle->pI2Cx->I2C_CR1 |= (1<< 9);
		// read data to buffer
		*prx_buff |= pI2CHandle->pI2Cx->I2C_DR;
	}
	else if(length > 1){
		// clear the ADDR bit
		Read_clear_ADDR(pI2CHandle->pI2Cx);  // start of ddata reception
		// for loop from len to 0 in decrement
		for(volatile int i = length ; i > 0 ; i--){
			// wait until RXNE is set
			while(!(pI2CHandle->pI2Cx->I2C_CR1 & (1<<6)));
			if(length ==2){
				// clear the Ack bit
				I2C_Manage_ACK(pI2CHandle->pI2Cx,DISABLE);
				// initiate the stop condition
				if(SR==I2C_DISABLE_SR)
					pI2CHandle->pI2Cx->I2C_CR1 |= (1<< 9);
			}
			// copy the data from the buffer
			*prx_buff |= pI2CHandle->pI2Cx->I2C_DR;
			// increment the buffer
			prx_buff++;
			return;
		}
		// re-enable the ACK if only the ACK is enabled in the HAndle!
		if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
			I2C_Manage_ACK(pI2CHandle->pI2Cx,ENABLE);
		}

	}

}
/*
 * Required for Master receive API
 */
void I2C_Manage_ACK(I2C_RegDef_t *pI2C, uint8_t S_O_R){

	if(S_O_R == ENABLE){
		// enable the ACK
		pI2C->I2C_SR1 |= (1<<10);
	}
	else if(S_O_R == DISABLE){
		// disable
		pI2C->I2C_SR1 &= ~(1<<10);
	}
}
/*
 * Interrupt based API in I2C
 *
 */
uint8_t I2C_MasterSend_IT(I2C_Handle_t *pI2CHandle, uint8_t *ptx_buff , uint32_t length,uint8_t Sadd,uint8_t SR){

	uint8_t busystate = pI2CHandle->txrxstate;
	if((busystate != I2C_BUSY_IN_TX)&&(busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->ptxbuffer = ptx_buff;
		pI2CHandle->txlen = length;
		pI2CHandle->txrxstate = I2C_BUSY_IN_TX;
		pI2CHandle->devaddr = Sadd;
		pI2CHandle->sr = SR;

		// initiate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		// Set the ITBUFEN bitin CR2 buffer interrupt signal
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<10);
		// Set the ITEVTEN bit in CR2 register
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<9);
		// Set the ITERREN: bit in CR2 register
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<8);
	}

	return busystate;
}
/*
 * Master receive API using interrupt
 *
 */
uint8_t I2C_MasterReceive_IT(I2C_Handle_t *pI2CHandle, uint8_t *prx_buff , uint32_t length,uint8_t Sadd,uint8_t SR){
	uint8_t busystate = pI2CHandle->txrxstate;
	if((busystate != I2C_BUSY_IN_TX)&&(busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->prxbuffer = prx_buff;
		pI2CHandle->rxlen = length;
		pI2CHandle->txrxstate = I2C_BUSY_IN_RX;
		pI2CHandle->devaddr = Sadd;
		pI2CHandle->sr = SR;

		// initiate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		// Set the ITBUFEN bit in CR2 buffer interrupt signal
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<10);
		// Set the ITEVTEN bit in CR2 register
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<9);
		// Set the ITERREN: bit in CR2 register
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<<8);
	}

	return busystate;

}

/*
 *
 * I2C IRQ config API
 *
 *
 */
void I2C_IRQ_IT_config(uint8_t IRQ_Number, uint8_t S_O_R){ // SET OR RESET /* used*/
	if (S_O_R == ENABLE)
		{
			if(IRQ_Number <= 31)
			{
			*NVIC_ISER0 |= (1 << IRQ_Number);
			}
			else if(IRQ_Number >31 && IRQ_Number <64) // 32-63
			{
			*NVIC_ISER1 |= (1 << (IRQ_Number % 32));
			}
			else if(IRQ_Number >64 && IRQ_Number <96)
			{
			*NVIC_ISER2 |= (1 << (IRQ_Number % 64));
			}
		}
		else
		{
			if(IRQ_Number <= 31)
			{
			*NVIC_ICER0 &= ~(1 << IRQ_Number);
			}
			else if(IRQ_Number >31 && IRQ_Number <64) // 32-63
			{
			*NVIC_ICER1 &= ~(1 << (IRQ_Number % 32));
			}
			else if(IRQ_Number >64 && IRQ_Number <96)
			{
			*NVIC_ICER2 &= ~(1 << (IRQ_Number % 64));
			}
		}

}
/*
 *
 *
 *  Priority setup for I2C
 *
 *
 */

void I2C_Priority_Config(uint8_t IRQ_number , uint32_t priority){

	uint8_t iprx = IRQ_number/4;   	// TO FIND THE WHICH REGISTER OF PRI
	uint8_t iprx_section = IRQ_number % 4;		// THIS IS USED TO FIND THE SECTION THAT THE DTAA WILL BE UPDATED
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_BIT_IMPLEMENTED);	// IN THE IPR LAST 4 BITS ARE INGNORED
	*(NVIC_IPR0 + iprx) |= (priority << (shift_amount)); // SETS ALL THE 8 BIT REGISTER TO THE REQUIRED VALUE(PRIORITY)
}
/*
 *
 *
 *
 * IRQ handling function declaration
 *
 */
void I2CEV_IRQHandling(I2C_Handle_t *pI2CHandle){
	// check the event bit is set or not
	// bit checks are TEVFEN,ITEVFEN and ITBUFEN,ITERREN for more data check the reference manual pg 490
	uint32_t temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1<<10);// bit check  ITBUFEN
	uint32_t temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1<<9);	// bit check ITEVTEN
	uint32_t temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<0); // bit check SB
	// check SB
	if(temp2 && temp3){
		// ready handle for SB. qont work in slave mode as in slave mode it is not neccessary
		// In this block we will execute the address phase
		if(pI2CHandle->txrxstate == I2C_BUSY_IN_TX){
			ExecuteAddress_Mastersend(pI2CHandle->pI2Cx,pI2CHandle->devaddr);
		}else if(pI2CHandle->txrxstate == I2C_BUSY_IN_RX){
			ExecuteAddress_MasterReceive(pI2CHandle->pI2Cx, pI2CHandle->devaddr);
		}

	}
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<1); // check addr bit
	//check ADDR
	if(temp2 && temp3){
			// ready handle for  ADDR
		Read_clear_ADDR(pI2CHandle->pI2Cx);
		}
	/*// check ADD10
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<3);
	if(temp2 & temp3){
		// handle for ADD10 bit set
	}*/ // not used as we don't use 10 bit addressing

	// check bit STOPF
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<4);
	if(temp2 && temp3){
			// handle for STOPF bit set
		}
	// check for btf
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<2);
	if(temp2 && temp3){
		// handle for btf
		// btf and TxE during transmission  // btf and rxne during reception
		if(pI2CHandle->txrxstate == I2C_BUSY_IN_TX ){
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1<<2)){
				// BTF and TXe is set
				// close transmission only if repeated start is disabled or not activated
				if(pI2CHandle->sr == I2C_DISABLE_SR)
					pI2CHandle->pI2Cx->I2C_CR1 |= (1<< 9);
				// reset all the I2C handle definitions
				I2C_CloseSendData();
				//notify the application transmission complete
				I2CEventCallBack(pi2CHandle ,I2C_EV_TX_CMPT);
			}
		}
	}
	// Now check the RxNE and TxE
	temp3= pI2CHandle->pI2Cx->I2C_SR1 & (1<<6); // Check the RxNE
	if((temp1 && temp2 && temp3)){
		 // handle for RxNe set
	}
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<<7);
	// chec for Txe
	if((temp1 && temp2 && temp3)){
			 // handle for Txe set
		}

}
void I2CER_IRQhandling(I2C_Handle_t *pI2CHandle){

}

