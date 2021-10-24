/*
 * blackpill.h
 *
 *  Created on: Sep 15, 2021
 *      Author: Gaurav
 *
 *
 *      **********MCU Specific header file for Black Pill************
 *
 *
 *
 *
 *
 *
 *
 */

#ifndef DRIVERS_INC_BLACKPILL_H_
#define DRIVERS_INC_BLACKPILL_H_

#include <stdint.h>
#include <stdlib.h>


#define __vol			volatile
#define ENABLE    		1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define __weak__		__attribute__((weak))
/*-------------------------------------------------Processor specific details--------------------------------------------*/

#define NVIC_ISER0		((__vol uint32_t *) 0xE000E100)
#define NVIC_ISER1		((__vol uint32_t *) 0xE000E104)
#define NVIC_ISER2		((__vol uint32_t *) 0xE000E108)
#define NVIC_ISER3		((__vol uint32_t *) 0xE000E10c)

#define NVIC_ICER0		((__vol uint32_t *) 0XE000E180)
#define NVIC_ICER1		((__vol uint32_t *) 0XE000E184)
#define NVIC_ICER2		((__vol uint32_t *) 0XE000E188)
#define NVIC_ICER3		((__vol uint32_t *) 0XE000E18C)

#define NVIC_IPR0				((__vol uint32_t *)0xE000E400)
#define NO_OF_BIT_IMPLEMENTED 	4    // the last 4 bits in priorty register in not implemented so sifting is needed with respect to number of bits implemented

#define FLASH_BASE_ADDR				0X00800000UL    //FLASH ADDRESS - VALUE FROM DATASHEET
#define SRAM_BASE_ADDR				0X20000000UL	//SRAM ADDRESS - VALUE FROM DATASHEET
#define SYSTEM_BASE_ADDR 			0X1FFF0000UL	// THIS IS BASCIALLY ROM

/*---------------------------------------BUS DOMAINS--------------------------------------------------------------*/

#define PER_BASE_ADDR				0X40000000UL
#define APB1_BASE_ADDR				PER_BASE_ADDR	//AHB1 BASE BUS ADDRESS - VALUE FROM DATASHEET
#define APB2_BASE_ADDR				0X40010000UL	//AHB2 BASE BUS ADDRESS - VALUE FROM DATASHEET
#define AHB1_BASE_ADDR				0x40020000UL	// AHB1 - BASE ADDRESS VALUE FROM REFERENCE MANUAL

#define AHB2_BASE_ADDR				0X50000000UL	//AHB2 BASE ADDRESS - VALUE FROM REFERENCE MANUAL
#define RCC_BASE_ADDR				0x40023800UL	// RCC BASE ADDRESS FROM MEMORY MAP
#define PWR_BASE_ADDR				0X40007000UL	// PWR base register address.

/*---------------------------------PERIPHERALS ON AHB1 BUS--------------------------------------------------------*/

#define GPIOA_BASE_ADDR				AHB1_BASE_ADDR
#define GPIOB_BASE_ADDR				0x40020400UL
#define GPIOC_BASE_ADDR				0x40020800UL
#define GPIOD_BASE_ADDR				0x40020C00UL
#define GPIOE_BASE_ADDR				0x40021000UL
#define GPIOH_BASE_ADDR				0x40021C00UL

/*------------------------------ PERIPHERALS ON APB2 PERIPHERALS-------------------------------------------------*/
/*  REFERENCE MANUAL -> MEMORY MAP */
#define USART1_BASE_ADDR 			0x40011000UL
#define USART6_BASE_ADDR			0x40011400UL 			// UARTS

#define SPI1_BASE_ADDR				0x40013000UL
#define SPI4_BASE_ADDR				0x40013400UL			// SPI's
#define SPI5_BASE_ADDR				0x40015000UL

#define I2S1_BASE_ADDR				SPI1_BASE_ADDR
#define I2S4_BASE_ADDR				SPI4_BASE_ADDR			// I2S's
#define I2S5_BASE_ADDR				SPI5_BASE_ADDR

#define EXTI_BASE_ADDR				0x40013C00UL
#define FLASH_INTERFACE_ADDR		0x40023C00UL	// Flash interface register used system clock setup

#define ADC1_BASE_ADDR				0X040012000 // ADC BASE ADDRESS


/*----------------------------------APB1 PERIPHERALS------------------------------------------------------------*/

#define UART2_BASE_ADDR				0x40004400UL		// UART's

#define SPI2_BASE_ADDR				0x40003800UL
#define SPI3_BASE_ADDR				0x40003C00UL		// SPI's

#define I2S2_BASE_ADDR				SPI2_BASE_ADDR
#define I2S3_BASE_ADDR				SPI3_BASE_ADDR		// I2S's

#define I2C1_BASE_ADDR				0x40004400UL
#define I2C2_BASE_ADDR				0x40005400UL		// I2C's
#define I2C3_BASE_ADDR				0x40005800UL
#define TIMER2_BASE_ADDR			0x40000000UL       // Timer 2 base address

/*-------------------------------------GPIO PERIPHERAL STRUCTURE-----------------------------------------------*/

typedef struct
{
	uint32_t __vol  MODER;					/*  GPIO port mode register  OFFSET 0x00 */
	uint32_t __vol	OTYPER;					/*  GPIO port output type register OFFSET 0X04 */
	uint32_t __vol	OSPEEDR;				/*  GPIO port output speed register OFFSET 0X08  */
	uint32_t __vol 	PUPDR;					/* 	GPIO port pull-up/pull-down register OFFSET 0X0C */
	uint32_t __vol	IDR;					/*  GPIO port input data register OFFSET 0X10*/
	uint32_t __vol	ODR;					/*  GPIO port output data register OFFSET 0X14*/
	uint32_t __vol	BSRR;					/*  GPIO port bit set/reset register OFFSET 0X18*/
	uint32_t __vol	LCKR;					/*  GPIO port configuration lock register OFFSET 0X1C*/
	uint32_t __vol	AFRL;					/*  GPIO alternate function low register OFFSET 0X20*/
	uint32_t __vol	AFRH;					/*  GPIO alternate function high register OFFSET 0X24*/

}GPIO_Reg_Def;

/*--------------------------------------------------GPIO STRUCTURE PEIPHERAL DECLARATION ----------------------------------*/

#define GPIOA 			((GPIO_Reg_Def*)GPIOA_BASE_ADDR)
#define GPIOB 			((GPIO_Reg_Def*)GPIOB_BASE_ADDR)
#define GPIOC 			((GPIO_Reg_Def*)GPIOC_BASE_ADDR)
#define GPIOD 			((GPIO_Reg_Def*)GPIOD_BASE_ADDR)
#define GPIOE 			((GPIO_Reg_Def*)GPIOE_BASE_ADDR)
#define GPIOH 			((GPIO_Reg_Def*)GPIOH_BASE_ADDR)

/*----------------------------------------------GPIO CLOCK ENABLE/DISABLE MACRO------------------------------------------------*/

#define GPIOA_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<0))
#define	GPIOB_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<1))
#define	GPIOC_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<2))
#define	GPIOD_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<3))
#define	GPIOE_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<4))
#define	GPIOH_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<7))

#define GPIOA_CLK_DI()				(RCC -> RCC_AHB1ENR &= ~(1<<0))
#define	GPIOB_CLK_DI()				(RCC -> RCC_AHB1ENR &= ~(1<<1))
#define	GPIOC_CLK_DI()				(RCC -> RCC_AHB1ENR &= ~(1<<2))
#define	GPIOD_CLK_DI()				(RCC -> RCC_AHB1ENR &= ~(1<<3))
#define	GPIOE_CLK_DI()				(RCC -> RCC_AHB1ENR &= ~(1<<4))
#define	GPIOH_CLK_DI()				(RCC -> RCC_AHB1ENR &= ~(1<<7))

#define GPIOA_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<0));	(RCC-> RCC_AHB1RSTR &= ~(1<<0));} while(0) 	// C TECHNIQUE TO USE MULTIPLE LINE IS A SINGLE MACRO
#define GPIOB_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<1));	(RCC-> RCC_AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<2));	(RCC-> RCC_AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<3));	(RCC-> RCC_AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<4));	(RCC-> RCC_AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOH_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<5));	(RCC-> RCC_AHB1RSTR &= ~(1<<5));} while(0)

/*------------------------------------------ RCC REGISTER INITIALISATION --------------------------------------------------*/

typedef struct
{

	uint32_t __vol	RCC_CR;			/* RCC clock control register OFFSET 0X00*/
	uint32_t __vol	RCC_PLLCFGR;	/* RCC PLL configuration register offset 0x04*/
	uint32_t __vol	RCC_CFGR;		/* RCC clock configuration register OFFSET 0X08*/
	uint32_t __vol	RCC_CIR;		/* RCC clock interrupt register OFFSET 0X0C*/
	uint32_t __vol	RCC_AHB1RSTR;	/* RCC AHB1 peripheral reset register OFFSET 0X10*/
	uint32_t __vol	RCC_AHB2RSTR;	/* RCC AHB2 peripheral reset register OFFSET 0X14*/
	uint32_t 		RESERVED1;		/* RESERVED AT OFFSET 0X18*/
	uint32_t        RESERVED2;		/* RESERVED AT OFFSET 0X1C*/
	uint32_t __vol	RCC_APB1RSTR;	/* RCC APB1 peripheral reset register OFFSET 0X20*/
	uint32_t __vol	RCC_APB2RSTR;	/* RCC APB2 peripheral reset register OFFSET 0X24*/
	uint32_t        RESERVED3;		/* RESERVED AT OFFSET 0X28*/
	uint32_t        RESERVED4;		/* RESERVED AT OFFSET 0X2C*/
	uint32_t __vol	RCC_AHB1ENR;	/* RCC AHB1 peripheral clock enable register OFFSET 0X30*/
	uint32_t __vol	RCC_AHB2ENR;	/* RCC AHB2 peripheral clock enable register OFFSET 0X34*/
	uint32_t		RESERVED5;		/* RESERVED AT OFFSET 0X38*/
	uint32_t		RESERVED6;		/* RESERVED AT OFFSET 0X3C*/
	uint32_t __vol	RCC_APB1ENR;	/* RCC APB1 peripheral clock enable register OFFSET 0X40*/
	uint32_t __vol	RCC_APB2ENR;	/* RCC APB2 peripheral clock enable register OFFSET 0X44*/
	uint32_t		RESERVED7;		/* RESERVED AT OFFSET 0X48*/
	uint32_t 		RESERVED8;		/* RESERVED AT OFFSET 0X4C*/
	uint32_t __vol	RCC_AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register OFFSET 0X50*/
	uint32_t __vol	RCC_AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register OFFSET 0X54 */
	uint32_t        RESERVED9;		/* RESERVED AT OFSET 0X58*/
	uint32_t		RESERVED10;		/* RESERVED AT OFFSET 0X5C*/
	uint32_t __vol	RCC_APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode register OFFSET 0X60*/
	uint32_t __vol  RCC_APB2LPENR;	/* RCC APB2 peripheral clock enabled in low power mode register OFFSET 0X64	*/
	uint32_t    	RESERVED11;		/* RESERVED AT OFFSET 0X68*/
	uint32_t		RESERVED12;		/*RESERVED AT OFFSET 0X6C*/
	uint32_t __vol  RCC_BDCR;		/* RCC Backup domain control register OFFSET 0X70*/
	uint32_t __vol  RCC_CSR;		/* RCC clock control & status register OFFSET 0X74*/
	uint32_t		RESERVED13;		/* RESERVED AT OFFSET 0X78*/
	uint32_t		RESERVED14;		/* RESERVED AT OFFSET 0X7C*/
	uint32_t __vol  RCC_SSCGR;		/* RCC spread spectrum clock generation register OFFSET 0X80*/
	uint32_t __vol  RRCC_PLLI2SCFGR;/* RCC PLLI2S configuration register OFFSET 0X84*/
	uint32_t		RESERVED15;		/* RESERVED AT OFFSET 0X88*/
	uint32_t __vol  RCC_DCKCFGR;	/* RCC Dedicated Clocks Configuration Register OFFSET 0X8C*/

}RCC_Reg_Def;

#define RCC    		((RCC_Reg_Def *)RCC_BASE_ADDR)



/*---------------------------------------------EXTERNAL INTTERUPT MEMORY STRUCTURE---------------------------------------------------------*/

typedef struct
{
	uint32_t __vol EXTI_IMR;		//Interrupt mask register at OFFSET OXOO
	uint32_t __vol EXTI_EMR;		//Event mask register AT OFFSET 0X04
	uint32_t __vol EXTI_RTSR;		// Rising trigger selection register AT OFFSET 0X08
	uint32_t __vol EXTI_FTSR;		//Falling trigger selection register at offset 0x0c
	uint32_t __vol EXTI_SWIER;		//Software interrupt event register AT OFFSET 0X10
	uint32_t __vol EXTI_PR;			// Pending Registor at offset 0x14

}EXTI_Reg_Def;

#define EXTI 		((EXTI_Reg_Def *)EXTI_BASE_ADDR)

/*------------------------------------------------SYSCONFIG MEMORY STRUCTURE------------------------------------------------------------------*/

#define SYSCFG_BASE_ADDR		(APB1_BASE_ADDR + 3800)
typedef struct
{
	uint32_t __vol	SYSCFG_MEMRMP;		//SYSCFG memory remap register AT OFFSET 0X00
	uint32_t __vol	SYSCFG_PMC;			// SYSCFG peripheral mode configuration register at offset 0x04
	uint32_t __vol	SYSCFG_EXTICR1;		//SYSCFG external interrupt configuration register 1 at offset 0x08
	uint32_t __vol	SYSCFG_EXTICR2;		//SYSCFG external interrupt configuration register 2 at offset 0x0c
	uint32_t __vol	SYSCFG_EXTICR3;		//SYSCFG external interrupt configuration register 3 at offset 0x10
	uint32_t __vol	SYSCFG_EXTICR4;		//SYSCFG external interrupt configuration register 4 at offset 0x14
	uint32_t RESERVED1;					// reserved location at 0x18
	uint32_t RESERVED2;					// RESERVED LOCATION AT 0X1C
	uint32_t __vol	SYSCFG_CMPCR;

}SYSCONFIG_Reg_Def;

#define SYSCFG					((SYSCONFIG_Reg_Def*)SYSCFG_BASE_ADDR)
#define SYSCFG_CLK_EN()			(RCC->RCC_APB2ENR |= (1<<14))
				// THESE VALUES ARE VLAUES OF INTERRUPT VECTOR VAILBALE ON THE VECTOR TABLE OF REFERENCE MANUAL.
#define IRQ_N_EXTI0			6
#define IRQ_N_EXTI1	 		7
#define IRQ_N_EXTI2			8
#define IRQ_N_EXTI3			9
#define IRQ_N_EXTI4			10
#define IRQ_N_EXTI9_5		23
#define IRQ_N_EXTI15_10		40
#define IRQ_N_EXTI17		41

/*
 *
 *
 * --------------------------------------------------------SPI PHERIPHERAL STRUCTURE ---------------------------------------------------------
 *
 *
 * */

/*TODO : Bit position definition needs to be done*/

// SPI IRQ numbers from vector Table
#define IRQ_N_SPI1			34
#define IRQ_N_SPI2			35
#define IRQ_N_SPI3			51
#define IRQ_N_SPI4			84
#define IRQ_N_SPI5			85

typedef struct
{
	uint32_t __vol	SPI_CR1;		// Info from reference manual
	uint32_t __vol	SPI_CR2;
	uint32_t __vol	RESERVED0;
	uint32_t __vol	SPI_SR;
	uint32_t __vol	SPI_DR;
	uint32_t __vol	SPI_CRCPR;
	uint32_t __vol	SPIRXCRCR;
	uint32_t __vol	SPITXCRCR;
	uint32_t __vol	SPI_I2SCFGR;
	uint32_t __vol	SPI_I2CPR;

}SPI_RegDef_t;

		// Structure peripheral declaration macro
#define SPI1	((SPI_RegDef_t *)SPI1_BASE_ADDR)
#define SPI2	((SPI_RegDef_t *)SPI2_BASE_ADDR)
#define SPI3	((SPI_RegDef_t *)SPI3_BASE_ADDR)
#define SPI4	((SPI_RegDef_t *)SPI4_BASE_ADDR)
#define SPI5	((SPI_RegDef_t *)SPI5_BASE_ADDR)

		// SPI Clock initialization macros
#define SPI1_Clock_Init()		(RCC->RCC_APB2ENR |= (1 << 12))		// Info  from reference manual
#define SPI2_Clock_Init()		(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_Clock_Init()		(RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_Clock_Init()		(RCC->RCC_APB2ENR |= (1 << 13))
#define SPI5_Clock_Init()		(RCC->RCC_APB2ENR |= (1 << 20))

		// SPI clock De-initialize macro
#define SPI1_Clock_De()			(RCC->RCC_APB2ENR &= ~(1 << 12))		// Info  from reference manual
#define SPI2_Clock_De()			(RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_Clock_De()			(RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_Clock_De()			(RCC->RCC_APB2ENR &= ~(1 << 13))
#define SPI5_Clock_De()			(RCC->RCC_APB2ENR &= ~(1 << 20))

		// SPI_Config_t ->@SPI_MODE
#define SPI_MODE_MASTER		0		// Master Mode
#define SPI_MODE_SLAVE		1		// Slave mode
		// SPI_Config_t -> @SPI_Bus_Config
#define SPI_BUS_CONFIG_FD			0	// full duplex mode
#define SPI_BUS_CONFIG_HD			1	// Half duplex mode
#define SPI_BUS_CONFIG_SIMPLEX_TX	2	// simplex with send only
#define SPI_BUS_CONFIG_SIMPLEX_RX	3	// simplex with recieve only

#define SPI_BUS_BIDIMODE_0	0			// Check CR1_register from reference manual for clarity.
#define SPI_BUS_BIDIMODE_1	1

		//SPI_Config_t -> @SPI_Clk_Speed
#define SPI_CLOCK_SPEED_DIV2		0	// check baud rate control register in SPI_CR1
#define SPI_CLOCK_SPEED_DIV4		1
#define SPI_CLOCK_SPEED_DIV8		2
#define SPI_CLOCK_SPEED_DIV16		3
#define SPI_CLOCK_SPEED_DIV32		4
#define SPI_CLOCK_SPEED_DIV64		5
#define SPI_CLOCK_SPEED_DIV128		6
#define SPI_CLOCK_SPEED_DIV256		7
	// SPI_Config_t -> @SPI_Dff
#define SPI_DFF_8BIT				0	// 8bit mode
#define SPI_DFF_16BIT				1	// 16bit mode
	// SPI_Config-> @SPI_CPOL
#define SPI_CPOL_0					0	//idle at 0
#define SPI_CPOL_1					1	// idle at 1
	//SPI_Config-> SPI_CPHA
#define SPI_CPHA_0					0	// first clock transition
#define SPI_CPHA_1					1	// 2nd clock transition
		//SPI_Config-> SPI_SSM
#define SPI_SSM_DI					0	// Software slave management disabled
#define SPI_SSM_EN					1	//software slave management enabled

#define SPI1_Clock_Reset()			do{RCC->RCC_APB2RSTR |= (1<<12); RCC->RCC_APB2RSTR &= ~(1<<12);} while(0)
#define SPI2_Clock_Reset()			do{RCC->RCC_APB1RSTR |= (1<<14); RCC->RCC_APB1RSTR &= ~(1<<14);} while(0)
#define SPI3_Clock_Reset()			do{RCC->RCC_APB1RSTR |= (1<<15); RCC->RCC_APB1RSTR &= ~(1<<15);} while(0)
#define SPI4_Clock_Reset()			do{RCC->RCC_APB2RSTR |= (1<<13); RCC->RCC_APB2RSTR &= ~(1<<13);} while(0)
#define SPI5_Clock_Reset()			do{RCC->RCC_APB2RSTR |= (1<<20); RCC->RCC_APB2RSTR &= ~(1<<20);} while(0)


/***********************************************************************
 *
 * 	I2C Peripheral Structure Declaration.
 *
 * 	Structure varalble is in the formate of peripheral_REGdef_t format
 *
 *
 ***********************************************************************/

typedef struct
{
	uint32_t __vol	I2C_CR1;
	uint32_t __vol	I2C_CR2;
	uint32_t __vol	I2C_OAR1;
	uint32_t __vol	I2C_OAR2;
	uint32_t __vol	I2C_DR;
	uint32_t __vol	I2C_SR1;
	uint32_t __vol	I2C_SR2;
	uint32_t __vol	I2C_CCR;
	uint32_t __vol	I2C_TRISE;
	uint32_t __vol	I2C_FLTR;

}I2C_RegDef_t;

/*			Structure pointers for different I2C's			*/

#define I2C1			((I2C_RegDef_t *)I2C1_BASE_ADDR)
#define I2C2			((I2C_RegDef_t *)I2C2_BASE_ADDR)
#define I2C3			((I2C_RegDef_t *)I2C3_BASE_ADDR)
/*					Clock enable macros						*/
#define I2C1_CLOCK_EN()		(RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_CLOCK_EN()		(RCC->RCC_APB1ENR |= (1<<22))
#define I2C3_CLOCK_EN()		(RCC->RCC_APB1ENR |= (1<<23))

#define I2C1_CLOCK_DI()		(RCC->RCC_APB1ENR &= ~(1<<21))
#define I2C2_CLOCK_DI()		(RCC->RCC_APB1ENR &= ~(1<<22))
#define I2C3_CLOCK_DI()		(RCC->RCC_APB1ENR &= ~(1<<23))



/***************************************************************************
 *
 *
 * Flash Interface Register map structure init
 *
 *
 ***************************************************************************/

typedef struct
{
	uint32_t __vol	FLASH_ACR;
	uint32_t __vol	FLASH_KEYR;
	uint32_t __vol	FLASH_OPTKEYR;
	uint32_t __vol	FLASH_SR;
	uint32_t __vol	FLASH_CR;
	uint32_t __vol	FLASH_OPTCR;

}Flash_Reg_Def;

#define FLASH 	((Flash_Reg_Def *)FLASH_INTERFACE_ADDR)


/*********************************************************************************
 * PWR register Init
 * *******************************************************************************
 */


typedef struct
{
	uint32_t __vol  PWR_CR;
	uint32_t __vol	PWR_CSR;
}Pwr_Reg_Def;

#define PWR		((Pwr_Reg_Def*)PWR_BASE_ADDR)
/*
 * Timer register structure init.
 *
 */
typedef struct
{
	uint32_t __vol TIM2_CR1;  //
	uint32_t __vol TIM2_CR2;
	uint32_t __vol TIM2_SMCR;
	uint32_t __vol TIM2_DIER;
	uint32_t __vol TIM2_SR;
	uint32_t __vol TIM2_EGR;
	uint16_t __vol TIM2_CCMR1_IN;
	uint16_t __vol TIM2_CCMR1_OP;
	uint32_t __vol TIM2_CCMR2_IN;
	uint32_t __vol TIM2_CCMR2_OP;
	uint32_t __vol TIM2_CCER;
	uint32_t __vol TIM2_CNT;
	uint32_t __vol TIM2_PSC;
	uint32_t __vol TIM2_ARR;
	uint32_t __vol TIM2_CCR1;
	uint32_t __vol TIM2_CCR2;
	uint32_t __vol TIM2_CCR3;
	uint32_t __vol TIM2_CCR4;
	uint32_t __vol RESERVED1;
	uint32_t __vol TIM2_DCR;
	uint32_t __vol TIM2_DMAR;
	uint32_t __vol TIM2_OR;
	uint32_t __vol TIM5_OR;

}Tim_Reg_Def;

#define TIMER_2     ((Tim_Reg_Def *)TIMER2_BASE_ADDR)


/*
 * ADC Register Structure init
 *
 */

typedef struct
{

	uint32_t __vol ADC_SR;
	uint32_t __vol ADC_CR1;
	uint32_t __vol ADC_CR2;
	uint32_t __vol ADC_SMRP1;
	uint32_t __vol ADC_SMRP2;
	uint32_t __vol ADC_JOFR1;
	uint32_t __vol ADC_JOFR2;
	uint32_t __vol ADC_JOFR3;
	uint32_t __vol ADC_JOFR4;
	uint32_t __vol ADC_HTR;
	uint32_t __vol ADC_LTR;
	uint32_t __vol ADC_SQR1;
	uint32_t __vol ADC_SQR2;
	uint32_t __vol ADC_SQR3;
	uint32_t __vol ADC_JSQR;
	uint32_t __vol ADC_JDR1;
	uint32_t __vol ADC_JRD2;
	uint32_t __vol ADC_JRD3;
	uint32_t __vol ADC_JRD4;
	uint32_t __vol ADC_DR;

}ADC_Reg_Def;


#define ADC1  		((ADC_Reg_Def*)ADC1_BASE_ADDR)
#endif /* DRIVERS_INC_BLACKPILL_H_ */
