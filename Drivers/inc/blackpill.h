/*
 * blackpill.h
 *
 *  Created on: Sep 15, 2021
 *      Author: Gaurav
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
/*-------------------------------------------------Processor specific details--------------------------------------------*/

#define NVIC_ISER0		((__vol uint32_t *) 0xE000E100)
#define NVIC_ISER1		((__vol uint32_t *) 0xE000E104)
#define NVIC_ISER2		((__vol uint32_t *) 0xE000E108)
#define NVIC_ISER3		((__vol uint32_t *) 0xE000E10c)

#define NVIC_ICER0		((__vol uint32_t *) 0XE000E180)
#define NVIC_ICER1		((__vol uint32_t *) 0XE000E184)
#define NVIC_ICER2		((__vol uint32_t *) 0XE000E188)
#define NVIC_ICER3		((__vol uint32_t *) 0XE000E18C)


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


/*---------------------------------PERIPHERALS ON AHB1 BUS--------------------------------------------------------*/

#define GPIOA_BASE_ADDR				AHB1_BASE_ADDR
#define GPIOB_BASE_ADDR				0x40020400UL
#define GPIOC_BASE_ADDR				0x40020800UL
#define GPIOD_BASE_ADDR				0x40020C00UL
#define GPIOE_BASE_ADDR				0x40021000UL
#define GPIOH_BASE_ADDR				0x40021C00UL

/*------------------------------ PERIPHERALS ON APB2 PERIPHERALS-------------------------------------------------*/

#define USART1_BASE_ADDR 			0x40011000UL    /*  REFERENCE MANUAL -> MEMORY MAP */
#define USART6_BASE_ADDR			0x40011400UL
#define SPI1_BASE_ADDR				0x40013000UL
#define I2S1_BASE_ADDR				SPI1_BASE_ADDR
#define SPI4_BASE_ADDR				0x40013400UL
#define I2S4_BASE_ADDR				SPI4_BASE_ADDR
#define EXTI_BASE_ADDR				0x40013C00UL
#define SPI5_BASE_ADDR				0x40015000UL
#define I2S5_BASE_ADDR				SPI5_BASE_ADDR

/*----------------------------------APB1 PERIPHERALS------------------------------------------------------------*/

#define SPI2_BASE_ADDR				0x40003800UL
#define I2S2_BASE_ADDR				SPI2_BASE_ADDR
#define SPI3_BASE_ADDR				0x40003C00UL
#define I2S3_BASE_ADDR				SPI3_BASE_ADDR
#define UART2_BASE_ADDR				0x40004400UL
#define I2C1_BASE_ADDR				0x40004400UL
#define I2C2_BASE_ADDR				0x40005400UL
#define I2C3_BASE_ADDR				0x40005800UL

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

/*--------------------------------------------------GPIO STRUCTURE INTILIZATION----------------------------------*/

#define GPIOA 			((GPIO_Reg_Def*)GPIOA_BASE_ADDR)
#define GPIOB 			((GPIO_Reg_Def*)GPIOB_BASE_ADDR)
#define GPIOC 			((GPIO_Reg_Def*)GPIOC_BASE_ADDR)
#define GPIOD 			((GPIO_Reg_Def*)GPIOD_BASE_ADDR)
#define GPIOE 			((GPIO_Reg_Def*)GPIOE_BASE_ADDR)
#define GPIOH 			((GPIO_Reg_Def*)GPIOH_BASE_ADDR)

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



/*----------------------------------------------CLOCK ENABLE AND DISABLE MACROS ------------------------------------------------------------*/

#define GPIOA_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<0))
#define	GPIOB_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<1))
#define	GPIOC_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<2))
#define	GPIOD_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<3))
#define	GPIOE_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<4))
#define	GPIOH_CLK_EN()				(RCC -> RCC_AHB1ENR |= (1<<7))

#define GPIOA_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<0));	(RCC-> RCC_AHB1RSTR &= ~(1<<0));} while(0) 	// C TECHNIQUE TO USE MULTIPLE LINE IS A SINGLE MACRO
#define GPIOB_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<1));	(RCC-> RCC_AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<2));	(RCC-> RCC_AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<3));	(RCC-> RCC_AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<4));	(RCC-> RCC_AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOH_CLK_RESET()			do {(RCC-> RCC_AHB1RSTR |= (1<<5));	(RCC-> RCC_AHB1RSTR &= ~(1<<5));} while(0)





#endif /* DRIVERS_INC_BLACKPILL_H_ */
