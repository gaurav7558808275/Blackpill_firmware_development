/*
 * i2c.h
 *
 *  Created on: 28 Sep. 2021
 *      Author: Gaurav
 */

#ifndef DRIVERS_INC_I2C_H_
#define DRIVERS_INC_I2C_H_

#include "blackpill.h"

/*
 *		SCL Speed Setup.
 *		Its 100KHz for std mode
 *		Its 400kHz for speed mode
 *
 */
#define I2C_SCL_SM_SPEED	100000
#define I2C_SCL_PM_SPEED	400000
/*
 * 	ACK CONTROL
 *
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * 	Duty control
 * 	DUTY: Fm mode duty cycle
	0: Fm mode tlow/thigh = 2
	1: Fm mode tlow/thigh = 16/9 (see CCR)
 *
 */
#define I2C_FMDUTY_2		0
#define I2C_FMDUTY16_9		1

typedef struct
{
	uint32_t	I2C_SCL_Speed;
	uint32_t	I2C_Device_Addr;
	uint32_t	I2C_ACKControl;
	uint32_t	I2C_FMduty;

}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;

}I2C_Handle_t;


#endif /* DRIVERS_INC_I2C_H_ */
