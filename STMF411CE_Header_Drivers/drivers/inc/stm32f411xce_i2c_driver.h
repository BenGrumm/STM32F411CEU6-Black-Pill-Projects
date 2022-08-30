/*
 * stmf411xce_i2c_driver.h
 *
 *  Created on: 15 Jul 2022
 *      Author: bengr
 */

#ifndef INC_STM32F411XCE_I2C_DRIVER_H_
#define INC_STM32F411XCE_I2C_DRIVER_H_

#include "stm32f411xce.h"

typedef struct {
	uint32_t I2C_SCL_Speed;
	uint8_t I2C_Device_Adress;
	uint8_t I2C_ACK_Control;
	uint16_t I2C_FM_Duty_Cycle;
} I2C_Config_t;

typedef struct {
	I2C_Reg_Def_t *pI2Cx;
	I2C_Config_t *I2C_Config;
} I2C_Handler_t;

/**
 * I2C speed mode definitions for @I2C_SCL_Speed
 */
#define I2C_Speed_SM 		100000
#define I2C_Speed_FM4k 		400000
#define I2C_Speed_FM2k 		200000

/**
 * I2C ack control definitions for @I2C_ACK_Control
 */
#define I2C_ACK_ENABLE 		1
#define I2C_ACK_DISABLE 	0	// Default


/**
 * I2C fast mode duty cycle definitions for @I2C_FM_Duty_Cycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUT_16_9		1

/*
 * I2C flag definitions
 */
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)

/******************************************************************************************
 * API For Driver
 *****************************************************************************************/
void I2C_MasterSendData(I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t txBufferLen, uint8_t slaveAddr);

void I2C_PClockControl(I2C_Reg_Def_t *pI2Cx, uint8_t EnOrDi);

void I2C_Init(I2C_Handler_t *pI2CHandler);
void I2C_DeInit(I2C_Reg_Def_t *pI2Cx);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_PeripheralControl(I2C_Reg_Def_t *pI2Cx, uint8_t enOrDi);
uint8_t I2C_GetFlagStatus(I2C_Reg_Def_t *pI2Cx, uint8_t flagName);

void I2C_Event_Application_Callback(I2C_Handler_t *pHandler, uint8_t appEvent);

#endif /* INC_STM32F411XCE_I2C_DRIVER_H_ */
