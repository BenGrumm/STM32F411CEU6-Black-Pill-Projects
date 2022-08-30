/*
 * stm32f411xce_i2c_driver.c
 *
 *  Created on: 21 Jul 2022
 *      Author: bengr
 */

#include "stm32f411xce_i2c_driver.h"

uint16_t AHBPreValues[8] = {2, 4, 8, 16, 64, 128, 256, 512};

static void I2C_GenerateStartCondition(I2C_Reg_Def_t *pI2CRegDef);
static void I2C_ExecuteAddressPhase(I2C_Reg_Def_t *pI2Cx, uint8_t addr);
static void I2C_ClearAddrFlag(I2C_Reg_Def_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_Reg_Def_t *pI2Cx);


/**
 *
 */
void I2C_PClockControl(I2C_Reg_Def_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_PLLGetOutputCLK(){
	// Confirm pll is used
	if(((RCC->CFGR >> RCC_CFGR_SWS) & (uint8_t)0b11) == 0b10){

		uint32_t pllInputSpeed = 0;

		if(((RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC) & 0b1) == 0b0){
			// PLL Clk Src is HSI
			pllInputSpeed = 16000000;
		}else{
			// PLL Clk Src is HSE
			pllInputSpeed = 25000000;
		}

		// f(PLL general clock output) = (f(PLL clock input) × PLLN) / (PLLM * PLLP)
		uint16_t PLLN = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLN) & 0x1FF; // 9 bits used for PLLN
		uint8_t PLLM = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLM) & 0x3F; // 5 bits used for PLLM
		uint8_t PLLP = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLP) & 0x3; // 2 bits used for PLLP

		return (pllInputSpeed * PLLN) / (PLLM * PLLP);
	}else{
		return 0;
	}
}

uint32_t RCC_GetPCLK1Value(void){
	uint32_t sysClock = 0;

	// Get first 2 bits to find current clock in use
	uint8_t clockType = (RCC->CFGR >> RCC_CFGR_SWS) & (uint8_t)0b11;

	// 00: HSI oscillator used as the system clock
	// 01: HSE oscillator used as the system clock
	// 10: PLL used as the system clock
	if(clockType == (uint8_t)0b00){
		// HSI
		sysClock = 16000000;
	}else if(clockType == (uint8_t)0b01){
		// HSE
		sysClock = 25000000;
	}else if(clockType == (uint8_t)0b10){
		// PLL
		sysClock = RCC_PLLGetOutputCLK();
	}

	uint8_t AHBPrescaler = (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF;
	uint8_t APB1Prescaler = (RCC->CFGR >> RCC_CFGR_PPRE1) & 0b111;

	uint8_t AHBPreValue = 1;
	uint8_t APB1PreValue = 1;

	// If 4th bit = 0 then signal divided by 1 no matter other bits
	if(AHBPrescaler >= 8){
		AHBPreValue = AHBPreValues[(AHBPrescaler & 0b0111)];
	}

	// If 3rd bit = 0 then signal is divided by 1 no matter other bits
	if(APB1Prescaler >= 4){
		APB1PreValue = AHBPreValues[(APB1Prescaler & 0b011)]; // Same values for first 4 which are used by APB1
	}

	return (sysClock / AHBPreValue) / APB1PreValue;
}

/**
 *
 */
void I2C_Init(I2C_Handler_t *pI2CHandler){
	uint32_t tempreg = 0;

	// Enable acking
	pI2CHandler->pI2Cx->CR1 |= pI2CHandler->I2C_Config->I2C_ACK_Control << I2C_CR1_ACK;

	// Config PCLK value in CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U; // in MHz
	pI2CHandler->pI2Cx->CR2 |= (tempreg & 0x3F); // 5 bits used for FREQ

	// Config device address (when slave device)
	tempreg = 0;
	tempreg |= (pI2CHandler->I2C_Config->I2C_Device_Adress & 0x7F) << 1; // 7 bit addressing goes from bits 7:1
	tempreg |= 1 << 14; // 14th bit must be set (check ref man)
	pI2CHandler->pI2Cx->OAR1 = tempreg;

	// Config Mode (fast or standard) and speed of serial clock (higher freq need shorter wires)
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandler->I2C_Config->I2C_SCL_Speed <= I2C_Speed_SM){
		// Config standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandler->I2C_Config->I2C_SCL_Speed);
	}else{
		// Config fast mode
		pI2CHandler->pI2Cx->CCR |= 1 << I2C_CCR_FS;
		pI2CHandler->pI2Cx->CCR |= pI2CHandler->I2C_Config->I2C_FM_Duty_Cycle << I2C_CCR_DUTY;

		if(pI2CHandler->I2C_Config->I2C_FM_Duty_Cycle == I2C_FM_DUTY_2){
			// If I2C_FM_Duty_Cycle is 0 or T_high = CCR * TPCLK1 and T_low = 2 * CCR * TPCLK1
			// (1 / pI2CHandler->I2C_Config->I2C_SCL_Speed) = 3 * CCR * (1 / RCC_GetPCLK1Value());
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandler->I2C_Config->I2C_SCL_Speed);
		}else{
			// If I2C_FM_Duty_Cycle is 1 or T_high = 9 * CCR * TPCLK1 and T_low = 16 * CCR * TPCLK1
			// (1 / pI2CHandler->I2C_Config->I2C_SCL_Speed) = 25 * CCR * (1 / RCC_GetPCLK1Value());
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandler->I2C_Config->I2C_SCL_Speed);
		}
	}
	pI2CHandler->pI2Cx->CCR |= (ccr_value & 0xFFF); // 12 bits used for ccr_value

	// Configure rise time for i2c pins TRISE TODO
	if(pI2CHandler->I2C_Config->I2C_SCL_Speed <= I2C_Speed_SM){
		// Standard mode has max rise time of 1,000ns
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1; // Documentation shows T we use F
	}else{
		// Fast mode has max rise time of 300ns
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandler->pI2Cx->TRISE |= (tempreg & 0x3F);
}

/**
 *
 */
uint8_t I2C_GetFlagStatus(I2C_Reg_Def_t *pI2Cx, uint8_t flagName){
	if(pI2Cx->SR1 & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**
 *
 */
void I2C_MasterSendData(I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t txBufferLen, uint8_t slaveAddr){
	I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

	// Confirm start condition completed
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SB));

	I2C_ExecuteAddressPhase(pI2CHandler->pI2Cx, slaveAddr);

	// Confirm address phase completed
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR));

	I2C_ClearAddrFlag(pI2CHandler->pI2Cx);

	// Send all data
	while(txBufferLen > 0){
		while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TXE)); // Wait until TXE set (when Data Register empty)
		pI2CHandler->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		txBufferLen--;
	}

	// TXE and BTF must be set before generating stop condition
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_BTF));

	I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
}

/**
 *
 */
static void I2C_GenerateStartCondition(I2C_Reg_Def_t *pI2Cx){
	pI2Cx->CR1 |= 1 << I2C_CR1_START;
}

/**
 *
 */
static void I2C_ExecuteAddressPhase(I2C_Reg_Def_t *pI2Cx, uint8_t addr){
	addr = addr << 1; // Make space for r/w bit
	addr &= ~1; // Set r/w bit to 0
	pI2Cx->DR = addr;
}

/**
 * Addr flag is cleared by reading SR1 and SR2
 */
static void I2C_ClearAddrFlag(I2C_Reg_Def_t *pI2Cx){
	uint32_t dummy = pI2Cx->SR1;
	dummy = pI2Cx->SR2;
	(void)dummy; // Just for compiler warning
}

/**
 *
 */
static void I2C_GenerateStopCondition(I2C_Reg_Def_t *pI2Cx){
	pI2Cx->CR1 |= 1 << I2C_CR1_STOP;
}

/**
 *
 */
void I2C_DeInit(I2C_Reg_Def_t *pI2Cx){
	// Using reset values to de init
	pI2Cx->CR1 = 0x0000;
	pI2Cx->CR2 = 0x0000;
	pI2Cx->OAR1 = 0x0000;
	pI2Cx->OAR1 |= 1 << 14; // ?? Say bit 14 must be kept 1 by software
	pI2Cx->OAR2 = 0x0000;
	pI2Cx->DR = 0x0000;
	pI2Cx->SR1 = 0x0000;
	pI2Cx->SR2 = 0x0000;
	pI2Cx->CCR = 0x0000;
	pI2Cx->TRISE = 0x0002;
	pI2Cx->FLTR = 0x0000;
}

/**
 *
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(IRQNumber >= 32 && IRQNumber <= 63){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber <= 95){
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}else{
		if(IRQNumber >= 32 && IRQNumber <= 63){
			*NVIC_ISER1 &= ~(1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber <= 95){
			*NVIC_ISER2 &= ~(1 << (IRQNumber % 32));
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*((__vo uint32_t*)(NVIC_PR_BASE_ADDR + (iprx * 4))) |= (IRQPriority << shift_amount);
}

void I2C_PeripheralControl(I2C_Reg_Def_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pI2Cx->CR1 |= 1 << I2C_CR1_PE;
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
