/*
 * stm32f411xce_spi_driver.c
 *
 *  Created on: 21 Aug 2021
 *      Author: bengr
 */

#include "stm32f411xce.h"

/**
 *
 */
void SPI_PClockControl(SPI_Reg_Def_t *pSPIx, uint8_t EnorDI){
	if(EnorDI == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}else if(pSPIx == SPI5){
			SPI5_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}else if(pSPIx == SPI5){
			SPI5_PCLK_DI();
		}
	}
}



/**
 *
 */
void SPI_Init(SPI_Handler_t *pSPIHandler){
	// Config SPI_CR1
	uint32_t cr1_tempreg = 0;

	SPI_PClockControl(pSPIHandler->SPIx, ENABLE);

	cr1_tempreg |= (pSPIHandler->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	if(pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// Clear BIDIMode So 2-line unidirectional data mode selected
		cr1_tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// Set BIDIMode So 1-line bidirectional data mode selected
		cr1_tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RX){
		// Clear BIDIMode So 2-line unidirectional data mode selected
		cr1_tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// Set RXONLY
		cr1_tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// Clr current clk val and replace with val in handler
	cr1_tempreg |= (pSPIHandler->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// Set DFF Val
	cr1_tempreg |= (pSPIHandler->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// Set CPOL
	cr1_tempreg |= (pSPIHandler->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Set CPHA
	cr1_tempreg |= (pSPIHandler->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// Set SSM
	cr1_tempreg |= (pSPIHandler->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandler->SPIx->CR1 = cr1_tempreg;
}

/**
 *
 */
void SPI_DeInit(SPI_Reg_Def_t *pSPIx){

}



/**
 * Blocking call to send data (Wait for all bytes to be sent before return)
 */
void SPI_Send_Data(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){

	while(len-- > 0){
		// Wait For Transmit Buffer To Be Empty
		while(pSPIx->SR | (1 << SPI_SR_TXE));

		// If sending 16 bit else send 8 bit (DR is like a window into RX & TX buffer
		// 		when reading from DR it will read from RX buffer when writing will
		// 		write to TX buffer
		if(!(pSPIx->CR1 | (1 << SPI_CR1_DFF))){
			// Load DR with 16 bit
			pSPIx->DR = *((uint16_t*)(pTxBuffer));
			// 16 bits so decrement for 2nd time
			len--;
			(uint16_t*)pTxBuffer++;
		}else{
			// Load DR with 8 bit
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
		}
	}

}

/**
 *
 */
void SPI_Receive_Data(SPI_Reg_Def_t *pSPIx){

}



/**
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}

/**
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}

/**
 *
 */
void SPI_IRQHandling(SPI_Handler_t *pHandle){

}

/*
 *
 */
void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
