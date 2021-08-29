/*
 * stm32f411xce_spi_driver.c
 *
 *  Created on: 21 Aug 2021
 *      Author: bengr
 */

#include "stm32f411xce.h"

static void SPI_Txe_It_Handle(SPI_Handler_t *pHandler);
static void SPI_Rxne_It_Handle(SPI_Handler_t *pHandler);
static void SPI_OVR_It_Handle(SPI_Handler_t *pHandler);

/**
 *
 */
void SPI_PClockControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
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

void SPI_SSIControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
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
		while((pSPIx->SR & (1 << SPI_SR_TXE)) == 0);

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
void SPI_Receive_Data(SPI_Reg_Def_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){

	while(len-- > 0){
		// Wait For Receive Buffer To Be Empty
		while((pSPIx->SR & (1 << SPI_SR_RXNE)) == 0);

		// If receiving 16 bit else receiving 8 bit (DR is like a window into RX & TX buffer
		// 		when reading from DR it will read from RX buffer when writing will
		// 		write to TX buffer
		if(!(pSPIx->CR1 | (1 << SPI_CR1_DFF))){
			// Read 16 bit from DR
			*((uint16_t*)(pRxBuffer)) = pSPIx->DR;
			// 16 bits so decrement for 2nd time
			len--;
			(uint16_t*)pRxBuffer++;
		}else{
			// Read 8 bit from DR
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
		}
	}
}



/**
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
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

/**
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

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

uint8_t SPI_Send_Data_IT(SPI_Handler_t *pSPIHandler, uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandler->TxState;

	if(state != SPI_BUSY_IN_TX){
		pSPIHandler->pTxBuffer = pTxBuffer;
		pSPIHandler->TxLen = len;

		pSPIHandler->TxState = SPI_BUSY_IN_TX;

		pSPIHandler->SPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}

uint8_t SPI_Receive_Data_IT(SPI_Handler_t *pSPIHandler, uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandler->RxState;

		if(state != SPI_BUSY_IN_RX){
			pSPIHandler->pRxBuffer = pRxBuffer;
			pSPIHandler->RxLen = len;

			pSPIHandler->RxState = SPI_BUSY_IN_RX;

			pSPIHandler->SPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		}

		return state;
}

void SPI_IRQHandling(SPI_Handler_t *pHandler){
	uint8_t temp1, temp2;
	temp1 = pHandler->SPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandler->SPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){
		// Deal with TXE
		SPI_Txe_It_Handle(pHandler);
	}

	temp1 = pHandler->SPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandler->SPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		// Deal with TXE
		SPI_Rxne_It_Handle(pHandler);
	}

	// Check OVR Err
	temp1 = pHandler->SPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandler->SPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		// Deal with TXE
		SPI_OVR_It_Handle(pHandler);
	}
}

/*
 * Private Helper Functions
 */
static void SPI_Txe_It_Handle(SPI_Handler_t *pHandler){
	// If sending 16 bit else send 8 bit (DR is like a window into RX & TX buffer
	// 		when reading from DR it will read from RX buffer when writing will
	// 		write to TX buffer
	if(!(pHandler->SPIx->CR1 | (1 << SPI_CR1_DFF))){
		// Load DR with 16 bit
		pHandler->SPIx->DR = *((uint16_t*)(pHandler->pTxBuffer));
		// 16 bits so decrement 2 times
		pHandler->TxLen -= 2;
		(uint16_t*)pHandler->pTxBuffer++;
	}else{
		// Load DR with 8 bit
		pHandler->SPIx->DR = *pHandler->pTxBuffer;
		pHandler->pTxBuffer++;
		// 8 Bits so decrement len 1 time
		pHandler->TxLen--;
	}

	if(pHandler->TxLen <= 0){
		SPI_Close_Transmission(pHandler);
		SPI_Even_Application_Callback(pHandler, SPI_EVENT_TX_CMPLT);
	}
}

static void SPI_Rxne_It_Handle(SPI_Handler_t *pHandler){
	// If receiving 16 bit else receiving 8 bit (DR is like a window into RX & TX buffer
	// 		when reading from DR it will read from RX buffer when writing will
	// 		write to TX buffer
	if(!(pHandler->SPIx->CR1 | (1 << SPI_CR1_DFF))){
		// Read 16 bit from DR
		*((uint16_t*)(pHandler->pRxBuffer)) = pHandler->SPIx->DR;
		// 16 bits so decrement for 2nd time
		pHandler->RxLen -= 2;
		(uint16_t*)pHandler->pRxBuffer++;
	}else{
		// Read 8 bit from DR
		*pHandler->pRxBuffer = pHandler->SPIx->DR;
		pHandler->pRxBuffer++;
		pHandler->RxLen--;
	}

	if(pHandler->RxLen <= 0){
		SPI_Close_Reception(pHandler);

		SPI_Even_Application_Callback(pHandler, SPI_EVENT_RX_CMPLT);
	}
}

static void SPI_OVR_It_Handle(SPI_Handler_t *pHandler){
	// CLR Flag

	if(pHandler->TxState != SPI_BUSY_IN_TX){
		SPI_CLR_OVR_Flag(pHandler->SPIx);
	}

	// Inform Application
	SPI_Even_Application_Callback(pHandler, SPI_EVENT_OVR_CMPLT);
}

void SPI_CLR_OVR_Flag(SPI_Reg_Def_t *pSPIx){
	uint32_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void) temp;
}

void SPI_Close_Transmission(SPI_Handler_t *pHandler){
	// Close SPI TX Communication & Inform TX Over
	// Clear TXE To Prevent Future TX It
	pHandler->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	// Clear Global Data
	pHandler->pTxBuffer = NULL;
	pHandler->TxLen = 0;
	pHandler->TxState = SPI_READY;
}

void SPI_Close_Reception(SPI_Handler_t *pHandler){
	pHandler->SPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	pHandler->pRxBuffer = NULL;
	pHandler->RxState = SPI_READY;
	pHandler->RxLen = 0;
}


/*
 * Weak Implementation
 */
__weak void SPI_Even_Application_Callback(SPI_Handler_t *pHandler, uint8_t appEvent){
	// Application may override this
}


















