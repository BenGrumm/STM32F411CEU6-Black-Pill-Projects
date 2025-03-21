/*
 * stm32f411xce_spi_driver.h
 *
 *  Created on: 21 Aug 2021
 *      Author: bengr
 */

#ifndef INC_STM32F411XCE_SPI_DRIVER_H_
#define INC_STM32F411XCE_SPI_DRIVER_H_

#include <stdint.h>
#include "stm32f411xce.h"

/*
 * Configuration Settings For The SPIx Periph
 */
typedef struct {
	uint8_t SPI_DeviceMode;						/* Device Can Be Master Or Slave 								*/
	uint8_t SPI_BusConfig;						/* Full Duplex, Half Duplex, Simplex 							*/
	uint8_t SPI_SclkSpeed;						/* What Bus Clock Division Is (min 2)							*/
	uint8_t SPI_DFF;							/* Data Frame Format (Shift Reg Is 8 or 16 bit)					*/
	uint8_t SPI_CPOL;							/* Clock Polarity When IDLE (High Or Low)						*/
	uint8_t SPI_CPHA;							/* Clock Phase (Which Edge Is Data Capture)						*/
	uint8_t SPI_SSM;							/* Software Slave Management (If SS bit is external controlled)	*/
} SPI_Config_t;

/*
 * Handler For Setting Up SPIx Periph
 */
typedef struct {
	SPI_Reg_Def_t 	*SPIx;
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
} SPI_Handler_t;

/*
 * SPI Application States
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_CMPLT	3
#define SPI_EVENT_CRC_CMPLT	4

/*
 * @SPI_DeviceMode	(Default Slave)
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD		0
#define SPI_BUS_CONFIG_HD		1
#define SPI_BUS_CONFIG_S_RX		3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/******************************************************************************************
 * API For Driver
 *****************************************************************************************/
void SPI_PClockControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi);

void SPI_Init(SPI_Handler_t *pSPIHandler);
void SPI_DeInit(SPI_Reg_Def_t *pSPIx);

void SPI_Send_Data(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_Receive_Data(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

uint8_t SPI_Send_Data_IT(SPI_Handler_t *pSPIHandler, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_Receive_Data_IT(SPI_Handler_t *pSPIHandler, uint8_t *pTxBuffer, uint32_t len);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handler_t *pHandler);

void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi);

void SPI_CLR_OVR_Flag(SPI_Reg_Def_t *pSPIx);
void SPI_Close_Transmission(SPI_Handler_t *pHandler);
void SPI_Close_Reception(SPI_Handler_t *pHandler);

/**
 * Application Callbacks
 */
void SPI_Event_Application_Callback(SPI_Handler_t *pHandler, uint8_t appEvent);

#endif /* INC_STM32F411XCE_SPI_DRIVER_H_ */
