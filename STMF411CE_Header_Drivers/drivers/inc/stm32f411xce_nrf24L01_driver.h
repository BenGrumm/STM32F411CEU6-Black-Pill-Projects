/*
 * stm32f411xce_nrf24L01_driver.h
 *
 *  Created on: 4 Sep 2021
 *      Author: bengr
 */

#ifndef INC_STM32F411XCE_NRF24L01_DRIVER_H_
#define INC_STM32F411XCE_NRF24L01_DRIVER_H_

#include <stdint.h>
#include "stm32f411xce_spi_driver.h"

/*
 * Commands
 */
#define R_REGISTER			0x00UL	/* Read command and status registers. AAAAA = 5 bit Register Map Address */
#define W_REGISTER			0x20UL	/* Write command and status registers. AAAAA = 5 bit Register Map Address Executable in power down or standby modes only */
#define R_RX_PAYLOAD		0x61UL	/* Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode. */
#define W_TX_PAYLOAD		0xA0UL	/* Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload. */
#define FLUSH_TX			0xE1UL	/* Flush TX FIFO, used in TX mode */
#define FLUSH_RX			0xE2UL	/* Flush RX FIFO, used in RX mode Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed */
#define REUSE_TX_PL			0xE3UL	/* Used for a PTX device Reuse last transmitted payload. Packets are repeatedly retransmitted as long as CE is high. TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deactivated during package transmission */
#define ACTIVATE			0x50UL	/* This write command followed by data 0x73 activates the following features: • R_RX_PL_WID • W_ACK_PAYLOAD • W_TX_PAYLOAD_NOACK A new ACTIVATE command with the same data deactivates them again. This is executable in power down or stand by modes only. */
#define ACTIVATE_BYTE		0x73UL
#define R_RX_PL_WID			0x60UL	/* Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO. */
#define W_ACK_PAYLOAD		0xA8UL	/* Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101). Maximum three ACK packet payloads can be pending. Payloads with same PPP are handled using first in - first out principle. Write payload: 1– 32 bytes. A write operation always starts at byte 0. */
#define W_TX_PAYLOAD_NO_ACK	0xB0UL	/* Used in TX mode. Disables AUTOACK on this specific packet. */
#define NOP					0xFFUL	/*  No Operation. Might be used to read the STATUS register */

/*
 * Register Addresses
 */
#define CONFIG				0x00UL	/* Configuration Register */
#define EN_AA				0x01UL	/* Enable ‘Auto Acknowledgment’ Function Disable this functionality to be compatible with nRF240 */
#define EN_RXADDR			0x02UL	/* Enabled RX Addresses */
#define SETUP_AW			0x03UL	/* Setup of Address Widths (common for all data pipes) */
#define SETUP_RETR			0x04UL	/* Setup of Automatic Retransmission */
#define RF_CH				0x05UL	/* RF Channel */
#define RF_SETUP			0x06UL	/* RF Setup Register */
#define STATUS				0x07UL	/* Status Register (In parallel to the SPI command word applied on the MOSI pin, the STATUS register is shifted serially out on the MISO pin) */
#define OBSERVE_TX			0x08UL	/* Transmit observe register */
#define CD					0x09UL	/* Carrier Detect */
#define RX_ADDR_P0			0x0AUL	/* Receive address data pipe 0. 5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW) */
#define RX_ADDR_P1			0x0BUL	/* Receive address data pipe 1. 5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW) */
#define RX_ADDR_P2			0x0CUL	/* Receive address data pipe 2. Only LSB. MSBytes is equal to RX_ADDR_P1[39:8] */
#define RX_ADDR_P3			0x0DUL	/* Receive address data pipe 3. Only LSB. MSBytes is equal to RX_ADDR_P1[39:8] */
#define RX_ADDR_P4			0x0EUL	/* Receive address data pipe 4. Only LSB. MSBytes is equal to RX_ADDR_P1[39:8] */
#define RX_ADDR_P5			0x0FUL	/* Receive address data pipe 5. Only LSB. MSBytes is equal to RX_ADDR_P1[39:8] */
#define TX_ADDR				0x10UL	/* Transmit address. Used for a PTX device only. (LSByte is written first) Set RX_ADDR_P0 equal to this address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled.  */
#define RX_PW_P0			0x11UL	/* Number of bytes in RX payload in data pipe 0 */
#define RX_PW_P1			0x12UL	/* Number of bytes in RX payload in data pipe 1 */
#define RX_PW_P2			0x13UL	/* Number of bytes in RX payload in data pipe 2 */
#define RX_PW_P3			0x14UL	/* Number of bytes in RX payload in data pipe 3 */
#define RX_PW_P4			0x15UL	/* Number of bytes in RX payload in data pipe 4 */
#define RX_PW_P5			0x16UL	/* Number of bytes in RX payload in data pipe 5 */
#define FIFO_STATUS			0x17UL	/* FIFO Status Register */
#define DYNPD				0x1CUL	/* Enable dynamic payload length */
#define FEATURE				0x1DUL	/* Feature Register */

/*
 * Bit Positions
 */

#define CFG_MASK_RX_DR		6
#define CFG_MASK_TX_DS		5
#define CFG_MASK_MAX_RT		4
#define CFG_EN_CRC			3
#define CFG_CRCO			2
#define CFG_PWR_UP			1
#define CFG_PRIM_RX			0

#define ENAA_P5				5
#define ENAA_P4				4
#define ENAA_P3				3
#define ENAA_P2				2
#define ENAA_P1				1
#define ENAA_P0				0

#define ERX_P5				5
#define ERX_P4				4
#define ERX_P3				3
#define ERX_P2				2
#define ERX_P1				1
#define ERX_P0				0

#define AW					0

#define SETUP_ARD			4
#define SETUP_ARC			0

#define RF_PLL_LOCK			4
#define RF_DR				3
#define RF_PWR				1
#define RF_LNA_HCURR		0

#define STATUS_RX_DR		6
#define STATUS_TX_DS		5
#define STATUS_MAX_RT		4
#define STATUS_RX_P_NO		1
#define STATUS_TX_FULL		0

#define FEATURE_EN_DPL		2
#define FEATURE_EN_ACK_PAY	1
#define FEATURE_EN_DYN_ACK	0

/*
 * PIPE Definitions
 */
#define PIPE_0	0x00
#define PIPE_1	0x01
#define PIPE_2	0x02
#define PIPE_3	0x03
#define PIPE_4	0x04
#define PIPE_5	0x05

/*
 * Offsets
 */
#define RX_ADDR_OFFSET 0x0A

/*
 * SETUP_AW Address Widths
 */
#define BYTES_3	0b01
#define BYTES_4	0b10
#define BYTES_5	0b11

#define DUMMY_BYTE  0xF1


typedef struct {
	// Function pointers
	void (*NRF_CE_HIGH)(void);
	void (*NRF_CE_LOW)(void);
	void (*delayMs)(uint32_t);

	// General
	SPI_Handler_t *spiHandler;
	uint8_t enableDynamicPlWidth; // Dynamic payload width
	uint8_t crcEncoding;
	uint8_t addressWidth;
	uint8_t rfAirDataRate;
	uint8_t channleFrequency;
	uint8_t crcEncodingScheme;
	uint8_t rfChannel;
	uint8_t payloadWidth; // Used when pl width isnt dynamic
	uint8_t enableAutoAck; // Optional when using static pl width

	// RX
	uint8_t rxPipe;
	uint8_t isReceiver;
	uint8_t enableRXDRIRQ; // IRQ For Data Ready (DR)
	uint32_t rxAddrHigh;
	uint8_t rxAddrLow;

	// TX
	uint8_t rfOutputPower;
	uint8_t enableTXDSIRQ; // IRQ for TX Data Sent (DS)
	uint8_t enableMaxRtIRQ; // IRQ for max tx retries
	uint8_t autoRetransmitCount;
	uint8_t autoRetransmitDelay;
	uint32_t txAddrHigh;
	uint8_t txAddrLow;

}NRF24L01_Config_t;

/*
 * Functions
 */
void NRF_init(NRF24L01_Config_t *config);
void NRF_modify_reg(SPI_Handler_t *spiHandler, uint8_t reg, uint8_t bit, uint8_t enOrDi);
void NRF_activate(SPI_Handler_t *spiHandler);
void NRF_write_reg(SPI_Handler_t *spiHandler, uint8_t reg, uint8_t val);
void NRF_set_rx_addr(SPI_Handler_t *spiHandler, uint8_t rxPipe, uint32_t addrHigh, uint8_t addrLow);
void NRF_set_tx_addr(SPI_Handler_t *spiHandler, uint32_t addrHigh, uint8_t addrLow, uint8_t autoAck);
void NRF_set_rx(SPI_Handler_t *spiHandler, uint8_t enOrDi);
void NRF_listen(void);
void NRF_write_tx_payload(SPI_Handler_t *spiHandler, uint8_t *txBuffer, uint8_t len);
void NRF_clear_interrupts(SPI_Handler_t *spiHandler);
void NRF_write_tx_payload(SPI_Handler_t *spiHandler, uint8_t *txBuffer, uint8_t len);
void NRF_read_rx_payload(SPI_Handler_t *spiHandler, uint8_t *rxBuffer, uint8_t len);
void NRF_FLUSH_RX(SPI_Handler_t *spiHandler);

uint8_t NRF_ready_dynamic_pl_width(SPI_Handler_t *spiHandler);

#endif /* INC_STM32F411XCE_NRF24L01_DRIVER_H_ */
