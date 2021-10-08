/*
 * stm32f411xce_nrf24L01_driver.c
 *
 *  Created on: 15 Sep 2021
 *      Author: bengr
 */


#include "stm32f411xce_nrf24L01_driver.h"

uint8_t dummyByte = DUMMY_BYTE;

void (*NRF_CE_HIGH)(void);
void (*NRF_CE_LOW)(void);
void (*delaySysMs)(uint32_t);

void NRF_init(NRF24L01_Config_t *config){
	NRF_CE_HIGH = config->NRF_CE_HIGH;
	NRF_CE_LOW = config->NRF_CE_LOW;
	delaySysMs = config->delayMs;

	// Set CE low?
	NRF_CE_LOW();

	// Configure NRF
	NRF_modify_reg(config->spiHandler, CONFIG, CFG_PWR_UP, SET); // Set the PWR_UP bit in NRF_CONFIG
	// Delay needed?
	NRF_modify_reg(config->spiHandler, CONFIG, CFG_CRCO, config->crcEncodingScheme); // 0 = 1 byte, 1 = 2 byte
	NRF_modify_reg(config->spiHandler, CONFIG, CFG_EN_CRC, config->crcEncoding);
	// NRF_cmd_modify_reg(NRF_CONFIG, MASK_TX_DS, !(nrf_type->set_enable_tx_ds_interrupt));    //dsiable TX_DS interrupt on IRQ pin
	NRF_modify_reg(config->spiHandler, CONFIG, CFG_MASK_TX_DS, !(config->enableTXDSIRQ));	// Interrupt is masked with 1
	NRF_modify_reg(config->spiHandler, CONFIG, CFG_MASK_MAX_RT, !(config->enableMaxRtIRQ));
	NRF_modify_reg(config->spiHandler, CONFIG, CFG_MASK_RX_DR, !(config->enableRXDRIRQ));

	NRF_write_reg(config->spiHandler, RF_CH, config->rfChannel);	// RF Channel
	NRF_write_reg(config->spiHandler, EN_AA, 0x00);	// Auto Acknowledgement disabled by default
	NRF_write_reg(config->spiHandler, STATUS, 0x70);	// Write 1s to interrupt flags to clear
	NRF_write_reg(config->spiHandler, SETUP_AW, config->addressWidth);

	if(config->isReceiver){
		NRF_modify_reg(config->spiHandler, EN_RXADDR, config->rxPipe, SET);

		NRF_set_rx_addr(config->spiHandler, config->rxPipe, config->rxAddrHigh, config->rxAddrLow);

		if(config->enableDynamicPlWidth){
			NRF_activate(config->spiHandler);

			NRF_modify_reg(config->spiHandler, FEATURE, FEATURE_EN_DPL, SET);
			NRF_modify_reg(config->spiHandler, DYNPD, config->rxPipe, SET);
			NRF_modify_reg(config->spiHandler, DYNPD, PIPE_1, SET);

			// Auto ack must be enabled
			NRF_modify_reg(config->spiHandler, EN_AA, config->rxPipe, SET);
			NRF_modify_reg(config->spiHandler, EN_AA, PIPE_1, SET);
		}else{
			// NRF_cmd_write_entire_reg((nrf_type->set_rx_pipe + RX_PW_OFFSET), nrf_type->set_payload_width);   //write the static payload width
			NRF_write_reg(config->spiHandler, (config->rxPipe + RX_PW_P0), config->payloadWidth);

			// Auto ack is optional for static payload
			if(config->enableAutoAck){
				NRF_modify_reg(config->spiHandler, EN_AA, config->rxPipe, SET);
				NRF_modify_reg(config->spiHandler, EN_AA, PIPE_1, SET);
			}
		}
	}else{
		NRF_write_reg(config->spiHandler, SETUP_RETR, 0x2F);

		NRF_set_tx_addr(config->spiHandler, config->txAddrHigh, config->txAddrLow, config->enableAutoAck);

		if(config->enableDynamicPlWidth){
			NRF_activate(config->spiHandler);
			NRF_modify_reg(config->spiHandler, FEATURE, FEATURE_EN_DPL, SET);
			NRF_modify_reg(config->spiHandler, DYNPD, PIPE_0, SET);
		}
	}
}

/**
 * Function to write whole 8 bit reg at once
 */
void NRF_write_reg(SPI_Handler_t *spiHandler, uint8_t reg, uint8_t val){
	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	uint8_t commandToSend = W_REGISTER | reg;
	uint8_t dummyData;

	SPI_Send_Data(spiHandler->SPIx, &commandToSend, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummyData, 1);

	SPI_Send_Data(spiHandler->SPIx, &val, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummyData, 1);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
}

/**
 * Function to read 8 Bit Reg On NRF
 */
uint8_t NRF_read_reg(SPI_Handler_t *spiHandler, uint8_t reg){
	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	uint8_t commandToSend = R_REGISTER | reg;
	uint8_t returnVal;

	SPI_Send_Data(spiHandler->SPIx, &commandToSend, 1);
	SPI_Receive_Data(spiHandler->SPIx, &returnVal, 1);

	SPI_Send_Data(spiHandler->SPIx, &dummyByte, 1);
	SPI_Receive_Data(spiHandler->SPIx, &returnVal, 1);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);

	return returnVal;
}

/**
 * Function to modify a bit value in a Reg on NRF
 */
void NRF_modify_reg(SPI_Handler_t *spiHandler, uint8_t reg, uint8_t bit, uint8_t enOrDi){
	// Read current val
	uint8_t reg_val = NRF_read_reg(spiHandler, reg);

	// Update with given bit pos
	if(enOrDi){
		reg_val |= (1 << bit);
	}else{
		reg_val &= ~(1 << bit);
	}

	uint8_t commandToSend = W_REGISTER | reg;
	uint8_t dummy;

	// Write to reg
	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &commandToSend, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	SPI_Send_Data(spiHandler->SPIx, &reg_val, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
}

void NRF_read_multi_byte_reg(SPI_Handler_t *spiHandler, uint8_t reg, uint8_t numBytes, uint8_t *buff){
	uint8_t dummy;

	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &reg, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	// TODO think about this
	for(uint8_t i = 0; i < numBytes; i++){
		SPI_Send_Data(spiHandler->SPIx, &dummyByte, 1);
		SPI_Receive_Data(spiHandler->SPIx, &(buff[i]), 1);
	}

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
}

void NRF_listen(void){
	// Set CE to high
	NRF_CE_HIGH();
}

uint8_t NRF_get_status(SPI_Handler_t *spiHandler){
	return NRF_read_reg(spiHandler, STATUS);
}

void NRF_clear_interrupts(SPI_Handler_t *spiHandler){
	// Write 1 to bits 6, 5, 4 to clear RX_DR, TX_DS and MAX_RT respectively
	NRF_write_reg(spiHandler, STATUS, 0x70);
}

uint8_t NRF_cmd_get_pipe_current_pl(SPI_Handler_t *spiHandler){
	return NRF_read_reg(spiHandler, STATUS_RX_P_NO);
}

void NRF_FLUSH_TX(SPI_Handler_t *spiHandler){
	uint8_t command = FLUSH_TX;
	uint8_t dummy;

	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
}

void NRF_FLUSH_RX(SPI_Handler_t *spiHandler){
	uint8_t command = FLUSH_RX;
	uint8_t dummy;

	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
}

void NRF_set_rx(SPI_Handler_t *spiHandler, uint8_t enOrDi){
	NRF_modify_reg(spiHandler, CONFIG, CFG_PRIM_RX, enOrDi);
}

uint8_t NRF_ready_dynamic_pl_width(SPI_Handler_t *spiHandler){
	return NRF_read_reg(spiHandler, R_RX_PL_WID);
}

void NRF_setup_addr_width(SPI_Handler_t *spiHandler, uint8_t width){
	// Write width to Setup Address width reg (only first 2 bits may be modified
	// so or with 0x3 to make sure they are only bits written
	NRF_write_reg(spiHandler, SETUP_AW, (width | 0x3));
}

void NRF_activate(SPI_Handler_t *spiHandler){
	uint8_t command = ACTIVATE;
	uint8_t dummy;

	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	command = ACTIVATE_BYTE;

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
}

void NRF_set_tx_addr(SPI_Handler_t *spiHandler, uint32_t addrHigh, uint8_t addrLow, uint8_t autoAck){
	uint8_t command = W_REGISTER | TX_ADDR;
	uint8_t dummy;

	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	// TX Addr is a 5 byte address written lsb first
	SPI_Send_Data(spiHandler->SPIx, &addrLow, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	// Set the command to the first byte of addr high
	command = (addrHigh & 0xFF);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	// Set the command to the second byte of addr high
	command = ((addrHigh >> 8) & 0xFF);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	// Set the command to the third byte of addr high
	command = ((addrHigh >> 16) & 0xFF);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	// Set the command to the fourth byte of addr high
	command = ((addrHigh >> 24) & 0xFF);

	SPI_Send_Data(spiHandler->SPIx, &command, 1);
	SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);

	if(autoAck == ENABLE){
		// Enable Auto Acknowledge on pipe 0
		NRF_modify_reg(spiHandler, EN_AA, ENAA_P0, 1);

		SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

		command = W_REGISTER | RX_ADDR_P0;

		// Write Address Of TX into pipe 0
		SPI_Send_Data(spiHandler->SPIx, &command, 1);
		SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

		// TX Addr is a 5 byte address written lsb first
		SPI_Send_Data(spiHandler->SPIx, &addrLow, 1);
		command = (addrHigh & 0xFF);
		SPI_Send_Data(spiHandler->SPIx, &command, 1);
		command = ((addrHigh >> 8) & 0xFF);
		SPI_Send_Data(spiHandler->SPIx, &command, 1);
		command = ((addrHigh >> 16) & 0xFF);
		SPI_Send_Data(spiHandler->SPIx, &command, 1);
		command = ((addrHigh >> 24) & 0xFF);
		SPI_Send_Data(spiHandler->SPIx, &command, 1);

		SPI_Receive_Data(spiHandler->SPIx, &dummy, 1);

		SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
	}
}

void NRF_set_rx_addr(SPI_Handler_t *spiHandler, uint8_t rxPipe, uint32_t addrHigh, uint8_t addrLow){
	// rx pipe should be from PIPE_x definition
	uint8_t temp;

	// First two pipes (0 & 1) have the full 5 byte regs
	if(rxPipe > 1){
		// Write low byte into correct reg
		// Base address for RX_ADDR_OFFSET = 0x0A = RX_ADDR_P0, So adding
		temp = (rxPipe + RX_ADDR_OFFSET);
		NRF_write_reg(spiHandler, temp, addrLow);

		SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

		temp = (W_REGISTER | RX_ADDR_P1);
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		// Write low byte then high from LSB to MSB

		temp = addrHigh & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		temp = (addrHigh >> 8) & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		temp = (addrHigh >> 16) & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		temp = (addrHigh >> 24) & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
	}else{
		// Else is pipe 1 or 0 so write whole addr into reg
		SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

		temp = W_REGISTER | (rxPipe + RX_ADDR_OFFSET);
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		SPI_Send_Data(spiHandler->SPIx, &addrLow, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		temp = addrHigh & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		temp =  (addrHigh >> 8) & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		temp =  (addrHigh >> 16) & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		temp =  (addrHigh >> 24) & 0xFF;
		SPI_Send_Data(spiHandler->SPIx, &temp, 1);
		SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

		SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
	}

}

void NRF_read_rx_payload(SPI_Handler_t *spiHandler, uint8_t *rxBuffer, uint8_t len){
	// Disable CE
	NRF_CE_LOW();

	uint8_t temp = R_RX_PAYLOAD;

	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &temp, 1);
	SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

	temp = DUMMY_BYTE;
	SPI_Send_Data(spiHandler->SPIx, &temp, 1);

	// TODO check if this is ok?
	SPI_Receive_Data(spiHandler->SPIx, rxBuffer, len);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);
}

void NRF_write_tx_payload(SPI_Handler_t *spiHandler, uint8_t *txBuffer, uint8_t len){
	// TODO Disable CE
	NRF_CE_LOW();

	uint8_t temp = W_TX_PAYLOAD;

	SPI_PeripheralControl(spiHandler->SPIx, ENABLE);

	SPI_Send_Data(spiHandler->SPIx, &temp, 1);
	SPI_Receive_Data(spiHandler->SPIx, &temp, 1);

	SPI_Send_Data(spiHandler->SPIx, txBuffer, len);

	SPI_PeripheralControl(spiHandler->SPIx, DISABLE);

	//TODO set CE to high then wait and set to low again
	NRF_CE_HIGH();

	delaySysMs((uint32_t)5);

	NRF_CE_LOW();
}

