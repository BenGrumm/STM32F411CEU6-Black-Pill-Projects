#include "NRF24L01.h"
#include <stdio.h>

void NRF24L01_writeTransmitAddress(NRF24L01* nrf_device);
void NRF24L01_writeReceiveAddress(NRF24L01* nrf_device);
void NRF24L01_transmitCommand(NRF24L01* nrf_device, uint8_t command);
void NRF24L01_transmitCommandDMA(NRF24L01* nrf_device, uint8_t command);
void NRF24L01_flushTX(NRF24L01* nrf_device);
void NRF24L01_flushTXDMA(NRF24L01* nrf_device);
void NRF24L01_flushRX(NRF24L01* nrf_device);
void NRF24L01_flushRXDMA(NRF24L01* nrf_device);
void NRF24L01_resetAllRegs(NRF24L01* nrf_device);
void NRF24L01_activate(NRF24L01* nrf_device);
void NRF24L01_enableDPL(NRF24L01* nrf_device);

// TODO Remove after debug
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

void NRF24L01_setup(NRF24L01* nrf_device){
    uint8_t tempReg = 0;

    nrf_device->interruptTrigger = false;
    nrf_device->hasTransmitted = false;
    nrf_device->txCpltInterrupt = false;
    nrf_device->isTransmitCEWaitDMA = false;
    nrf_device->rxCpltInterrupt = false;
    nrf_device->txRXCpltInterrupt = false;
    nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_FLUSH;
    nrf_device->dmaLoopTransmitState = NRF_DMA_TRANSMIT_LOOP_STATE_READING;
    sprintf((char*)nrf_device->data, "No Data\n");

    // Ensure chip enable is off and then power up
    nrf_device->NRF_setCEPin(GPIO_PIN_RESET);
    NRF24L01_resetAllRegs(nrf_device);
    NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, NRF_MASK_CONFIG_PWR_UP, 0);

    HAL_Delay(100);

    // Set the encoding scheme for CRC then enable it
    tempReg = nrf_device->crcScheme << NRF_CONFIG_CRCO;
    NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, tempReg, tempReg ^ (1 << NRF_CONFIG_CRCO));
    NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, nrf_device->enableCRC << NRF_CONFIG_EN_CRC, !(nrf_device->enableCRC) << NRF_CONFIG_EN_CRC);

    // Set up the interrupts
    tempReg |= (!nrf_device->enableMaxRtInterrupt) << NRF_CONFIG_MASK_MAX_RT;
    tempReg |= (!nrf_device->enableRxDrInterrupt) << NRF_CONFIG_MASK_RX_DR;
    tempReg |= (!nrf_device->enableTxDsInterrupt) << NRF_CONFIG_MASK_TX_DS;

    // Set the masks by creating which need to be set (0 enables them 1 disables) then xor to ensure the other bits are reset
    NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, tempReg, 
        (NRF_MASK_CONFIG_MASK_MAX_RT | NRF_MASK_CONFIG_MASK_RX_DR | NRF_MASK_CONFIG_MASK_TX_DS) ^ tempReg);

    // Set RF channel (6 bits)
    nrf_device->rfChannel &= 0b111111;
    NRF24L01_writeRegister(nrf_device, NRF_REG_RF_CH, &nrf_device->rfChannel, 1);

    // Disable all auto ack by default
    tempReg = 0;
    NRF24L01_writeRegister(nrf_device, NRF_REG_EN_AA, &tempReg, 1);

    // Set transmit speed (high and low registers and seperated in memory each 1 bit)
    tempReg = (nrf_device->transmitSpeed & 0b10) << (NRF_RF_SETUP_RF_DR_LOW - 1); // - 1 as already bit position 1 not 0
    tempReg |= (nrf_device->transmitSpeed & 0b1) << NRF_RF_SETUP_RF_DR_HIGH;
    NRF24L01_modifyRegister(nrf_device, NRF_REG_RF_SETUP, tempReg, tempReg ^ (1 << NRF_RF_SETUP_RF_DR_LOW | 1 << NRF_RF_SETUP_RF_DR_HIGH));

    NRF24L01_clearInterrupts(nrf_device);

    // Set address width - Illegal to have 00 in address width reg
    if(nrf_device->addressWidth != 0){
        // Ensure the address width is valid then write to the reg
        nrf_device->addressWidth &= 0b11;
        NRF24L01_writeRegister(nrf_device, NRF_REG_SETUP_AW, &nrf_device->addressWidth, 1);
    }

    if(nrf_device->mode == NRF_MODE_RECEIVER){
        // Configure as receiver
        NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, NRF_MASK_CONFIG_PRIM_RX, 0);

        // Enable the pipe being used
        tempReg = 0;
        tempReg |= 1 << nrf_device->rxPipe;
        NRF24L01_writeRegister(nrf_device, NRF_REG_EN_RXADDR, &tempReg, 1);

        NRF24L01_writeReceiveAddress(nrf_device);

        if(nrf_device->enableAutoAck || nrf_device->enableDynamicPlWidth){
            tempReg |= 1 << NRF_PIPE_1;
            // Enable auto ack on chosen pipe and pipe 1
            NRF24L01_modifyRegister(nrf_device, NRF_REG_EN_AA, tempReg, 0);
        }

        if(nrf_device->enableDynamicPlWidth){
            NRF24L01_enableDPL(nrf_device);
        }else{
            // Enable static payload width and write known width
            // First 2 bits in payload width are reserved so ensure 0
            nrf_device->payloadWidth &= 0b00111111;
            // Write the payload width
            NRF24L01_writeRegister(nrf_device, NRF_REG_RX_PW_P0 + nrf_device->rxPipe, &nrf_device->payloadWidth, 1);
        }
    }

    if(nrf_device->mode == NRF_MODE_TRANSMITTER){
        // Configure as transmitter
        NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, 0, NRF_MASK_CONFIG_PRIM_RX);

        // Set up SETUP_RETR for automatic retry and wait time
        tempReg = 0;
        tempReg |= (0b1111 & nrf_device->autoRetransmitDelay) << NRF_SETUP_RETR_ARD;
        tempReg |= (0b1111 & nrf_device->autoRetransmitCount) << NRF_SETUP_RETR_ARC;
        NRF24L01_modifyRegister(nrf_device, NRF_REG_SETUP_RETR, tempReg, ~tempReg);

        // Set up tx address
        NRF24L01_writeTransmitAddress(nrf_device);

        if(nrf_device->enableDynamicPlWidth){
            NRF24L01_enableDPL(nrf_device);
        }
    }
    
    NRF24L01_flushRX(nrf_device);
    NRF24L01_flushTX(nrf_device);

    NRF24L01_clearInterrupts(nrf_device);

    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_CONFIG, &tempReg, 1);
    printf("Config - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(500);

    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_RF_CH, &tempReg, 1);
    printf("RF Chan - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(500);

    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_SETUP_AW, &tempReg, 1);
    printf("AW - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(100);

    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_EN_AA, &tempReg, 1);
    printf("EN AA - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(100);

    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_EN_RXADDR, &tempReg, 1);
    printf("ENRXADDR - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(100);

    // TODO REMOVE
    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_SETUP_AW, &tempReg, 1);
    printf("AW - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(100);

    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_SETUP_RETR, &tempReg, 1);
    printf("RETR - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(100);

    // DEBUG TODO REMOVE
    NRF24L01_readRegister(nrf_device, NRF_REG_RF_SETUP, &tempReg, 1);
    printf("RF Setup - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(tempReg));
    HAL_Delay(100);
}

void NRF24L01_enableDPL(NRF24L01* nrf_device){
    NRF24L01_activate(nrf_device); // May not be need for NRF24L01+
    // Enable dynamic payload 
    NRF24L01_modifyRegister(nrf_device, NRF_REG_FEATURE, NRF_MASK_FEATURE_EN_DPL, 0);
    if(nrf_device->mode == NRF_MODE_TRANSMITTER){
        // A PTX that transmits to a PRX with DPL enabled must have the DPL_P0 bit in DYNPD set.
        NRF24L01_modifyRegister(nrf_device, NRF_REG_DYNPD, 1 << NRF_PIPE_0, 0);
    }else{
        // Set pipe to enable dynamic payload on (0 - 5)
        NRF24L01_modifyRegister(nrf_device, NRF_REG_DYNPD, 1 << nrf_device->rxPipe | 1 << NRF_PIPE_1, 0);
    }
}

void NRF24L01_activate(NRF24L01* nrf_device){
    const uint8_t tempReg = 0x73;
    NRF24L01_writeRegister(nrf_device, 0b01010000, &tempReg, 1);
}

void NRF24L01_resetAllRegs(NRF24L01* nrf_device){
    uint8_t writeBuffer;

    writeBuffer = 0x08;
    NRF24L01_writeRegister(nrf_device, NRF_REG_CONFIG, &writeBuffer, 1);
    writeBuffer = 0x3F;
    NRF24L01_writeRegister(nrf_device, NRF_REG_EN_AA, &writeBuffer, 1);
    writeBuffer = 0x03;
    NRF24L01_writeRegister(nrf_device, NRF_REG_EN_RXADDR, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_SETUP_AW, &writeBuffer, 1);
    writeBuffer = 0x02;
    NRF24L01_writeRegister(nrf_device, NRF_REG_RF_CH, &writeBuffer, 1);
    writeBuffer = 0x02;
    NRF24L01_writeRegister(nrf_device, NRF_REG_RF_CH, &writeBuffer, 1);
    writeBuffer = 0x0E;
    NRF24L01_writeRegister(nrf_device, NRF_REG_RF_SETUP, &writeBuffer, 1);
    writeBuffer = 0x00;
    NRF24L01_writeRegister(nrf_device, NRF_REG_STATUS, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_RX_PW_P0, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_RX_PW_P1, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_RX_PW_P2, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_RX_PW_P3, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_RX_PW_P4, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_RX_PW_P5, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_DYNPD, &writeBuffer, 1);
    NRF24L01_writeRegister(nrf_device, NRF_REG_FEATURE, &writeBuffer, 1);
}

void NRF24L01_writeTransmitAddress(NRF24L01* nrf_device){
    uint8_t lsbToMSBConversion[5] = {0};

    NRF24L01_getLSBToMSBArray(nrf_device->tx_address, lsbToMSBConversion);

    // Write the receivers pipe address in the TX_ADDR register (receivers pipe address and address width must match transmitter settings above)
    // Write address 3-5 bytes depending on setup (write LSByte-MSByte)
    // nrf_device->addressWidth; NRF_ADDRES_WIDTH_3_BYTES; NRF_ADDRES_WIDTH_4_BYTES; NRF_ADDRES_WIDTH_5_BYTES;
    NRF24L01_writeRegister(nrf_device, NRF_REG_TX_ADDR, lsbToMSBConversion, (2 + nrf_device->addressWidth));

    if(nrf_device->enableAutoAck){
        // Enable auto ack on pipe 0
        NRF24L01_modifyRegister(nrf_device, NRF_REG_EN_AA, (1 << NRF_PIPE_0), 0);

        // Copy the same address from TX_ADDR to Pipe 0 on  RX_ADDR_P0 register  because after transmit the NRF momentarily becomes a receiver 
        //      to listen for the auto ACK and it listens on Pipe 0. Remember the address you are transmitting is at least 3 bytes long depending on the width setting. 
        //      You must write that same amount of bytes in the TX_ADDR register and Pipe 0 address register 
        NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P0, lsbToMSBConversion, (2 + nrf_device->addressWidth));
    }
}

void NRF24L01_writeReceiveAddress(NRF24L01* nrf_device){
    uint8_t convertedValues[5] = {0};

    NRF24L01_getLSBToMSBArray(nrf_device->rx_address, convertedValues);

    // Setup expected data width and write the RX address
    switch (nrf_device->rxPipe){
        case NRF_PIPE_0:
            // Pipe 0 write full address to single reg
            NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P0, convertedValues, 5);
            break;
        case NRF_PIPE_1:
            // Pipe 1 write full address to single reg
            NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P1, convertedValues, 5);
            break;
        case NRF_PIPE_2:
            // Pipe 2 write LSB to reg and later rest into pipe 1
            NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P2, convertedValues, 1);
            break;
        case NRF_PIPE_3:
            // Pipe 3 write LSB to reg and later rest into pipe 1
            NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P3, convertedValues, 1);
            break;
        case NRF_PIPE_4:
            // Pipe 4 write LSB to reg and later rest into pipe 1
            NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P4, convertedValues, 1);
            break;
        case NRF_PIPE_5:
            // Pipe 5 write LSB to reg and later rest into pipe 1
            NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P5, convertedValues, 1);
            break;
    }

    // Set up the rest of the address if pipe 2-5
    if(nrf_device->rxPipe != NRF_PIPE_0 && nrf_device->rxPipe != NRF_PIPE_1){
        // Write the 4 MSBytes to pipe 1, first get the existing LSB incase it's being used
        NRF24L01_readRegister(nrf_device, NRF_REG_RX_ADDR_P1, convertedValues, 1);
        // Write the whole address back to the pipe 1 address
        NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P1, convertedValues, 5);
    }
}

void NRF24L01_getLSBToMSBArray(uint64_t valueToConvert, uint8_t* destination){
    // Convert so we transmit the least significant byte to most significant
    destination[0] = valueToConvert & 0xFF;
    destination[1] = (valueToConvert >> 8) & 0xFF;
    destination[2] = (valueToConvert >> 16) & 0xFF;
    destination[3] = (valueToConvert >> 24) & 0xFF;
    destination[4] = (valueToConvert >> 32) & 0xFF;
}

bool NRF24L01_receive(NRF24L01* nrf_device){

    // If using interrupts and interrupt received or not using interrupts
    if((nrf_device->enableRxDrInterrupt && nrf_device->interruptTrigger) || !nrf_device->enableRxDrInterrupt){
        // Check for RX flag
        NRF24L01_readRegister(nrf_device, NRF_REG_STATUS, &nrf_device->status, 1);
        if(!(nrf_device->status & NRF_MASK_STATUS_RX_DR)){
            // RX flag not set so return false no data
            return false;
        }

        // If more than one pipe check which pipe
        // TODO

        // Stop listening
        nrf_device->NRF_setCEPin(GPIO_PIN_RESET);

        uint8_t payloadWidth = 0;

        if(nrf_device->enableDynamicPlWidth){
            NRF24L01_readRegister(nrf_device, NRF_COMMAND_R_RX_PL_WID, &payloadWidth, 1);
        }else{
            payloadWidth = nrf_device->payloadWidth;
        }

        if(payloadWidth <= 32){
            // Retreive data
            NRF24L01_readRegister(nrf_device, NRF_COMMAND_R_RX_PAYLOAD, nrf_device->data, payloadWidth);
        }

        // Clear interrupt
        nrf_device->interruptTrigger = false;
        NRF24L01_clearInterrupts(nrf_device);

        NRF24L01_flushRX(nrf_device);

        // Start listening again
        nrf_device->NRF_setCEPin(GPIO_PIN_SET);

        return true;
    }

    return false;
}

void NRF24L01_transmitLoop(NRF24L01* nrf_device){
    if(!nrf_device->interruptTrigger){
        return;
    }
    
    uint8_t status = 0;
    NRF24L01_readRegister(nrf_device, NRF_REG_STATUS, &status, 1);

    NRF24L01_clearInterrupts(nrf_device);
    nrf_device->interruptTrigger = false;

    if(status & NRF_MASK_STATUS_TX_DS || status & NRF_MASK_STATUS_MAX_RT){
        // Transmit completed succesfully or unsucessfully
        nrf_device->hasTransmitted = false;
    }
}

void NRF24L01_transmitLoopDMA(NRF24L01* nrf_device){
    if(!nrf_device->interruptTrigger && !nrf_device->isTransmitCEWaitDMA){
        return;
    }

    if(HAL_SPI_GetState(nrf_device->spiHandler) == HAL_SPI_STATE_READY && nrf_device->dmaLoopTransmitState == NRF_DMA_TRANSMIT_LOOP_STATE_READING){
        // First read status then store
        nrf_device->transmitLoopTXBuffer[0] = NRF_COMMAND_R_REGISTER | NRF_REG_STATUS;
        nrf_device->transmitLoopTXBuffer[1] = NRF_COMMAND_NOP;
        // Second Clear interrupts
        nrf_device->transmitLoopTXBuffer[2] = NRF_COMMAND_W_REGISTER | NRF_REG_STATUS;
        nrf_device->transmitLoopTXBuffer[3] = NRF_INTERRUPT_MASK;

        nrf_device->dmaLoopTransmitState = NRF_DMA_TRANSMIT_LOOP_STATE_CLEANING;
        nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(nrf_device->spiHandler, nrf_device->transmitLoopTXBuffer, nrf_device->transmitLoopRXBuffer, 4);
    }else if(nrf_device->dmaLoopTransmitState == NRF_DMA_TRANSMIT_LOOP_STATE_CLEANING && nrf_device->txRXCpltInterrupt){
        nrf_device->interruptTrigger = false;
        nrf_device->txRXCpltInterrupt = false;

        if(nrf_device->transmitLoopRXBuffer[1] & NRF_MASK_STATUS_TX_DS || nrf_device->transmitLoopRXBuffer[1] & NRF_MASK_STATUS_MAX_RT){
            // Transmit completed succesfully or unsucessfully
            nrf_device->hasTransmitted = false;
        }

        nrf_device->dmaLoopTransmitState = NRF_DMA_TRANSMIT_LOOP_STATE_READING;
    }
}

/**
 * @brief 
 * 
 * @param nrf_device 
 * @param receiverAddress In order LSByte-MSByte with length of addressWidth set
 * @param data 
 * @param dataLen max 32 bytes
 * @return bool
 */
bool NRF24L01_transmit(NRF24L01* nrf_device, uint8_t* receiverAddress, uint8_t* data, uint8_t dataLen){
    if(nrf_device->mode != NRF_MODE_TRANSMITTER || nrf_device->hasTransmitted){
        return false;
    }

    // Make sure CE is LOW
    nrf_device->NRF_setCEPin(GPIO_PIN_RESET);
    
    // Flush the TX payload
    NRF24L01_flushTX(nrf_device);

    // Send the TX_PAYLOAD command
    NRF24L01_writeRegister(nrf_device, NRF_COMMAND_W_TX_PAYLOAD, data, dataLen);
    
    // Send the payload data.
    // Drive the CE pin HIGH for a minimum of 10us to start the transmission and then bring it back LOW
    nrf_device->NRF_setCEPin(GPIO_PIN_SET);
    HAL_Delay(1);
    nrf_device->NRF_setCEPin(GPIO_PIN_RESET);

    nrf_device->hasTransmitted = true;

    return true;
}

/**
 * @brief 
 * 
 * @param nrf_device 
 * @param receiverAddress In order LSByte-MSByte with length of addressWidth set
 * @param data 
 * @param dataLen max 32 bytes
 * @return bool
 */
bool NRF24L01_transmitDMA(NRF24L01* nrf_device, uint8_t* receiverAddress, uint8_t* data, uint8_t dataLen){

    if(nrf_device->mode != NRF_MODE_TRANSMITTER || (nrf_device->hasTransmitted && !nrf_device->isTransmitCEWaitDMA)){
        return false;
    }
    static uint32_t waitTime = 0;

    if(nrf_device->dmaTransmitState == NRF_DMA_TRANSMIT_STATE_FLUSH){
        // Make sure CE is LOW
        nrf_device->NRF_setCEPin(GPIO_PIN_RESET);

        nrf_device->txCpltInterrupt = false;

        // Flush the TX payload
        NRF24L01_flushTXDMA(nrf_device); // Send DMA then wait for interrupt
        nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_WRITE;
    }else if(nrf_device->txCpltInterrupt && nrf_device->dmaTransmitState == NRF_DMA_TRANSMIT_STATE_WRITE){
        nrf_device->txCpltInterrupt = false;
        // Send the TX_PAYLOAD command
        NRF24L01_writeRegisterDMA(nrf_device, NRF_COMMAND_W_TX_PAYLOAD, data, dataLen);
        nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_SEND;
    }else if(nrf_device->txCpltInterrupt && nrf_device->dmaTransmitState == NRF_DMA_TRANSMIT_STATE_SEND){
        // Send the payload data.
        // Drive the CE pin HIGH for a minimum of 10us to start the transmission and then bring it back LOW
        nrf_device->isTransmitCEWaitDMA = true;
        nrf_device->hasTransmitted = true;
        nrf_device->NRF_setCEPin(GPIO_PIN_SET);
        nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_SENT;
        nrf_device->txCpltInterrupt = false;

        waitTime = HAL_GetTick();
    }else if(nrf_device->dmaTransmitState == NRF_DMA_TRANSMIT_STATE_SENT && (HAL_GetTick() - waitTime) > 0){
        
        nrf_device->isTransmitCEWaitDMA = false;
        nrf_device->NRF_setCEPin(GPIO_PIN_RESET);

        nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_FLUSH;

        return true;
    }

    return false;
}

void NRF24L01_startListening(NRF24L01* nrf_device){
    // Set CE high
    nrf_device->NRF_setCEPin(GPIO_PIN_SET);
    // Wait for interrupt
}

void NRF24L01_flushRX(NRF24L01* nrf_device){
    NRF24L01_transmitCommand(nrf_device, NRF_COMMAND_FLUSH_RX);
}

void NRF24L01_flushTX(NRF24L01* nrf_device){
    NRF24L01_transmitCommand(nrf_device, NRF_COMMAND_FLUSH_TX);
}

void NRF24L01_flushTXDMA(NRF24L01* nrf_device){
    NRF24L01_transmitCommandDMA(nrf_device, NRF_COMMAND_FLUSH_TX);
}

void NRF24L01_transmitCommand(NRF24L01* nrf_device, uint8_t command){
    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
    HAL_SPI_Transmit(nrf_device->spiHandler, &command, 1, 100);
    nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

void NRF24L01_transmitCommandDMA(NRF24L01* nrf_device, uint8_t command){
    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(nrf_device->spiHandler, &command, 1);
}

void NRF24L01_modifyRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t setMask, uint8_t resetMask){
    uint8_t currentRegVal = 0;

    NRF24L01_readRegister(nrf_device, regAddr, &currentRegVal, 1);

    currentRegVal = currentRegVal & (~resetMask);
    currentRegVal = currentRegVal | setMask;

    NRF24L01_writeRegister(nrf_device, regAddr, &currentRegVal, 1);
}

void NRF24L01_writeRegisterDMA(NRF24L01* nrf_device, uint8_t regAddr, uint8_t* pWriteData, uint8_t len){
    uint8_t command[len + 1];
    command[0] = NRF_COMMAND_W_REGISTER | regAddr;
    memcpy(&command[1], pWriteData, len);

    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);

    HAL_SPI_Transmit_DMA(nrf_device->spiHandler, command, (len + 1));

    // Do this in callback?
    // nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

void NRF24L01_writeRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t* pWriteData, uint8_t len){
    uint8_t command = NRF_COMMAND_W_REGISTER | regAddr;

    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);

    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(nrf_device->spiHandler, &command, &nrf_device->status, 1, 100);

    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(nrf_device->spiHandler, pWriteData, len, 100);

    nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

void NRF24L01_readRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t* pReadData, uint8_t len){
    uint8_t command = NRF_COMMAND_R_REGISTER | regAddr;

    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);

    // Transmit the command with reg to read (store the value returned which is the status reg)
    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(nrf_device->spiHandler, &command, &nrf_device->status, 1, 100);

    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_Receive(nrf_device->spiHandler, pReadData, len, 100);

    nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

void NRF24L01_readRegisterDMA(NRF24L01* nrf_device, uint8_t regAddr, uint8_t* pReadData, uint8_t len){
    uint8_t transmitBuffer[len + 1];
    uint8_t receiveBuffer[len + 1]; 

    transmitBuffer[0] = NRF_COMMAND_R_REGISTER | regAddr;

    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);

    // Transmit the command with reg to read (store the value returned which is the status reg)
    HAL_SPI_TransmitReceive_DMA(nrf_device->spiHandler, transmitBuffer, receiveBuffer, len + 1);
}

/**
 * @brief Clear interrupts by writing 1s to corresponding values in the status reg
 * 
 * @param nrf_device device to clear regs on
 */
void NRF24L01_clearInterrupts(NRF24L01* nrf_device){
    uint8_t interruptMask = NRF_INTERRUPT_MASK;
    NRF24L01_writeRegister(nrf_device, NRF_REG_STATUS, &interruptMask, 1);
}
