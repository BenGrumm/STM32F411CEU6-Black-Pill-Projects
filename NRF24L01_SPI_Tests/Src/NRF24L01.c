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

/**
 * @brief Function to initialise the NRF24L01 with setting passed
 * 
 * @param nrf_device Device with settings set
 */
void NRF24L01_setup(NRF24L01* nrf_device){
    uint8_t tempReg = 0;

    nrf_device->interruptTrigger = 0;
    nrf_device->hasTransmitted = false;
    nrf_device->txCpltInterrupt = false;
    nrf_device->rxCpltInterrupt = false;
    nrf_device->txRXCpltInterrupt = false;
    nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_FLUSH_TX;

    // New stuff 
    nrf_device->isWaitingSend = false;

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

/**
 * @brief Function to enable dynamic payload on nrf_device
 * 
 * @param nrf_device the device on which to enable the dynamic payload
 */
void NRF24L01_enableDPL(NRF24L01* nrf_device){
    NRF24L01_activate(nrf_device); // May not be needed for NRF24L01+
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

/**
 * @brief Function for NRF24L01 that activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features
 * 
 * @param nrf_device device which to activate commands
 */
void NRF24L01_activate(NRF24L01* nrf_device){
    const uint8_t tempReg = 0x73;
    NRF24L01_writeRegister(nrf_device, 0b01010000, &tempReg, 1);
}

/**
 * @brief Function to write reset values to all registers on NRF24L01
 * 
 * @param nrf_device device which to reset
 */
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

/**
 * @brief Write the addres the NRF is sending to to registers
 * 
 * @param nrf_device The device to write to
 */
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

/**
 * @brief Function to write the receivers address to chosen pipe
 * 
 * @param nrf_device The device to write address to
 */
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

/**
 * @brief Function to convert a 40bit address to an array with LSByte to MSByte
 * 
 * @param valueToConvert The address to convert
 * @param destination The destination register of size 5 to write conversion to
 */
void NRF24L01_getLSBToMSBArray(uint64_t valueToConvert, uint8_t* destination){
    // Convert so we transmit the least significant byte to most significant
    destination[0] = valueToConvert & 0xFF;
    destination[1] = (valueToConvert >> 8) & 0xFF;
    destination[2] = (valueToConvert >> 16) & 0xFF;
    destination[3] = (valueToConvert >> 24) & 0xFF;
    destination[4] = (valueToConvert >> 32) & 0xFF;
}

/**
 * @brief Blocking function to first check if there is data to receive and if their is then
 *  read the data and deal with interrupt
 * 
 * @param nrf_device The device to read from
 * @return true If data was succesfully read
 * @return false If there was no data to read
 */
bool NRF24L01_receive(NRF24L01* nrf_device){

    // If using interrupts and interrupt received or not using interrupts
    if((nrf_device->enableRxDrInterrupt && nrf_device->interruptTrigger) || !nrf_device->enableRxDrInterrupt){
        
        nrf_device->NRF_setCEPin(GPIO_PIN_RESET);
        
        // Check for RX flag
        NRF24L01_readStatus(nrf_device);

        if(!(nrf_device->status & NRF_MASK_STATUS_RX_DR)){
            // RX flag not set so return false no data
            nrf_device->NRF_setCEPin(GPIO_PIN_SET);

            return false;
        }

        // If more than one pipe check which pipe
        // TODO

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
        nrf_device->interruptTrigger--;
        NRF24L01_clearInterrupts(nrf_device);

        NRF24L01_flushRX(nrf_device);

        // Start listening again
        nrf_device->NRF_setCEPin(GPIO_PIN_SET);

        return true;
    }

    return false;
}

/**
 * @brief Function to call in main loop that will wait for interrupt from NRF saying it transmitted the data (or reached max retries)
 *  and then clears the interrupt (Blocking SPI calls made)
 * 
 * @param nrf_device The device that is transmitting
 */
void NRF24L01_transmitLoop(NRF24L01* nrf_device){
    if(!nrf_device->interruptTrigger){
        return;
    }
    
    uint8_t status = 0;
    NRF24L01_readRegister(nrf_device, NRF_REG_STATUS, &status, 1);

    NRF24L01_clearInterrupts(nrf_device);
    nrf_device->interruptTrigger--;

    if(status & NRF_MASK_STATUS_TX_DS || status & NRF_MASK_STATUS_MAX_RT){
        // Transmit completed succesfully or unsucessfully
        nrf_device->hasTransmitted = false;
    }
}

/**
 * @brief Function to transmit data to other NRF device (blocking)
 * 
 * @param nrf_device The device that is transmitting the data
 * @param receiverAddress In order LSByte-MSByte with length of addressWidth set
 * @param data The data to send
 * @param dataLen Length of the data max 32 bytes
 * @return bool True if succesfully sent else False
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
 * @brief Function to begin transmission to another NRF using non-blocking DMA spi calls
 * 
 * @param nrf_device The device sending the data
 * @param receiverAddress The receiver address in LSB-MSByte array
 * @param data The data to send
 * @param dataLen The length of the data to send (max 32 bytes)
 * @return true If the NRF is not busy sending
 * @return false If the NRF is unable to send
 */
bool NRF24L01_transmitDMA(NRF24L01* nrf_device, uint8_t* receiverAddress, uint8_t* data, uint8_t dataLen){
    // Check something is not being sent right now
    if(nrf_device->isWaitingSend){
        return false;
    }

    nrf_device->isWaitingSend = true;
    // Copy into buffer
    memmove(&nrf_device->sendBuffer[1], data, dataLen);
    // Set flags, data len and address
    nrf_device->dataLen = dataLen;
    nrf_device->sendAddress = receiverAddress;

    return true;
}

/**
 * @brief Function called in main loop that will call all non-blocking spi calls when
 * the user wants to transmit data
 * 
 * @param nrf_device The device that is transmitting
 */
void NRF24L01_transmitDMALoop(NRF24L01* nrf_device){
    if(!nrf_device->isWaitingSend){
        return;
    }

    switch (nrf_device->dmaTransmitState)
    {
    case NRF_DMA_TRANSMIT_STATE_FLUSH_TX:
        nrf_device->txCpltInterrupt = false;
        NRF24L01_flushTXDMA(nrf_device);
        nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_WRITE_PAYLOAD;
        break;
    case NRF_DMA_TRANSMIT_STATE_WRITE_PAYLOAD:
        if(nrf_device->txCpltInterrupt){
            nrf_device->sendBuffer[0] = NRF_COMMAND_W_TX_PAYLOAD;

            // Make sure CE is LOW
            nrf_device->NRF_setCEPin(GPIO_PIN_RESET);
            nrf_device->txCpltInterrupt = false;

            nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
            HAL_SPI_Transmit_DMA(nrf_device->spiHandler, nrf_device->sendBuffer, nrf_device->dataLen + 1);

            nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_START_SEND;
        }
        break;
    case NRF_DMA_TRANSMIT_STATE_START_SEND:
        if(nrf_device->txCpltInterrupt){
            nrf_device->sendWaitTime = HAL_GetTick();
            nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_FINISH_SEND;

            nrf_device->NRF_setCEPin(GPIO_PIN_SET);
        }
        break;
    case NRF_DMA_TRANSMIT_STATE_FINISH_SEND:
        // TODO currently wait 1ms only need to wait 10uS have user pass function to facilitate
        if((HAL_GetTick() - nrf_device->sendWaitTime) > 0){
            nrf_device->NRF_setCEPin(GPIO_PIN_RESET);
            nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_CHECK_INTERRUPT;
        }
        break;
    case NRF_DMA_TRANSMIT_STATE_CHECK_INTERRUPT:
        if(nrf_device->interruptTrigger){
            // First read status then store
            nrf_device->transmitLoopTXBuffer[0] = NRF_COMMAND_R_REGISTER | NRF_REG_STATUS;
            nrf_device->transmitLoopTXBuffer[1] = NRF_COMMAND_NOP;

            nrf_device->interruptTrigger--;
            nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_CLEAR_INTERRUPT;

            nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive_DMA(nrf_device->spiHandler, nrf_device->transmitLoopTXBuffer, nrf_device->transmitLoopRXBuffer, 2);
        }
        break;
    case NRF_DMA_TRANSMIT_STATE_CLEAR_INTERRUPT:
        if(nrf_device->txRXCpltInterrupt){
            nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
            nrf_device->txRXCpltInterrupt = false;
            nrf_device->txCpltInterrupt = false;
            // Clear interrupts
            nrf_device->transmitLoopTXBuffer[0] = NRF_COMMAND_W_REGISTER | NRF_REG_STATUS;
            nrf_device->transmitLoopTXBuffer[1] = NRF_INTERRUPT_MASK;

            HAL_SPI_Transmit_DMA(nrf_device->spiHandler, nrf_device->transmitLoopTXBuffer, 2);
            nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_CONFIRM_INTERRUPT;
        }
        break;
    case NRF_DMA_TRANSMIT_STATE_CONFIRM_INTERRUPT:
        if(nrf_device->txCpltInterrupt){
            nrf_device->txRXCpltInterrupt = false;

            if(nrf_device->transmitLoopRXBuffer[1] & NRF_MASK_STATUS_TX_DS || nrf_device->transmitLoopRXBuffer[1] & NRF_MASK_STATUS_MAX_RT){
                // Transmit completed succesfully or unsucessfully
                nrf_device->hasTransmitted = false;
            }

            nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_FLUSH_TX;
            nrf_device->isWaitingSend = false;
        }
        break;
    default:
        nrf_device->dmaTransmitState = NRF_DMA_TRANSMIT_STATE_FLUSH_TX;
        break;
    }
}

/**
 * @brief Function to have receiver start listening for transmissions to it
 * 
 * @param nrf_device The device that should start listening
 */
void NRF24L01_startListening(NRF24L01* nrf_device){
    // Set CE high
    nrf_device->NRF_setCEPin(GPIO_PIN_SET);
    // Wait for interrupt
}

/**
 * @brief Blocking function to flush any data that in the receive buffer
 * 
 * @param nrf_device The device to flush
 */
void NRF24L01_flushRX(NRF24L01* nrf_device){
    NRF24L01_transmitCommand(nrf_device, NRF_COMMAND_FLUSH_RX);
}

/**
 * @brief Blocking function to flush any data that is in transmit buffer
 * 
 * @param nrf_device The device to flush
 */
void NRF24L01_flushTX(NRF24L01* nrf_device){
    NRF24L01_transmitCommand(nrf_device, NRF_COMMAND_FLUSH_TX);
}

/**
 * @brief Non-blocking DMA function to flush the transmit buffer
 * 
 * @param nrf_device The device to flush
 */
void NRF24L01_flushTXDMA(NRF24L01* nrf_device){
    NRF24L01_transmitCommandDMA(nrf_device, NRF_COMMAND_FLUSH_TX);
}

/**
 * @brief Function to transmit a command to the NRF
 * 
 * @param nrf_device Device to send command to
 * @param command Command being sent
 */
void NRF24L01_transmitCommand(NRF24L01* nrf_device, uint8_t command){
    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
    HAL_SPI_Transmit(nrf_device->spiHandler, &command, 1, 100);
    nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

/**
 * @brief Non-blocking function to transmit a command to the NRF
 * 
 * @param nrf_device Device to send command to
 * @param command Command being sent
 */
void NRF24L01_transmitCommandDMA(NRF24L01* nrf_device, uint8_t command){
    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(nrf_device->spiHandler, &command, 1);
}

/**
 * @brief Blocking function to modify a register on the NRF
 * 
 * @param nrf_device Device to modify
 * @param regAddr Address of reg to modify
 * @param setMask The mask of bits that should be set
 * @param resetMask The mask of bits that should be reset
 */
void NRF24L01_modifyRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t setMask, uint8_t resetMask){
    uint8_t currentRegVal = 0;

    NRF24L01_readRegister(nrf_device, regAddr, &currentRegVal, 1);

    currentRegVal = currentRegVal & (~resetMask);
    currentRegVal = currentRegVal | setMask;

    NRF24L01_writeRegister(nrf_device, regAddr, &currentRegVal, 1);
}

/**
 * @brief Non-blocking function to write a register on the NRF (DO NOT USE)
 * 
 * @param nrf_device Device to write to
 * @param regAddr The address of the register being written to
 * @param pWriteData The data being written
 * @param len Length of the data being written
 */
void NRF24L01_writeRegisterDMA(NRF24L01* nrf_device, uint8_t regAddr, const uint8_t* pWriteData, uint8_t len){
    // DO NOT USE TODO MOVE ARRAY TO NRF
    uint8_t command[len + 1];
    command[0] = NRF_COMMAND_W_REGISTER | regAddr;
    memcpy(&command[1], pWriteData, len);

    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);

    HAL_SPI_Transmit_DMA(nrf_device->spiHandler, command, (len + 1));

    // Do this in callback?
    // nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

/**
 * @brief Blocking function to write whole register(s)
 * 
 * @param nrf_device Device to write to
 * @param regAddr The address of the register being written
 * @param pWriteData The data being written
 * @param len Then amount of data being written (bytes)
 */
void NRF24L01_writeRegister(NRF24L01* nrf_device, uint8_t regAddr, const uint8_t* pWriteData, uint8_t len){
    uint8_t command = NRF_COMMAND_W_REGISTER | regAddr;

    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);

    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(nrf_device->spiHandler, &command, &nrf_device->status, 1, 100);

    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(nrf_device->spiHandler, pWriteData, len, 100);

    nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

/**
 * @brief Function to read register(s) from the NRF
 * 
 * @param nrf_device The device to read from
 * @param regAddr Address of the register to read from
 * @param pReadData Location to store data
 * @param len Amount of data to read
 */
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

void NRF24L01_readStatus(NRF24L01* nrf_device){
    uint8_t command = NRF_COMMAND_NOP;

    nrf_device->NRF_setCSNPin(GPIO_PIN_RESET);

    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(nrf_device->spiHandler, &command, &nrf_device->status, 1, 100);

    nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

/**
 * @brief Non-blocking function to read a register from NRF device
 * 
 * @param nrf_device The device to read from
 * @param regAddr Address of the register to read from
 * @param pReadData Location to store data
 * @param len Amount of data to read
 */
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
