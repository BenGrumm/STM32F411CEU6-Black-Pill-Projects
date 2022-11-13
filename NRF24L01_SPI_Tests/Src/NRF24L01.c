#include "NRF24L01.h"

void NRF24L01_setup(NRF24L01* nrf_device){

    // Ensure chip enable is off and then power up
    nrf_device->NRF_setCEPin(GPIO_PIN_RESET);
    NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, NRF_MASK_CONFIG_PWR_UP, 0);

    if(nrf_device->mode == NRF_MODE_TRANSMITTER){
        // Configure as transmitter
        NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, 0, NRF_MASK_CONFIG_PRIM_RX);
    }else if(nrf_device->mode == NRF_MODE_RECEIVER){
        // Configure as receiver
        NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, NRF_MASK_CONFIG_PRIM_RX, 0);
    }

    // Set the encoding scheme for CRC then enable it
    NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, nrf_device->crcScheme << NRF_CONFIG_CRCO, (!nrf_device->crcScheme) << NRF_CONFIG_CRCO);
    // NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, NRF_MASK_CONFIG_EN_CRC, 0); // Not needed as reset 1 and will force enable with auto ack

    // Set up the interrupts
    uint8_t tempReg = 0;
    tempReg |= (!nrf_device->enableMaxRtInterrupt) << NRF_CONFIG_MASK_MAX_RT;
    tempReg |= (!nrf_device->enableRxDrInterrupt) << NRF_CONFIG_MASK_RX_DR;
    tempReg |= (!nrf_device->enableTxDsInterrupt) << NRF_CONFIG_MASK_TX_DS;

    // Set the masks by creating which need to be set (0 enables them 1 disables) then xor to ensure the other bits are reset
    NRF24L01_modifyRegister(nrf_device, NRF_REG_CONFIG, tempReg, 
        (NRF_MASK_CONFIG_MASK_MAX_RT | NRF_MASK_CONFIG_MASK_RX_DR | NRF_MASK_CONFIG_MASK_TX_DS) ^ tempReg);

    // Illegal to have 00 in address width reg
    if(nrf_device->addressWidth != 0){
        // Ensure the address width is valid then write to the reg
        nrf_device->addressWidth &= 0b11;
        NRF24L01_writeRegister(nrf_device, NRF_REG_SETUP_AW, &nrf_device->addressWidth, 1);
    }

    // Set up SETUP_RETR for automatic retry and wait time
    tempReg = 0;
    tempReg |= (0b1111 & nrf_device->autoRetransmitDelay) << NRF_SETUP_RETR_ARD;
    tempReg |= (0b1111 & nrf_device->autoRetransmitCount) << NRF_SETUP_RETR_ARC;
    NRF24L01_writeRegister(nrf_device, NRF_REG_SETUP_RETR, &tempReg, 1);

    NRF24L01_clearInterrupts(nrf_device);
}

/**
 * @brief 
 * 
 * @param nrf_device 
 * @param receiverAddress In order LSByte-MSByte with length of addressWidth set
 * @param data 
 * @param dataLen max 32 bytes
 */
void NRF24L01_transmit(NRF24L01* nrf_device, uint8_t* receiverAddress, uint8_t* data, uint8_t dataLen){
    // Make sure CE is LOW
    nrf_device->NRF_setCEPin(GPIO_PIN_RESET);

    // Write the receivers pipe address in the TX_ADDR register (receivers pipe address and address width must match transmitter settings above)
    // Write address 3-5 bytes depending on setup (write LSByte-MSByte)
    // nrf_device->addressWidth; NRF_ADDRES_WIDTH_3_BYTES; NRF_ADDRES_WIDTH_4_BYTES; NRF_ADDRES_WIDTH_5_BYTES;
    NRF24L01_writeRegister(nrf_device, NRF_REG_TX_ADDR, receiverAddress, (2 + nrf_device->addressWidth));

    // Copy the same address from TX_ADDR to Pipe 0 on  RX_ADDR_P0 register  because after transmit the NRF momentarily becomes a receiver 
    //      to listen for the auto ACK and it listens on Pipe 0. Remember the address you are transmitting is at least 3 bytes long depending on the width setting. 
    //      You must write that same amount of bytes in the TX_ADDR register and Pipe 0 address register 
    NRF24L01_writeRegister(nrf_device, NRF_REG_RX_ADDR_P0, receiverAddress, (2 + nrf_device->addressWidth));
    
    // Send the TX_PAYLOAD command
    NRF24L01_writeRegister(nrf_device, NRF_COMMAND_W_TX_PAYLOAD, data, dataLen);
    
    // Send the payload data.
    // Drive the CE pin HIGH for a minimum of 10us to start the transmission and then bring it back LOW
    nrf_device->NRF_setCEPin(GPIO_PIN_SET);
    HAL_Delay(1);
    nrf_device->NRF_setCEPin(GPIO_PIN_RESET);
}

void NRF24L01_modifyRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t setMask, uint8_t resetMask){
    uint8_t currentRegVal = 0;

    NRF24L01_readRegister(nrf_device, regAddr, &currentRegVal, 1);

    currentRegVal = currentRegVal & (~resetMask);
    currentRegVal = currentRegVal | setMask;

    NRF24L01_writeRegister(nrf_device, regAddr, &currentRegVal, 1);
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

    command = NRF_COMMAND_NOP;
    while(HAL_SPI_GetState(nrf_device->spiHandler) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(nrf_device->spiHandler, &command, pReadData, len, 100);

    nrf_device->NRF_setCSNPin(GPIO_PIN_SET);
}

/**
 * @brief Clear interrupts by writing 1s to corresponding values in the status reg
 * 
 * @param nrf_device device to clear regs on
 */
void NRF24L01_clearInterrupts(NRF24L01* nrf_device){
    uint8_t interruptMask = NRF_MASK_STATUS_MAX_RT | NRF_MASK_STATUS_TX_DS | NRF_MASK_STATUS_RX_DR;
    NRF24L01_writeRegister(nrf_device, NRF_REG_STATUS, &interruptMask, 1);
}
