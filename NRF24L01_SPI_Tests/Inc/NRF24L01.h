#ifndef NRF24L01_H
#define NRF24L01_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    SPI_HandleTypeDef* spiHandler;
    uint8_t status;                 // store the last retreived status reg

    bool enableCRC;                 // true / false
    uint8_t crcScheme;              // ENCODING_SCHEME_1_BYTE / ENCODING_SCHEME_2_BYTE
    bool enableAutoAck;             // true / false
    uint8_t rfChannel;              // 0 - 126  (2.400GHz to 2.525GHz)(2400 + RF_CH)
    uint8_t addressWidth;           // THREE_BYTES / FOUR_BYTES / FIVE_BYTES
    bool enableDynamicPlWidth;      // true / false
    uint8_t mode;                   // TX Mode or RX Mode

    uint8_t addrByte1;              // low byte will go in RX_ADDR_P#
    uint32_t addrByte2_5;           // high byte will go in Pipe 1

    //rx settings
    uint8_t rxPipe;                 // PIPE_1 / PIPE_2 / PIPE_3 .. . .. 
    uint8_t payloadWidth;           // 1 - 32 
    bool enableRxDrInterrupt;       // true / false

    //tx settings
    bool enableMaxRtInterrupt;      // true / false
    bool enableTxDsInterrupt;       // true / false
    uint8_t autoRetransmitDelay;    // Auto Retransmit Delay ‘0000’ – Wait 250µS ‘0001’ – Wait 500µS ‘0010’ – Wait 750µS …….. ‘1111’ – Wait 4000µS (Delay defined from end of transmission to start of next transmission)
    uint8_t autoRetransmitCount;    //  Auto Retransmit Count ‘0000’ –Re-Transmit disabled ‘0001’ – Up to 1 Re-Transmit on fail of AA …… ‘1111’ – Up to 15 Re-Transmit on fail of AA

    void (*NRF_setCEPin)(GPIO_PinState);
    void (*NRF_setCSNPin)(GPIO_PinState);
}NRF24L01;

// SPI Commands
#define NRF_COMMAND_R_REGISTER          ((uint8_t)0b00000000) // 000A AAAA Read command and status registers. AAAAA = 5 bit Register Map Address
#define NRF_COMMAND_W_REGISTER          ((uint8_t)0b00100000) // 001A AAAA Write command and status registers. AAAAA = 5 bit Register Map Address Executable in power down or standby modes only.
#define NRF_COMMAND_R_RX_PAYLOAD        ((uint8_t)0b01100001) // Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode.
#define NRF_COMMAND_W_TX_PAYLOAD        ((uint8_t)0b10100000) // Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload.
#define NRF_COMMAND_FLUSH_TX            ((uint8_t)0b11100001) // Flush TX FIFO, used in TX mode
#define NRF_COMMAND_FLUSH_RX            ((uint8_t)0b11100010) // Flush TX FIFO, used in TX mode
#define NRF_COMMAND_REUSE_TX_PL         ((uint8_t)0b11100011) // Used for a PTX device Reuse last transmitted payload. TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deactivated during package transmission.
#define NRF_COMMAND_R_RX_PL_WID         ((uint8_t)0b01100000) // Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define NRF_COMMAND_W_ACK_PAYLOAD       ((uint8_t)0b10101000) // Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101). Maximum three ACK packet payloads can be pending. Payloads with same PPP are handled using first in - first out principle. Write payload: 1– 32 bytes. A write operation always starts at byte 0.
#define NRF_COMMAND_W_TX_PAYLOAD_NOACK  ((uint8_t)0b10110000) // Used in TX mode. Disables AUTOACK on this specific packet.
#define NRF_COMMAND_NOP                 ((uint8_t)0b11111111) // No Operation. Might be used to read the STATUS register

// Registers
#define NRF_REG_CONFIG          ((uint8_t)0x00) // Configuration Register
#define NRF_REG_EN_AA           ((uint8_t)0x01) // Enable ‘Auto Acknowledgment’ Function Disable this functionality to be compatible with nRF2401, see page 72
#define NRF_REG_EN_RXADDR       ((uint8_t)0x02) // Enabled RX Addresses
#define NRF_REG_SETUP_AW        ((uint8_t)0x03) // Setup of Address Widths (common for all data pipes)
#define NRF_REG_SETUP_RETR      ((uint8_t)0x04) // Setup of Automatic Retransmission
#define NRF_REG_RF_CH           ((uint8_t)0x05) // RF Channel
#define NRF_REG_RF_SETUP        ((uint8_t)0x06) // RF Setup Register
#define NRF_REG_STATUS          ((uint8_t)0x07) // Status Register (In parallel to the SPI command word applied on the MOSI pin, the STATUS register is shifted serially out on the MISO pin)
#define NRF_REG_OBSERVE_TX      ((uint8_t)0x08) // Transmit observe register
#define NRF_REG_RPD             ((uint8_t)0x09) // Received Power Detector
#define NRF_REG_RX_ADDR_P0      ((uint8_t)0x0A) // Receive address data pipe 0. 5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW)
#define NRF_REG_RX_ADDR_P1      ((uint8_t)0x0B) // Receive address data pipe 1. 5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW)
#define NRF_REG_RX_ADDR_P2      ((uint8_t)0x0C) // Receive address data pipe 2. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_RX_ADDR_P3      ((uint8_t)0x0D) // Receive address data pipe 3. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_RX_ADDR_P4      ((uint8_t)0x0E) // Receive address data pipe 4. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_RX_ADDR_P5      ((uint8_t)0x0F) // Receive address data pipe 5. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_TX_ADDR         ((uint8_t)0x10) // Transmit address. Used for a PTX device only. (LSByte is written first) Set RX_ADDR_P0 equal to this address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled. See page 72.
#define NRF_REG_RX_PW_P0        ((uint8_t)0x11) // Number of bytes in RX payload in data pipe 0
#define NRF_REG_RX_PW_P1        ((uint8_t)0x12) // Number of bytes in RX payload in data pipe 1
#define NRF_REG_RX_PW_P2        ((uint8_t)0x13) // Number of bytes in RX payload in data pipe 2
#define NRF_REG_RX_PW_P3        ((uint8_t)0x14) // Number of bytes in RX payload in data pipe 3
#define NRF_REG_RX_PW_P4        ((uint8_t)0x15) // Number of bytes in RX payload in data pipe 4
#define NRF_REG_RX_PW_P5        ((uint8_t)0x16) // Number of bytes in RX payload in data pipe 5
#define NRF_REG_FIFO_STATUS     ((uint8_t)0x17) // FIFO Status Register
#define NRF_REG_DYNPD           ((uint8_t)0x1C) // Enable dynamic payload length
#define NRF_REG_FEATURE         ((uint8_t)0x1D) // Feature Register

// Register Bit Pos
#define NRF_CONFIG_MASK_RX_DR   6 // Mask interrupt caused by RX_DR 1: Interrupt not reflected on the IRQ pin 0: Reflect RX_DR as active low interrupt on the IRQ pin
#define NRF_CONFIG_MASK_TX_DS   5 // Mask interrupt caused by TX_DS 1: Interrupt not reflected on the IRQ pin 0: Reflect TX_DS as active low interrupt on the IRQ pin
#define NRF_CONFIG_MASK_MAX_RT  4 // Mask interrupt caused by MAX_RT 1: Interrupt not reflected on the IRQ pin 0: Reflect MAX_RT as active low interrupt on the IRQ pin
#define NRF_CONFIG_EN_CRC       3 // Enable CRC. Forced high if one of the bits in the EN_AA is high
#define NRF_CONFIG_CRCO         2 // CRC encoding scheme '0' - 1 byte '1' – 2 bytes 
#define NRF_CONFIG_PWR_UP       1 // 1: POWER UP, 0:POWER DOWN
#define NRF_CONFIG_PRIM_RX      0 // RX/TX control 1: PRX, 0: PTX 

#define NRF_SETUP_RETR_ARD      4 // Auto Retransmit Delay ‘0000’ – Wait 250µS ‘0001’ – Wait 500µS ‘0010’ – Wait 750µS …….. ‘1111’ – Wait 4000µS (Delay defined from end of transmission to start of next transmission)b
#define NRF_SETUP_RETR_ARC      0 // Auto Retransmit Count ‘0000’ –Re-Transmit disabled ‘0001’ – Up to 1 Re-Transmit on fail of AA …… ‘1111’ – Up to 15 Re-Transmit on fail of AA

#define NRF_STATUS_RX_DR        6 // Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFOc. Write 1 to clear bit.
#define NRF_STATUS_TX_DS        5 // Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. If AUTO_ACK is activated, this bit is set high only when ACK is received. Write 1 to clear bit.
#define NRF_STATUS_MAX_RT       4 // Maximum number of TX retransmits interrupt Write 1 to clear bit. If MAX_RT is asserted it must be cleared to enable further communication. 
#define NRF_STATUS_RX_P_NO      1 // Data pipe number for the payload available for reading from RX_FIFO 000-101: Data Pipe Number 110: Not Used 111: RX FIFO Empty
#define NRF_STATUS_TX_FULL      0 // TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO

// Register Masks
#define NRF_MASK_CONFIG_MASK_RX_DR   (1 << NRF_CONFIG_MASK_RX_DR) // Mask interrupt caused by RX_DR 1: Interrupt not reflected on the IRQ pin 0: Reflect RX_DR as active low interrupt on the IRQ pin
#define NRF_MASK_CONFIG_MASK_TX_DS   (1 << NRF_CONFIG_MASK_TX_DS) // Mask interrupt caused by TX_DS 1: Interrupt not reflected on the IRQ pin 0: Reflect TX_DS as active low interrupt on the IRQ pin
#define NRF_MASK_CONFIG_MASK_MAX_RT  (1 << NRF_CONFIG_MASK_MAX_RT) // Mask interrupt caused by MAX_RT 1: Interrupt not reflected on the IRQ pin 0: Reflect MAX_RT as active low interrupt on the IRQ pin
#define NRF_MASK_CONFIG_EN_CRC       (1 << NRF_CONFIG_EN_CRC) // Enable CRC. Forced high if one of the bits in the EN_AA is high
#define NRF_MASK_CONFIG_CRCO         (1 << NRF_CONFIG_CRCO) // CRC encoding scheme '0' - 1 byte '1' – 2 bytes 
#define NRF_MASK_CONFIG_PWR_UP       (1 << NRF_CONFIG_PWR_UP) // 1: POWER UP, 0:POWER DOWN
#define NRF_MASK_CONFIG_PRIM_RX      (1 << NRF_CONFIG_PRIM_RX) // RX/TX control 1: PRX, 0: PTX 

#define NRF_MASK_STATUS_RX_DR        (1 << NRF_STATUS_RX_DR) // Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFOc. Write 1 to clear bit.
#define NRF_MASK_STATUS_TX_DS        (1 << NRF_STATUS_TX_DS) // Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. If AUTO_ACK is activated, this bit is set high only when ACK is received. Write 1 to clear bit.
#define NRF_MASK_STATUS_MAX_RT       (1 << NRF_STATUS_MAX_RT) // Maximum number of TX retransmits interrupt Write 1 to clear bit. If MAX_RT is asserted it must be cleared to enable further communication. 
#define NRF_MASK_STATUS_RX_P_NO      (1 << NRF_STATUS_RX_P_NO) // Data pipe number for the payload available for reading from RX_FIFO 000-101: Data Pipe Number 110: Not Used 111: RX FIFO Empty
#define NRF_MASK_STATUS_TX_FULL      (1 << NRF_STATUS_TX_FULL) // TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO

// Settings
#define NRF_MODE_TRANSMITTER    0
#define NRF_MODE_RECEIVER       1

#define NRF_CRC_1_BYTE          0
#define NRF_CRC_2_BYTE          1

#define NRF_ADDRES_WIDTH_3_BYTES    0b01
#define NRF_ADDRES_WIDTH_4_BYTES    0b10
#define NRF_ADDRES_WIDTH_5_BYTES    0b11

// Functions
void NRF24L01_setup(NRF24L01* nrf_device);
void NRF24L01_modifyRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t setMask, uint8_t resetMask);
void NRF24L01_writeRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t* pWriteData, uint8_t len);
void NRF24L01_readRegister(NRF24L01* nrf_device, uint8_t regAddr, uint8_t* pReadData, uint8_t len);
void NRF24L01_clearInterrupts(NRF24L01* nrf_device);
void NRF24L01_transmit(NRF24L01* nrf_device, uint8_t* receiverAddress, uint8_t* data, uint8_t dataLen);

#endif