#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>

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
#define NRF_COMMAND_NOP                 ((uint8_t)0b00000000) // No Operation. Might be used to read the STATUS register

// Registers
#define NRF_REG_CONFIG          ((uint16_t)0x00) // Configuration Register
#define NRF_REG_EN_AA           ((uint16_t)0x01) // Enable ‘Auto Acknowledgment’ Function Disable this functionality to be compatible with nRF2401, see page 72
#define NRF_REG_EN_RXADDR       ((uint16_t)0x02) // Enabled RX Addresses
#define NRF_REG_SETUP_AW        ((uint16_t)0x03) // Setup of Address Widths (common for all data pipes)
#define NRF_REG_SETUP_RETR      ((uint16_t)0x04) // Setup of Automatic Retransmission
#define NRF_REG_RF_CH           ((uint16_t)0x05) // RF Channel
#define NRF_REG_RF_SETUP        ((uint16_t)0x06) // RF Setup Register
#define NRF_REG_STATUS          ((uint16_t)0x07) // Status Register (In parallel to the SPI command word applied on the MOSI pin, the STATUS register is shifted serially out on the MISO pin)
#define NRF_REG_OBSERVE_TX      ((uint16_t)0x08) // Transmit observe register
#define NRF_REG_RPD             ((uint16_t)0x09) // Received Power Detector
#define NRF_REG_RX_ADDR_P0      ((uint16_t)0x0A) // Receive address data pipe 0. 5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW)
#define NRF_REG_RX_ADDR_P1      ((uint16_t)0x0B) // Receive address data pipe 1. 5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW)
#define NRF_REG_RX_ADDR_P2      ((uint16_t)0x0C) // Receive address data pipe 2. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_RX_ADDR_P3      ((uint16_t)0x0D) // Receive address data pipe 3. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_RX_ADDR_P4      ((uint16_t)0x0E) // Receive address data pipe 4. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_RX_ADDR_P5      ((uint16_t)0x0F) // Receive address data pipe 5. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
#define NRF_REG_TX_ADDR         ((uint16_t)0x10) // Transmit address. Used for a PTX device only. (LSByte is written first) Set RX_ADDR_P0 equal to this address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled. See page 72.
#define NRF_REG_RX_PW_P0        ((uint16_t)0x11) // Number of bytes in RX payload in data pipe 0
#define NRF_REG_RX_PW_P1        ((uint16_t)0x12) // Number of bytes in RX payload in data pipe 1
#define NRF_REG_RX_PW_P2        ((uint16_t)0x13) // Number of bytes in RX payload in data pipe 2
#define NRF_REG_RX_PW_P3        ((uint16_t)0x14) // Number of bytes in RX payload in data pipe 3
#define NRF_REG_RX_PW_P4        ((uint16_t)0x15) // Number of bytes in RX payload in data pipe 4
#define NRF_REG_RX_PW_P5        ((uint16_t)0x16) // Number of bytes in RX payload in data pipe 5
#define NRF_REG_FIFO_STATUS     ((uint16_t)0x17) // FIFO Status Register
#define NRF_REG_DYNPD           ((uint16_t)0x1C) // Enable dynamic payload length
#define NRF_REG_FEATURE         ((uint16_t)0x1D) // Feature Register

#endif