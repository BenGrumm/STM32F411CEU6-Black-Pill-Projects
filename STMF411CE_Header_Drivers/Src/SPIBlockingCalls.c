#include <stdint.h>
#include <string.h>

void sendHelloWorld(void);
void sendAndRecieveCommands(void);
uint8_t sendCommandForACK(uint8_t command);

#include "stm32f411xce.h"
#include "stm32f411xce_spi_driver.h"

#define NACK 0xA5
#define ACK 0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_PIN	5

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

/*
 * PB9 SPI2_NSS - AF05
 * PB10 SPI2_SCK - AF05
 * PB14 SPI2_MISO - AF05
 * PB15 SPI2_MOSI - AF05
 */

/*

void initSPIPins(void){
	GPIOB_PCLK_EN();

	// MOSI B15
	GPIOB->MODER |= (1 << 31);
	// MISO B14
	GPIOB->MODER |= (1 << 29);
	// SCK B10
	GPIOB->MODER |= (1 << 21);
	// NSS B9
	GPIOB->MODER |= (1 << 19);

	GPIOB->OSPEEDR |= (0x3 << 30);
	GPIOB->OSPEEDR |= (0x3 << 28);
	GPIOB->OSPEEDR |= (0x3 << 20);
	GPIOB->OSPEEDR |= (0x3 << 18);

	GPIOB->AFRH |= (0x5 << 28);
	GPIOB->AFRH |= (0x5 << 24);
	GPIOB->AFRH |= (0x5 << 8);
	GPIOB->AFRH |= (0x5 << 4);
}

void SPI2_init(void){
	SPI_Handler_t SPI2handler;

	SPI2handler.SPIx = SPI2;
	SPI2handler.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handler.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handler.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handler.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handler.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handler.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handler);
}

uint8_t SPI_Verify_Response(uint8_t ackByte){
	if(ackByte == ((uint8_t)ACK)){
		return 1;
	}

	return 0;
}

void sendAndRecieveCommands(void){
	initSPIPins();
	SPI2_init();

	SPI_SSOEControl(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);

	uint8_t commandCode = COMMAND_LED_CTRL;
	uint8_t args[2];
	uint8_t dummyWrite = 0xFF;

	if(commandCode == COMMAND_LED_CTRL){
		uint8_t ackByte = sendCommandForACK(commandCode);

		if(SPI_Verify_Response(ackByte)){
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_Send_Data(SPI2, args, 2);
		}
	}else if(commandCode == COMMAND_SENSOR_READ){
		uint8_t ackByte = sendCommandForACK(commandCode);

		if(SPI_Verify_Response(ackByte)){
			args[0] = ANALOG_PIN0;

			SPI_Send_Data(SPI2, args, 1);

			for(int i = 0; i < 5000; i++);

			SPI_Send_Data(SPI2, &dummyWrite,1);

			uint8_t analog_read;
			SPI_Receive_Data(SPI2, &analog_read, 1);
		}
	}

	while((SPI2->SR |= (1 << SPI_SR_BSY)) == 0);

	SPI_PeripheralControl(SPI2, DISABLE);

    /* Loop forever
	for(;;);
}

uint8_t sendCommandForACK(uint8_t command){
	uint8_t dummyRead;
	uint8_t dummyWrite = 0xFF;

	uint8_t ackByte;

	SPI_Send_Data(SPI2, &command, 1);

	// Dummy read data to clear RXNE
	SPI_Receive_Data(SPI2, &dummyRead, 1);

	// Send dummy data to then receive data (1 byte)
	SPI_Send_Data(SPI2, &dummyWrite, 1);

	SPI_Receive_Data(SPI2, &ackByte, 1);

	return ackByte;
}

void sendHelloWorld(void){
	char user_data[] = "Hello World";

	initSPIPins();
	SPI2_init();

	// When using software slave management
	// SPI_SSIControl(SPI2, ENABLE);

	SPI_SSOEControl(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);

	// Send length of data
	uint8_t datalength = strlen(user_data);
	SPI_Send_Data(SPI2, &datalength, 1);

	SPI_Send_Data(SPI2, (uint8_t*)user_data, strlen(user_data));

	while((SPI2->SR |= (1 << SPI_SR_BSY)) == 0);

	SPI_PeripheralControl(SPI2, DISABLE);

    /* Loop forever
	for(;;);
}

*/
