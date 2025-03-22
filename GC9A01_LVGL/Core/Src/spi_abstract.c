#include "spi_abstract.h"

void SPI_Write(uint8_t byte)
{
	HAL_SPI_Transmit(&hspi1, &byte, 1, 500);
}

void SPI_WriteArr(uint8_t *bytes, uint32_t len)
{
    HAL_SPI_Transmit(&hspi1, bytes, len, 500);
}

void DISP_SetDataPin(uint8_t val)
{
    HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, val == 0 ? GPIO_PIN_RESET:GPIO_PIN_SET);
}

void DISP_SetResetPin(uint8_t val)
{
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, val == 0 ? GPIO_PIN_RESET:GPIO_PIN_SET);
}

void DISP_SetChipSelectPin(uint8_t val)
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, val == 0 ? GPIO_PIN_RESET:GPIO_PIN_SET);
}

void DISP_SetBacklight(uint8_t val)
{
    TIM1->CCR1 = val;
}