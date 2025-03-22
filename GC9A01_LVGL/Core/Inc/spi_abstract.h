#ifndef _I_SPI_ABSTRACT
#define _I_SPI_ABSTRACT

#include <stdint.h>
#include "spi.h"
#include "gpio.h"

void SPI_Write(uint8_t byte);
void SPI_WriteArr(uint8_t *bytes, uint32_t len);
void DISP_SetDataPin(uint8_t val);
void DISP_SetResetPin(uint8_t val);
void DISP_SetChipSelectPin(uint8_t val);
void DISP_SetBacklight(uint8_t val);

#endif