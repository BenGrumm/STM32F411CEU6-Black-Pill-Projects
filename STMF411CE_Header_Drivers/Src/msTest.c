/*
 * msTest.c
 *
 *  Created on: 8 Oct 2021
 *      Author: bengr
 */
#include <stdint.h>
#include <string.h>

#include "stm32f411xce.h"

void msTimSetup(void);
void delayMs(uint32_t ms);

int main(void){

	msTimSetup();

	// Init LED GPIO C13
	GPIOC_PCLK_EN();
	GPIOC->MODER |= (1 << 26);
	GPIOC->OTYPER &= ~(1 << 13);
	GPIOC->OSPEEDR |= (1 << 27);

	// Reset
	GPIOC->BSRR |= (1 << 29);

	while(1){
		GPIOC->BSRR |= (1 << 13);

		delayMs(1000);

		GPIOC->BSRR |= (1 << 29);

		delayMs(1000);
	}

}

void msTimSetup(){
	TIM5_PCLK_EN();

	// HSI = 16 MHz For 1000Hz (T = 1ms) 16Mhz / 16000
	TIM5->PSC = 16000;
	// ARR Max
	TIM5->ARR = 0xFFFF;

	// Enable
	TIM5->CR1 |= (1 << 0);
}

void delayMs(uint32_t ms){
	// Reset Counter
	TIM5->CNT = 0;

	while(TIM5->CNT < ms);
}
