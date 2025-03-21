/**
 ******************************************************************************
 * @file           : stm32f411xce.h
 * @author         : Ben Grummitt
 * @brief          : Header file for the STM32F411xCE MCU.
 ******************************************************************************
 */
#ifndef __INC_STM32F411XCE_HEADER
#define __INC_STM32F411XCE_HEADER

#include <stdint.h>
#include <stddef.h>

/*
 * Volatile definition
 */
#define __vo volatile
#define __weak __attribute__((weak))

/*
 * M4 Specific
 */

/*
 * Interrupt
 */

#define NO_PR_BITS_IMPLEMENTED 		8 // Number of bits used for the priority registers

#define NVIC_CONTROLLER_BASEADDR 	(0xE000E000UL)
#define NVIC_PR_BASE_ADDR 			(NVIC_CONTROLLER_BASEADDR + 0x280UL)

#define NVIC_ISER0					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x100UL + (0x04UL * 0)))
#define NVIC_ISER1					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x100UL + (0x04UL * 1)))
#define NVIC_ISER2					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x100UL + (0x04UL * 2)))

#define NVIC_ICER0					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x180UL + (0x04UL * 0)))
#define NVIC_ICER1					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x180UL + (0x04UL * 1)))
#define NVIC_ICER2					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x180UL + (0x04UL * 2)))

#define NVIC_ISPR0					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x200UL + (0x04UL * 0)))
#define NVIC_ISPR1					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x200UL + (0x04UL * 1)))
#define NVIC_ISPR2					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x200UL + (0x04UL * 2)))

#define NVIC_ICPR0					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x280UL + (0x04 * 0)))
#define NVIC_ICPR1					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x280UL + (0x04 * 1)))
#define NVIC_ICPR2					((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x280UL + (0x04 * 2)))

#define NVIC_IPRx(x)				((__vo uint32_t*)(NVIC_CONTROLLER_BASEADDR + 0x400UL + 0x04UL * (x)))

/*
 * Base Addresses For Flash & SRAM
 */
#define FLASH_BASEADDR											(0x08000000UL)
#define SRAM1_BASEADDR											(0x20000000UL)

/*
 * Base Addresses For AHBx & APBx Busses
 */
#define AHB2_BASEADDR											(0x50000000UL)
#define AHB1_BASEADDR											(0x40020000UL)
#define APB2_BASEADDR											(0x40010000UL)
#define APB1_BASEADDR											(0x40000000UL)

/*
 * Addresses For Peripherals On The AHB2 Bus
 */
#define USB_OTG_BASEADDR										(AHB2_BASEADDR + 0x6400UL)

/*
 * Addresses For Peripherals On The AHB1 Bus
 */
#define DMA2_BASEADDR											(AHB1_BASEADDR + 0x6400UL)
#define DMA1_BASEADDR											(AHB1_BASEADDR + 0x6000UL)
#define FLASH_INTERFACE_BASEADDR								(AHB1_BASEADDR + 0x3C00UL)
#define RCC_BASEADDR											(AHB1_BASEADDR + 0x3800UL)
#define CRC_BASEADDR											(AHB1_BASEADDR + 0x3000UL)
#define GPIOH_BASEADDR											(AHB1_BASEADDR + 0x1C00UL)
#define GPIOE_BASEADDR											(AHB1_BASEADDR + 0x1000UL)
#define GPIOD_BASEADDR											(AHB1_BASEADDR + 0x0C00UL)
#define GPIOC_BASEADDR											(AHB1_BASEADDR + 0x0800UL)
#define GPIOB_BASEADDR											(AHB1_BASEADDR + 0x0400UL)
#define GPIOA_BASEADDR											(AHB1_BASEADDR)

/*
 * Addresses For Peripherals On The APB2 Bus
 */
#define SPI5_BASEADDR											(APB2_BASEADDR + 0x5000UL)
#define TIM11_BASEADDR											(APB2_BASEADDR + 0x4800UL)
#define TIM10_BASEADDR											(APB2_BASEADDR + 0x4400UL)
#define TIM9_BASEADDR											(APB2_BASEADDR + 0x4000UL)
#define EXTI_BASEADDR											(APB2_BASEADDR + 0x3C00UL)
#define SYSCFG_BASEADDR											(APB2_BASEADDR + 0x3800UL)
#define SPI4_BASEADDR											(APB2_BASEADDR + 0x3400UL)
#define SPI1_BASEADDR											(APB2_BASEADDR + 0x3000UL)
#define SDIO_BASEADDR											(APB2_BASEADDR + 0x2C00UL)
#define ADC1_BASEADDR											(APB2_BASEADDR + 0x2000UL)
#define USART6_BASEADDR											(APB2_BASEADDR + 0x1400UL)
#define USART1_BASEADDR											(APB2_BASEADDR + 0x1000UL)
#define TIM1_BASEADDR											(APB2_BASEADDR)

/*
 * Addresses For Peripherals On The APB1 Bus
 */
#define PWR_BASEADDR											(APB1_BASEADDR + 0x7000UL)
#define I2C3_BASEADDR											(APB1_BASEADDR + 0x5C00UL)
#define I2C2_BASEADDR											(APB1_BASEADDR + 0x5800UL)
#define I2C1_BASEADDR											(APB1_BASEADDR + 0x5400UL)
#define USART2_BASEADDR											(APB1_BASEADDR + 0x4400UL)
#define I2S3ext_BASEADDR										(APB1_BASEADDR + 0x4000UL)
#define SPI3_BASEADDR											(APB1_BASEADDR + 0x3C00UL)
#define SPI2_BASEADDR											(APB1_BASEADDR + 0x3800UL)
#define I2S2ext_BASEADDR										(APB1_BASEADDR + 0x3400UL)
#define IWDG_BASEADDR											(APB1_BASEADDR + 0x3000UL)
#define WWDG_BASEADDR											(APB1_BASEADDR + 0x2C00UL)
#define RTC_BKP_BASEADDR										(APB1_BASEADDR + 0x2800UL)
#define TIM5_BASEADDR											(APB1_BASEADDR + 0x0C00UL)
#define TIM4_BASEADDR											(APB1_BASEADDR + 0x0800UL)
#define TIM3_BASEADDR											(APB1_BASEADDR + 0x0400UL)
#define TIM2_BASEADDR											(APB1_BASEADDR)

/*
 * Register Definitions
 */
typedef struct {
	__vo uint32_t MODER;											/* GPIO port mode register										*/
	__vo uint32_t OTYPER;											/* GPIO port output type register								*/
	__vo uint32_t OSPEEDR;											/* GPIO port output speed register								*/
	__vo uint32_t PUPDR;											/* GPIO port pull-up/pull-down register							*/
	__vo uint32_t IDR;												/* GPIO port input data register. Bits 16-31 Reserved 			*/
	__vo uint32_t ODR;												/* GPIO port output data register. Bits 16-31 Reserved			*/
	__vo uint32_t BSRR;												/* GPIO port bit set/reset register								*/
	__vo uint32_t LCKR;												/* GPIO port configuration lock register 						*/
	__vo uint32_t AFRL;												/* GPIO alternate function low register 						*/
	__vo uint32_t AFRH;												/* GPIO alternate function high register 						*/
} GPIO_Reg_Def_t;

typedef struct {
	__vo uint32_t CR;												/* RCC clock control register									*/
	__vo uint32_t PLLCFGR;											/* RCC PLL configuration register								*/
	__vo uint32_t CFGR;												/* RCC clock configuration register								*/
	__vo uint32_t CIR;												/* RCC clock interrupt register									*/
	__vo uint32_t AHB1RSTR;											/* RCC AHB1 peripheral reset register							*/
	__vo uint32_t AHB2RSTR;											/* RCC AHB2 peripheral reset register							*/
	uint32_t RESERVED1[2];
	__vo uint32_t APB1RSTR;											/* RCC APB1 peripheral reset register							*/
	__vo uint32_t APB2RSTR;											/* RCC APB2 peripheral reset register							*/
	uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;											/* RCC AHB1 peripheral clock enable register					*/
	__vo uint32_t AHB2ENR;											/* RCC AHB2 peripheral clock enable register					*/
	uint32_t RESERVED3[2];
	__vo uint32_t APB1ENR;											/* RCC APB1 peripheral clock enable register					*/
	__vo uint32_t APB2ENR;											/* RCC APB2 peripheral clock enable register					*/
	uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;										/* RCC AHB1 peripheral clock enable in low power mode register	*/
	__vo uint32_t AHB2LPENR;										/* RCC AHB2 peripheral clock enable in low power mode register	*/
	uint32_t RESERVED5[2];
	__vo uint32_t APB1LPENR;										/* RCC APB1 peripheral clock enable in low power mode register	*/
	__vo uint32_t APB2LPENR;										/* RCC APB2 peripheral clock enabled in low power mode register	*/
	uint32_t RESERVED6[2];
	__vo uint32_t BDCR;												/* RCC Backup domain control register							*/
	__vo uint32_t CSR;												/* RCC clock control & status register							*/
	uint32_t RESERVED7[2];
	__vo uint32_t SSCGR;											/* RCC spread spectrum clock generation register				*/
	__vo uint32_t PLLI2SCFGR;										/* RCC PLLI2S configuration register							*/
	__vo uint32_t DCKCFGR;											/* RCC Dedicated Clocks Configuration Register					*/
} RCC_Reg_Def_t;

typedef struct {
	__vo uint32_t IMR;												/* Interrupt mask register										*/
	__vo uint32_t EMR;												/* Event mask register											*/
	__vo uint32_t RTSR;												/* Rising trigger selection register							*/
	__vo uint32_t FTSR;												/* Falling trigger selection register							*/
	__vo uint32_t SWIER;											/* Software interrupt event register							*/
	__vo uint32_t PR;												/* Pending register												*/
}EXTI_Reg_Def_t;

typedef struct {
	__vo uint32_t MEMRMP;											/* SYSCFG memory remap register									*/
	__vo uint32_t PMC;												/* SYSCFG peripheral mode configuration register				*/
	__vo uint32_t EXTICR1;											/* SYSCFG external interrupt configuration register 1			*/
	__vo uint32_t EXTICR2;											/* SYSCFG external interrupt configuration register 2			*/
	__vo uint32_t EXTICR3;											/* SYSCFG external interrupt configuration register 3			*/
	__vo uint32_t EXTICR4;											/* SYSCFG external interrupt configuration register 4			*/
	uint32_t RESERVED[2];
	__vo uint32_t CMPCR;											/* Compensation cell control register							*/
} SYSCFG_Reg_Def_t;

typedef struct {
	__vo uint32_t CR1;												/* TIMx control register 1										*/
	__vo uint32_t CR2;												/* TIMx control register 2										*/
	__vo uint32_t SMCR;												/* TIMx slave mode control register								*/
	__vo uint32_t DIER;												/* TIMx DMA/Interrupt enable register							*/
	__vo uint32_t SR;												/* TIMx status register											*/
	__vo uint32_t EGR;												/* TIMx event generation register								*/
	__vo uint32_t CCMR1;											/* TIMx capture/compare mode register 1							*/
	__vo uint32_t CCMR2;											/* TIMx capture/compare mode register 2							*/
	__vo uint32_t CCER;												/* TIMx capture/compare enable register							*/
	__vo uint32_t CNT;												/* TIMx counter													*/
	__vo uint32_t PSC;												/* TIMx prescaler												*/
	__vo uint32_t ARR;												/* TIMx auto-reload register									*/
	uint32_t RESERVED;
	__vo uint32_t CCR1;												/* TIMx capture/compare register 1								*/
	__vo uint32_t CCR2;												/* TIMx capture/compare register 2								*/
	__vo uint32_t CCR3;												/* TIMx capture/compare register 3								*/
	__vo uint32_t CCR4;												/* TIMx capture/compare register 4								*/
	uint32_t RESERVED2;
	__vo uint32_t DCR;												/* TIMx DMA control register									*/
	__vo uint32_t DMAR;												/* TIMx DMA address for full transfer							*/
	__vo uint32_t OR;												/* TIM2 / TIM5 option register									*/
} TIM2_5_Reg_Def_t;

typedef struct {
	__vo uint32_t CR1;												/* SPI control register 1 										*/
	__vo uint32_t CR2;												/* SPI control register 2 										*/
	__vo uint32_t SR;												/* SPI status register 											*/
	__vo uint32_t DR;												/* SPI data register 											*/
	__vo uint32_t CRCPR;											/* SPI CRC polynomial register 									*/
	__vo uint32_t RXCRCR;											/* SPI RX CRC register 											*/
	__vo uint32_t TXCRCR;											/* SPI TX CRC register 											*/
	__vo uint32_t I2SCFGR;											/* SPI_I2S configuration register 								*/
	__vo uint32_t I2SPR;											/* SPI_I2S prescaler register 									*/
} SPI_Reg_Def_t;

typedef struct {
	__vo uint32_t CR1;												/* I2C Control register 1										*/
	__vo uint32_t CR2;												/* I2C Control register 2										*/
	__vo uint32_t OAR1;												/* I2C Own address register 1									*/
	__vo uint32_t OAR2;												/* I2C Own address register 2									*/
	__vo uint32_t DR;												/* I2C Data register											*/
	__vo uint32_t SR1;												/* I2C Status register 1										*/
	__vo uint32_t SR2;												/* I2C Status register 2										*/
	__vo uint32_t CCR;												/* I2C Clock control register									*/
	__vo uint32_t TRISE;											/* I2C TRISE register											*/
	__vo uint32_t FLTR;												/* I2C FLTR register											*/
} I2C_Reg_Def_t;


/*
 * Peripheral Definitions Type Casted As Reg From Reg Definitions
 */

#define GPIOA													((GPIO_Reg_Def_t*) GPIOA_BASEADDR)
#define GPIOB													((GPIO_Reg_Def_t*) GPIOB_BASEADDR)
#define GPIOC													((GPIO_Reg_Def_t*) GPIOC_BASEADDR)
#define GPIOD													((GPIO_Reg_Def_t*) GPIOD_BASEADDR)
#define GPIOE													((GPIO_Reg_Def_t*) GPIOE_BASEADDR)
#define GPIOH													((GPIO_Reg_Def_t*) GPIOH_BASEADDR)

#define RCC														((RCC_Reg_Def_t*) RCC_BASEADDR)

#define EXTI													((EXTI_Reg_Def_t*) EXTI_BASEADDR)

#define SYSCFG													((SYSCFG_Reg_Def_t*) SYSCFG_BASEADDR)

#define TIM2													((TIM2_5_Reg_Def_t*) TIM2_BASEADDR)
#define TIM3													((TIM2_5_Reg_Def_t*) TIM3_BASEADDR)
#define TIM4													((TIM2_5_Reg_Def_t*) TIM4_BASEADDR)
#define TIM5													((TIM2_5_Reg_Def_t*) TIM5_BASEADDR)

#define SPI1													((SPI_Reg_Def_t*) SPI1_BASEADDR)
#define SPI2													((SPI_Reg_Def_t*) SPI2_BASEADDR)
#define SPI3													((SPI_Reg_Def_t*) SPI3_BASEADDR)
#define SPI4													((SPI_Reg_Def_t*) SPI4_BASEADDR)
#define SPI5													((SPI_Reg_Def_t*) SPI5_BASEADDR)

#define I2C1													((I2C_Reg_Def_t*) I2C1_BASEADDR)
#define I2C2													((I2C_Reg_Def_t*) I2C2_BASEADDR)
#define I2C3													((I2C_Reg_Def_t*) I2C3_BASEADDR)

/*
 * Macros For Enabling And Disabling Clocks
 */

/*
 * AHB1 Bus Clocks
 */
#define DMA2_PCLK_EN()											(RCC->AHB1ENR |= (1 << 22))
#define DMA1_PCLK_EN()											(RCC->AHB1ENR |= (1 << 21))
#define CRC_PCLK_EN()											(RCC->AHB1ENR |= (1 << 12))
#define GPIOH_PCLK_EN()											(RCC->AHB1ENR |= (1 << 7))
#define GPIOE_PCLK_EN()											(RCC->AHB1ENR |= (1 << 4))
#define GPIOD_PCLK_EN()											(RCC->AHB1ENR |= (1 << 3))
#define GPIOC_PCLK_EN()											(RCC->AHB1ENR |= (1 << 2))
#define GPIOB_PCLK_EN()											(RCC->AHB1ENR |= (1 << 1))
#define GPIOA_PCLK_EN()											(RCC->AHB1ENR |= (1 << 0))

#define DMA2_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 22))
#define DMA1_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 21))
#define CRC_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 12))
#define GPIOH_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOE_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOA_PCLK_DI()											(RCC->AHB1ENR &= ~(1 << 0))

/*
 * AHB2 Bus Clocks
 */
#define OTGFSEN_PCLK_EN()										(RCC->AHB2ENR  |= (1 << 7))

#define OTGFSEN_PCLK_DI()										(RCC->AHB2ENR  &= ~(1 << 7))

/*
 * APB1 Bus Clocks
 */
#define PWR_PCLK_EN()											(RCC->APB1ENR  |= (1 << 28))
#define I2C3_PCLK_EN()											(RCC->APB1ENR  |= (1 << 23))
#define I2C2_PCLK_EN()											(RCC->APB1ENR  |= (1 << 22))
#define I2C1_PCLK_EN()											(RCC->APB1ENR  |= (1 << 21))
#define USART2_PCLK_EN()										(RCC->APB1ENR  |= (1 << 17))
#define SPI3_PCLK_EN()											(RCC->APB1ENR  |= (1 << 15))
#define SPI2_PCLK_EN()											(RCC->APB1ENR  |= (1 << 14))
#define WWDG_PCLK_EN()											(RCC->APB1ENR  |= (1 << 11))
#define TIM5_PCLK_EN()											(RCC->APB1ENR  |= (1 << 3))
#define TIM4_PCLK_EN()											(RCC->APB1ENR  |= (1 << 2))
#define TIM3_PCLK_EN()											(RCC->APB1ENR  |= (1 << 1))
#define TIM2_PCLK_EN()											(RCC->APB1ENR  |= (1 << 0))

#define PWR_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 28))
#define I2C3_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 23))
#define I2C2_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 22))
#define I2C1_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 21))
#define USART2_PCLK_DI()										(RCC->APB1ENR  &= ~(1 << 17))
#define SPI3_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 15))
#define SPI2_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 14))
#define WWDG_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 11))
#define TIM5_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 3))
#define TIM4_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 2))
#define TIM3_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 1))
#define TIM2_PCLK_DI()											(RCC->APB1ENR  &= ~(1 << 0))

/*
 * APB2 Bus Clocks
 */
#define SPI5_PCLK_EN()											(RCC->APB2ENR |= (1 << 20))
#define TIM11_PCLK_EN()											(RCC->APB2ENR |= (1 << 18))
#define TIM10_PCLK_EN()											(RCC->APB2ENR |= (1 << 17))
#define TIM9_PCLK_EN()											(RCC->APB2ENR |= (1 << 16))
#define SYSCFG_PCLK_EN()										(RCC->APB2ENR |= (1 << 14))
#define SPI4_PCLK_EN()											(RCC->APB2ENR |= (1 << 13))
#define SPI1_PCLK_EN()											(RCC->APB2ENR |= (1 << 12))
#define SDIO_PCLK_EN()											(RCC->APB2ENR |= (1 << 11))
#define ADC1_PCLK_EN()											(RCC->APB2ENR |= (1 << 8))
#define USART6_PCLK_EN()										(RCC->APB2ENR |= (1 << 5))
#define USART1_PCLK_EN()										(RCC->APB2ENR |= (1 << 4))
#define TIM1_PCLK_EN()											(RCC->APB2ENR |= (1 << 0))

#define SPI5_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 20))
#define TIM11_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 18))
#define TIM10_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 17))
#define TIM9_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 16))
#define SYSCFG_PCLK_DI()										(RCC->APB2ENR &= ~(1 << 14))
#define SPI4_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 13))
#define SPI1_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 12))
#define SDIO_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 11))
#define ADC1_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 8))
#define USART6_PCLK_DI()										(RCC->APB2ENR &= ~(1 << 5))
#define USART1_PCLK_DI()										(RCC->APB2ENR &= ~(1 << 4))
#define TIM1_PCLK_DI()											(RCC->APB2ENR &= ~(1 << 0))

// IRQ Numbers
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85

// Generic Macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define FLAG_SET			SET
#define FLAG_RESET			RESET
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

/**
 * Bit Position Definitions For RCC
 */

#define RCC_PLLCFGR_PLLQ	24
#define RCC_PLLCFGR_PLLSRC	22
#define RCC_PLLCFGR_PLLP	16
#define RCC_PLLCFGR_PLLN	6
#define RCC_PLLCFGR_PLLM	0

#define RCC_CFGR_MCO2		30
#define RCC_CFGR_MCO2PRE	27
#define RCC_CFGR_MCO1PRE	24
#define RCC_CFGR_I2SSRC		23
#define RCC_CFGR_MCO1		21
#define RCC_CFGR_RTCPRE		16
#define RCC_CFGR_PPRE2		13
#define RCC_CFGR_PPRE1		10
#define RCC_CFGR_HPRE		4
#define RCC_CFGR_SWS		2
#define RCC_CFGR_SW			0

/**
 * Bit Position Definitions For SPI
 */
#define SPI_CR1_BIDIMODE	15
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_CRCEN		13
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_DFF			11
#define SPI_CR1_RXONLY		10
#define SPI_CR1_SSM			9
#define SPI_CR1_SSI			8
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SPE			6
#define SPI_CR1_BR			3
#define SPI_CR1_MSTR		2
#define SPI_CR1_CPOL		1
#define SPI_CR1_CPHA		0

#define SPI_CR2_TXEIE		7
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_ERRIE		5
#define SPI_CR2_FRF			4
#define SPI_CR2_SSOE		2
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_RXDMAEN		0

#define SPI_SR_FRE			8
#define SPI_SR_BSY			7
#define SPI_SR_OVR			6
#define SPI_SR_MODF			5
#define SPI_SR_CRCERR		4
#define SPI_SR_UDR			3
#define SPI_SR_CHSIDE		2
#define SPI_SR_TXE			1
#define SPI_SR_RXNE			0

/**
 * Bit Positions For I2C Registers
 */
#define I2C_CR1_SWRST		15
#define I2C_CR1_ALERT		13
#define I2C_CR1_PEC			12
#define I2C_CR1_POS			11
#define I2C_CR1_ACK			10
#define I2C_CR1_STOP		9
#define I2C_CR1_START		8
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_ENGC		6
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENARP		4
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_SMBUS		1
#define I2C_CR1_PE			0

#define I2C_CR2_LAST		12
#define I2C_CR2_DMAEN		11
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITERREN		8
#define I2C_CR2_FREQ		0

#define I2C_SR1_SMBALERT	15
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_PECERR		12
#define I2C_SR1_OVR			11
#define I2C_SR1_AF			10
#define I2C_SR1_ARLO		9
#define I2C_SR1_BERR		8
#define I2C_SR1_TxE			7
#define I2C_SR1_RxNE		6
#define I2C_SR1_STOPF		4
#define I2C_SR1_ADD10		3
#define I2C_SR1_BTF			2
#define I2C_SR1_ADDR		1
#define I2C_SR1_SB			0

#define I2C_SR2_PEC			8
#define I2C_SR2_DUALF		7
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_GENCALL		4
#define I2C_SR2_TRA			2
#define I2C_SR2_BUSY		1
#define I2C_SR2_MSL			0

#define I2C_CCR_FS			15
#define I2C_CCR_DUTY		14
#define I2C_CCR_CCR			0

#include "stm32f411xce_spi_driver.h"
#include "stm32f411xce_i2c_driver.h"

#endif
