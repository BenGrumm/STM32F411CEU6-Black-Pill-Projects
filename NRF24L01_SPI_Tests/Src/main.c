/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#undef RECEIVER
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
NRF24L01 nrf;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void printAddrRegs(void);
void setCEPin(GPIO_PinState state);
void setCSNPin(GPIO_PinState state);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // B2 - CSN
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // B1 - CE

  HAL_StatusTypeDef err = HAL_SPI_Init(&hspi1);

  HAL_Delay(500);

  // Common
  nrf.spiHandler = &hspi1;
  nrf.rfChannel = 0x7B;
  nrf.NRF_setCEPin = &setCEPin;
  nrf.NRF_setCSNPin = &setCSNPin;
  nrf.crcScheme = NRF_CRC_1_BYTE;
  nrf.enableCRC = true;
  nrf.enableAutoAck = true;
  nrf.enableDynamicPlWidth = true;
  nrf.addressWidth = NRF_ADDRES_WIDTH_5_BYTES;
  nrf.payloadWidth = 10;
  nrf.transmitSpeed = NRF_SPEED_2Mbps;

  // For transmitter
  #ifndef RECEIVER
  printf("Transmiter\n");
  nrf.mode = NRF_MODE_TRANSMITTER;
  nrf.enableMaxRtInterrupt = true;
  nrf.enableRxDrInterrupt = true;
  nrf.enableTxDsInterrupt = true;
  nrf.autoRetransmitDelay = 0b0110;
  nrf.autoRetransmitCount = 0b1001;
  nrf.tx_address = 0x28AABBCCDD;

  uint8_t receiveBuffer[10] = {0};
  uint8_t receiverAddr[5];
  uint8_t sendCount = 0;
  NRF24L01_getLSBToMSBArray(nrf.tx_address, receiverAddr);
  #endif

  // For receiver
  #ifdef RECEIVER
  printf("Receiver\n");
  nrf.rxPipe = NRF_PIPE_0;
  nrf.mode = NRF_MODE_RECEIVER;
  nrf.enableMaxRtInterrupt = true;
  nrf.enableRxDrInterrupt = true;
  nrf.enableTxDsInterrupt = true;
  nrf.rx_address = 0x28AABBCCDD;
  #endif

  HAL_Delay(5000);

  NRF24L01_setup(&nrf);

  HAL_Delay(100);

  uint32_t flashTime = HAL_GetTick();

  #ifdef RECEIVER
  // Start listening (Set CE high)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  #else
  sprintf((char*)receiveBuffer, "RX: %d\n", sendCount);
  uint32_t sendTime = 0;
  uint32_t numLoops = 0;
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // printAddrRegs();

    // if(HAL_SPI_GetState(nrf.spiHandler) == HAL_SPI_STATE_READY){
    //   NRF24L01_readRegister(&nrf, NRF_REG_STATUS, &nrf.status, 1);
    //   printf("Status - "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(nrf.status));
    // }

    #ifdef RECEIVER
    if(NRF24L01_receive(&nrf)){
      printf("Val = %s", (volatile char*)nrf.data);
    }
    #else

    NRF24L01_transmitDMALoop_New(&nrf);
    numLoops++;

    if((HAL_GetTick() - sendTime) > 100 && NRF24L01_transmitDMA_New(&nrf, receiverAddr, receiveBuffer, strlen((char*)receiveBuffer))){
      sendCount++;
      sendTime = HAL_GetTick();
      printf("Send: %s \nNum Loop: %ld\n", receiveBuffer, numLoops);
      
      sprintf((char*)receiveBuffer, "RX: %d\n", sendCount);

      numLoops = 0;
    }

    #endif

    if(HAL_GetTick() - flashTime > 500){
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      flashTime = HAL_GetTick();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void printAddrRegs(){
  uint8_t receiveBuffer[32];
  NRF24L01_readRegister(&nrf, NRF_REG_RX_ADDR_P0, receiveBuffer, 5);
  printf("ADDR 0 = %x:%x:%x:%x:%x\n", receiveBuffer[0], receiveBuffer[1], receiveBuffer[2], receiveBuffer[3], receiveBuffer[4]);
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_ADDR_P1, receiveBuffer, 5);
  printf("ADDR 1 = %x:%x:%x:%x:%x\n", receiveBuffer[0], receiveBuffer[1], receiveBuffer[2], receiveBuffer[3], receiveBuffer[4]);
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_ADDR_P2, receiveBuffer, 1);
  printf("ADDR 2 = %x\n", receiveBuffer[0]);
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_ADDR_P3, receiveBuffer, 1);
  printf("ADDR 3 = %x\n", receiveBuffer[0]);
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_ADDR_P4, receiveBuffer, 1);
  printf("ADDR 4 = %x\n", receiveBuffer[0]);
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_ADDR_P5, receiveBuffer, 1);
  printf("ADDR 5 = %x\n", receiveBuffer[0]);
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_PW_P0, receiveBuffer, 1);
  printf("PW 0 = "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(receiveBuffer[0]));
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_PW_P1, receiveBuffer, 1);
  printf("PW 1 = "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(receiveBuffer[0]));
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_PW_P2, receiveBuffer, 1);
  printf("PW 2 = "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(receiveBuffer[0]));
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_PW_P3, receiveBuffer, 1);
  printf("PW 3 = "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(receiveBuffer[0]));
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_PW_P4, receiveBuffer, 1);
  printf("PW 4 = "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(receiveBuffer[0]));
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_RX_PW_P5, receiveBuffer, 1);
  printf("PW 5 = "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(receiveBuffer[0]));
  HAL_Delay(50);
  NRF24L01_readRegister(&nrf, NRF_REG_TX_ADDR, receiveBuffer, 5);
  printf("TX Addr = %x:%x:%x:%x:%x\n", receiveBuffer[0], receiveBuffer[1], receiveBuffer[2], receiveBuffer[3], receiveBuffer[4]);
  HAL_Delay(50);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
  nrf.txCpltInterrupt = true;
  setCSNPin(GPIO_PIN_SET);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
  nrf.rxCpltInterrupt = true;
  setCSNPin(GPIO_PIN_SET);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  nrf.txRXCpltInterrupt = true;
  setCSNPin(GPIO_PIN_SET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  nrf.interruptTrigger = true;
}

void setCEPin(GPIO_PinState state){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, state);
}

void setCSNPin(GPIO_PinState state){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, state);
}

int _write(int file, char *ptr, int len)
{
  UNUSED(file);
  CDC_Transmit_FS((uint8_t*)ptr, len);
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
