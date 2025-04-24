/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIR_FILTER_ENABLE 1  // Set to 0 to disable filtering
#define NUM_TAPS 63          // Number of filter coefficients
#define BUTTON_PIN GPIO_PIN_6
#define BUTTON_PORT GPIOC
#define FILTER_LED_PIN GPIO_PIN_11
#define FILTER_LED_PORT GPIOC
#define BUFFER_SIZE 64     // Match WAV_WRITE_SAMPLE_COUNT
volatile uint8_t uartBusy = 0;
volatile uint8_t half_buffer = 0, full_buffer = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#define LED_PIN GPIO_PIN_4
#define LED_PORT GPIOB

// Buffer types for 24-bit data
uint32_t i2sBuffer[BUFFER_SIZE];
uint32_t filteredBuffer[BUFFER_SIZE];
uint8_t uartBuffer[BUFFER_SIZE * 3]; // 3 bytes per sample for 24-bit
uint32_t toggleCounter = 0;
uint32_t ledBlinkRate = 100;
uint8_t useFilteredData = 0; // Flag to switch between filtered and unfiltered data

// FIR filter state and instance
float32_t firStateF32[BUFFER_SIZE + NUM_TAPS - 1];
arm_fir_instance_f32 firFilter;

// FIR filter coefficients (from your advanced code)
const float32_t firCoeffs32[NUM_TAPS] = {
    -0.0000000000f, 0.0000001490f, 0.0000006141f, 0.0000014047f, 0.0000025049f, 0.0000038729f, 0.0000054408f, 0.0000071151f, 
    0.0000087777f, 0.0000102886f, 0.0000114883f, 0.0000122015f, 0.0000122418f, 0.0000114166f, 0.0000095331f, 0.0000064047f, 
    0.0000018580f, -0.0000042602f, -0.0000120747f, -0.0000216747f, -0.0000331060f, -0.0000463638f, -0.0000613856f, -0.0000780455f, 
    -0.0000961486f, -0.0001154272f, -0.0001355383f, -0.0001560622f, -0.0001765041f, -0.0001962960f, -0.0002148027f, -0.0002313285f, 
    -0.0002451273f, -0.0002554144f, -0.0002613811f, -0.0002622113f, -0.0002571005f, -0.0002452764f, -0.0002260214f, -0.0001986959f, 
    -0.0001627627f, -0.0001178119f, -0.0000635852f, 0.0000000000f, 0.0000728290f, 0.0001545688f, 0.0002446513f, 0.0003422584f, 
    0.0004463128f, 0.0005554717f, 0.0006681271f, 0.0007824110f, 0.0008962069f, 0.0010071680f, 0.0011127412f, 0.0012101990f, 
    0.0012966769f, 0.0013692178f, 0.0014248220f, 0.0014605034f, 0.0014733498f, 0.0014605879f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LED_GPIO_Init(void);
static void BUTTON_GPIO_Init(void);
static void FILTER_LED_GPIO_Init(void);
void ProcessAndTransmitData(uint32_t* rawData, uint16_t size);
void ApplyFIRFilter(uint32_t* inputData, uint32_t* outputData, uint16_t size);
void TransmitDataOverUART(uint8_t* data, uint16_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LED_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void BUTTON_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
}

static void FILTER_LED_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = FILTER_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FILTER_LED_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(FILTER_LED_PORT, FILTER_LED_PIN, GPIO_PIN_RESET);
}

void ProcessAndTransmitData(uint32_t* data, uint16_t size)
{
  // Pack 24-bit data into UART buffer (3 bytes per sample)
  for (uint16_t i = 0; i < size; i++) {
    uint32_t sample = data[i] & 0xFFFFFF; // Extract 24-bit data
    
    // Store the 24-bit word in the UART buffer (little-endian)
    uartBuffer[i * 3] = (uint8_t)(sample & 0xFF);         // LSB
    uartBuffer[i * 3 + 1] = (uint8_t)((sample >> 8) & 0xFF);
    uartBuffer[i * 3 + 2] = (uint8_t)((sample >> 16) & 0xFF); // MSB
  }
  
  // Transmit the data over UART
  TransmitDataOverUART(uartBuffer, size * 3);
}

// New function to handle UART transmission with proper error checking
void TransmitDataOverUART(uint8_t* data, uint16_t size)
{
  // Only transmit if UART is not busy and data is valid
  if (!uartBusy && data != NULL && size > 0) {
    uartBusy = 1;
    
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, data, size);
    
  }
}

void ApplyFIRFilter(uint32_t* inputData, uint32_t* outputData, uint16_t size)
{
  float32_t inputFloat[BUFFER_SIZE];
  float32_t outputFloat[BUFFER_SIZE];
  
  // Convert uint32_t to float32_t for FIR processing
  // For 24-bit data, we need to sign-extend if the MSB (bit 23) is set
  for (uint16_t i = 0; i < size; i++) {
    // Extract the 24-bit value from the 32-bit container
    int32_t sample = inputData[i] & 0xFFFFFF;
    
    // Sign extension for 24-bit data in 2's complement
    if (sample & 0x800000) {
      sample |= 0xFF000000; // Extend the sign bit
    }
    
    inputFloat[i] = (float32_t)sample;
  }
  

  arm_fir_f32(&firFilter, inputFloat, outputFloat, size);
  
  // Convert back to uint32_t (24-bit format)
  for (uint16_t i = 0; i < size; i++) {
    // Convert and clip to 24-bit range
    int32_t sample;
    
    if (outputFloat[i] > 8388607.0f) { // 2^23 - 1
      sample = 8388607;
    } else if (outputFloat[i] < -8388608.0f) { // -2^23
      sample = -8388608;
    } else {
      sample = (int32_t)outputFloat[i];
    }
    
    // Store only the lower 24 bits
    outputData[i] = sample & 0xFFFFFF;
  }
}


void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2)
  {
    // First half of buffer received
    half_buffer = 1;
    
    // Apply FIR filter to the first half of buffer if filtering is enabled
    if (FIR_FILTER_ENABLE) {
      ApplyFIRFilter(i2sBuffer, filteredBuffer, BUFFER_SIZE / 2);
    }
    
    // Choose which data to send based on button state
    if (useFilteredData) {
      ProcessAndTransmitData(filteredBuffer, BUFFER_SIZE / 2);
    } else {
      ProcessAndTransmitData(i2sBuffer, BUFFER_SIZE / 2);
    }
    
    // Blink LED at specified rate
    toggleCounter++;
    if (toggleCounter >= ledBlinkRate)
    {
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
      toggleCounter = 0;
    }
  }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2)
  {
    // Second half of buffer received
    full_buffer = 1;
    
    // Apply FIR filter to the second half of buffer if filtering is enabled
    if (FIR_FILTER_ENABLE) {
      ApplyFIRFilter(&i2sBuffer[BUFFER_SIZE / 2], &filteredBuffer[BUFFER_SIZE / 2], BUFFER_SIZE / 2);
    }
    
    // Choose which data to send based on button state
    if (useFilteredData) {
      ProcessAndTransmitData(&filteredBuffer[BUFFER_SIZE / 2], BUFFER_SIZE / 2);
    } else {
      ProcessAndTransmitData(&i2sBuffer[BUFFER_SIZE / 2], BUFFER_SIZE / 2);
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    uartBusy = 0; // Mark UART as available again
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    // Reset busy flag in case of error
    uartBusy = 0;
  }
}
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LED GPIO
  LED_GPIO_Init();
  
  // Initialize button GPIO
  BUTTON_GPIO_Init();
  
  // Initialize filter LED GPIO
  FILTER_LED_GPIO_Init();
  
  // Initialize FIR filter
  if (FIR_FILTER_ENABLE) {
    arm_fir_init_f32(&firFilter, NUM_TAPS, (float32_t *)firCoeffs32, firStateF32, BUFFER_SIZE / 2);
  }
  
  // Reset UART state
  uartBusy = 0;
  
  // Start I2S reception in DMA mode
  if (HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)i2sBuffer, BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  // Initial LED states
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FILTER_LED_PORT, FILTER_LED_PIN, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/* Infinite loop */
while (1)
{
  // Check current button state
  uint8_t buttonState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
  
  // Button pressed (LOW) = filtering ON, Button released (HIGH) = filtering OFF
  if (buttonState == GPIO_PIN_RESET) { // Button is pressed
    if (!useFilteredData) { // Only update if state changes
      useFilteredData = 1;
      HAL_GPIO_WritePin(FILTER_LED_PORT, FILTER_LED_PIN, GPIO_PIN_SET);
      ledBlinkRate = 25; // Faster blink rate for filtered mode
    }
  } else { // Button is released
    if (useFilteredData) { // Only update if state changes
      useFilteredData = 0;
      HAL_GPIO_WritePin(FILTER_LED_PORT, FILTER_LED_PIN, GPIO_PIN_RESET);
      ledBlinkRate = 100; // Slower blink rate for unfiltered mode
    }
  }
  
  // If UART is stuck busy for too long, reset it
  static uint32_t uartTimeoutCounter = 0;
  if (uartBusy) {
    uartTimeoutCounter++;
    if (uartTimeoutCounter > 1000) { // ~10 seconds with 10ms delay
      uartBusy = 0;
      HAL_UART_AbortTransmit(&huart2);
      uartTimeoutCounter = 0;
    }
  } else {
    uartTimeoutCounter = 0;
  }
  
  HAL_Delay(10); // Keep this delay for button debouncing
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 160, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(160000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLI2SM_DIV_8, 192, LL_RCC_PLLI2SR_DIV_2);
  LL_RCC_PLLI2S_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLLI2S_IsReady() != 1)
  {

  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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