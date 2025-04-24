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
#include "stdio.h"
#include "arm_math.h"
#include <errno.h>
#include <sys/unistd.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
static const float32_t h[] = {
  -0.00000000,    0.00000000,    -0.00000002,    -0.00000014,    -0.00000034,    -0.00000059,    -0.00000078,    -0.00000075,    -0.00000038,    0.00000037,    0.00000144,    0.00000261,    0.00000356,    0.00000391,    0.00000336,    0.00000175,    -0.00000082,    -0.00000396,    -0.00000705,    -0.00000929,    -0.00000994,    -0.00000846,    -0.00000473,    0.00000089,    0.00000747,    0.00001373,    0.00001817,    0.00001948,    0.00001680,    0.00001004,    0.00000000,    -0.00001168,    -0.00002271,    -0.00003063,    -0.00003328,    -0.00002931,    -0.00001863,    -0.00000259,    0.00001616,    0.00003403,    0.00004716,    0.00005218,    0.00004706,    0.00003161,    0.00000782,    -0.00002037,    -0.00004762,    -0.00006821,    -0.00007715,    -0.00007132,    -0.00005033,    -0.00001687,    0.00002352,    0.00006328,    0.00009425,    0.00010921,    0.00010354,    0.00007639,    0.00003119,    -0.00002459,    -0.00008063,    -0.00012562,    -0.00014943,    -0.00014533,    -0.00011165,    -0.00005257,    0.00002223,    0.00009903,    0.00016254,    0.00019887,    0.00019842,    0.00015823,    0.00008311,    -0.00001474,    -0.00011751,    -0.00020502,    -0.00025853,    -0.00026467,    -0.00021853,    -0.00012529,    -0.00000000,    0.00013476,    0.00025279,    0.00032932,    0.00034601,    0.00029516,    0.00018199,    0.00002456,    -0.00014899,    -0.00030525,    -0.00041194,    -0.00044439,    -0.00039100,    -0.00025648,    -0.00006200,    0.00015793,    0.00036136,    0.00050687,    0.00056175,    0.00050915,    0.00035246,    0.00011594,    -0.00015874,    -0.00041965,    -0.00061430,    -0.00069998,    -0.00065290,    -0.00047405,    -0.00019058,    0.00014795,    0.00047807,    0.00073408,    0.00086093,    0.00082580,    0.00062589,    0.00029081,    -0.00012138,    -0.00053398,    -0.00086567,    -0.00104635,    -0.00103167,    -0.00081320,    -0.00042226,    0.00007405,    0.00058404,    0.00100812,    0.00125798,    0.00127469,    0.00104191,    0.00059152,    0.00000000,    -0.00062416,    -0.00116008,    -0.00149761,    -0.00155960,    -0.00131888,    -0.00080633,    -0.00010789,    0.00064930,    0.00131975,    0.00176722,    0.00189198,    0.00165237,    0.00107606,    0.00025827,    -0.00065336,    -0.00148497,    -0.00206928,    -0.00227873,    -0.00205258,    -0.00141235,    -0.00046187,    0.00062879,    0.00165322,    0.00240720,    0.00272892,    0.00253279,    0.00183023,    0.00073246,    -0.00056612,    -0.00182166,    -0.00278602,    -0.00325510,    -0.00311109,    -0.00235002,    -0.00108845,    0.00045298,    0.00198727,    0.00321364,    0.00387562,    0.00381348,    0.00300060,    0.00155571,    -0.00027247,    -0.00214685,    -0.00370300,    -0.00461875,    -0.00467940,    -0.00382545,    -0.00217283,    -0.00000000,    0.00229720,    0.00427603,    0.00553048,    0.00577239,    0.00489442,    0.00300150,    0.00040303,    -0.00243513,    -0.00497168,    -0.00669046,    -0.00720226,    -0.00632836,    -0.00414868,    -0.00100301,    0.00255766,    0.00586367,    0.00824818,    0.00917626,    0.00835757,    0.00582005,    0.00192811,    -0.00266203,    -0.00710586,    -0.01051758,    -0.01213635,    -0.01148204,    -0.00847100,    -0.00346714,    0.00274585,    0.00907248,    0.01428039,    0.01721631,    0.01702809,    0.01335423,    0.00644553,    -0.00280718,    -0.01295127,    -0.02215097,    -0.02844442,    -0.03004225,    -0.02562266,    -0.01457564,    0.00284458,    0.02548751,    0.05140517,    0.07807225,    0.10269904,    0.12259336,    0.13551927,    0.13999987,    0.13551927,    0.12259336,    0.10269904,    0.07807225,    0.05140517,    0.02548751,    0.00284458,    -0.01457564,    -0.02562266,    -0.03004225,    -0.02844442,    -0.02215097,    -0.01295127,    -0.00280718,    0.00644553,    0.01335423,    0.01702809,    0.01721631,    0.01428039,    0.00907248,    0.00274585,    -0.00346714,    -0.00847100,    -0.01148204,    -0.01213635,    -0.01051758,    -0.00710586,    -0.00266203,    0.00192811,    0.00582005,    0.00835757,    0.00917626,    0.00824818,    0.00586367,    0.00255766,    -0.00100301,    -0.00414868,    -0.00632836,    -0.00720226,    -0.00669046,    -0.00497168,    -0.00243513,    0.00040303,    0.00300150,    0.00489442,    0.00577239,    0.00553048,    0.00427603,    0.00229720,    -0.00000000,    -0.00217283,    -0.00382545,    -0.00467940,    -0.00461875,    -0.00370300,    -0.00214685,    -0.00027247,    0.00155571,    0.00300060,    0.00381348,    0.00387562,    0.00321364,    0.00198727,    0.00045298,    -0.00108845,    -0.00235002,    -0.00311109,    -0.00325510,    -0.00278602,    -0.00182166,    -0.00056612,    0.00073246,    0.00183023,    0.00253279,    0.00272892,    0.00240720,    0.00165322,    0.00062879,    -0.00046187,    -0.00141235,    -0.00205258,    -0.00227873,    -0.00206928,    -0.00148497,    -0.00065336,    0.00025827,    0.00107606,    0.00165237,    0.00189198,    0.00176722,    0.00131975,    0.00064930,    -0.00010789,    -0.00080633,    -0.00131888,    -0.00155960,    -0.00149761,    -0.00116008,    -0.00062416,    0.00000000,    0.00059152,    0.00104191,    0.00127469,    0.00125798,    0.00100812,    0.00058404,    0.00007405,    -0.00042226,    -0.00081320,    -0.00103167,    -0.00104635,    -0.00086567,    -0.00053398,    -0.00012138,    0.00029081,    0.00062589,    0.00082580,    0.00086093,    0.00073408,    0.00047807,    0.00014795,    -0.00019058,    -0.00047405,    -0.00065290,    -0.00069998,    -0.00061430,    -0.00041965,    -0.00015874,    0.00011594,    0.00035246,    0.00050915,    0.00056175,    0.00050687,    0.00036136,    0.00015793,    -0.00006200,    -0.00025648,    -0.00039100,    -0.00044439,    -0.00041194,    -0.00030525,    -0.00014899,    0.00002456,    0.00018199,    0.00029516,    0.00034601,    0.00032932,    0.00025279,    0.00013476,    -0.00000000,    -0.00012529,    -0.00021853,    -0.00026467,    -0.00025853,    -0.00020502,    -0.00011751,    -0.00001474,    0.00008311,    0.00015823,    0.00019842,    0.00019887,    0.00016254,    0.00009903,    0.00002223,    -0.00005257,    -0.00011165,    -0.00014533,    -0.00014943,    -0.00012562,    -0.00008063,    -0.00002459,    0.00003119,    0.00007639,    0.00010354,    0.00010921,    0.00009425,    0.00006328,    0.00002352,    -0.00001687,    -0.00005033,    -0.00007132,    -0.00007715,    -0.00006821,    -0.00004762,    -0.00002037,    0.00000782,    0.00003161,    0.00004706,    0.00005218,    0.00004716,    0.00003403,    0.00001616,    -0.00000259,    -0.00001863,    -0.00002931,    -0.00003328,    -0.00003063,    -0.00002271,    -0.00001168,    0.00000000,    0.00001004,    0.00001680,    0.00001948,    0.00001817,    0.00001373,    0.00000747,    0.00000089,    -0.00000473,    -0.00000846,    -0.00000994,    -0.00000929,    -0.00000705,    -0.00000396,    -0.00000082,    0.00000175,    0.00000336,    0.00000391,    0.00000356,    0.00000261,    0.00000144,    0.00000037,    -0.00000038,    -0.00000075,    -0.00000078,    -0.00000059,    -0.00000034,    -0.00000014,    -0.00000002,    0.00000000,    -0.00000000
};


#define WAV_WRITE_SAMPLE_COUNT 1024 * 4  
int16_t  data_i2s[WAV_WRITE_SAMPLE_COUNT];
float32_t mic1_data1[WAV_WRITE_SAMPLE_COUNT / 4],
	mic1_data2[WAV_WRITE_SAMPLE_COUNT / 4];
float32_t data_out_fft1[WAV_WRITE_SAMPLE_COUNT / 4],
	data_out_fft2[WAV_WRITE_SAMPLE_COUNT / 4];
volatile int16_t sample_i2s;
volatile uint8_t button_flag, start_stop_recording;
volatile uint8_t  half_i2s, full_i2s;
arm_rfft_fast_instance_f32 fft_audio_instance;

#define FILTER_TAP_NUM 201
float32_t filtered_data[WAV_WRITE_SAMPLE_COUNT / 4];
arm_fir_instance_f32 fir_instance;
float32_t fir_state[WAV_WRITE_SAMPLE_COUNT / 4 + FILTER_TAP_NUM - 1];



#define UART_TX_BUFFER_SIZE 512 
static uint32_t uart_tx_buffer[512];
static volatile uint8_t uart_tx_busy = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uart_tx_busy = 0;
    }
}


// int _write(int file, char *ptr, int len)
// {
//     if (file != STDOUT_FILENO && file != STDERR_FILENO) {
//         errno = EBADF;
//         return -1;
//     }

//     HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
//     return len;
// }

void send_uart(char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
}


void send_fft_data(float32_t* fft_data, size_t data_size)
{
    while (uart_tx_busy) {
        // Wait for previous transfer to complete
    }

    // Copy data directly as it's already 32-bit aligned
    memcpy(uart_tx_buffer, fft_data, data_size * 4);
    
    // Ensure the end marker is properly set
    uart_tx_buffer[data_size-1] = ('>'<<24|'>'<<16|'>'<<8|'>');

    uart_tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_tx_buffer, data_size * 4);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t uart_counter = 0;
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
  MX_I2S2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2S_DMAStop(&hi2s2);
  HAL_Delay(500);
  arm_rfft_fast_init_f32(&fft_audio_instance, WAV_WRITE_SAMPLE_COUNT / 4);
  arm_fir_init_f32(&fir_instance, FILTER_TAP_NUM, (float32_t *)h, fir_state, WAV_WRITE_SAMPLE_COUNT / 4);

  char *msg = "Hello World!\r\n";
  send_uart(msg, strlen(msg));

  start_stop_recording = 1;

  msg = "start_recording\r\n";
  send_uart(msg, strlen(msg));
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); // Turn on the LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Turn on the LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); // Turn off the LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // Turn off the LED
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)data_i2s, sizeof(data_i2s)/2);
  HAL_Delay(1000);
  full_i2s = 0;
  half_i2s = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);  // Toggle PB4
	  // HAL_Delay(10);  
        // Delay for 500ms (0.5 seconds)
    // Add LED flashing during recording
    static uint32_t led_timestamp = 0;
    if(start_stop_recording)
    {
        if(HAL_GetTick() - led_timestamp > 500) // Flash every 500ms
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12); // Toggle the LED pin
            led_timestamp = HAL_GetTick();
        }
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
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
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
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

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
//	l,r,l,r,l,

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12); // Toggle the LED pin
    full_i2s = 1;
    
    // Process the data here instead of in main loop
    for(int i = 0; i < WAV_WRITE_SAMPLE_COUNT / 4; i++ )
    {
        mic1_data2[i] = (float32_t)data_i2s[i * 2 + WAV_WRITE_SAMPLE_COUNT / 2];
    }
    // Apply FIR filter
    arm_fir_f32(&fir_instance, mic1_data2, filtered_data, WAV_WRITE_SAMPLE_COUNT / 4);
    
    // Apply FFT to filtered data
    arm_rfft_fast_f32(&fft_audio_instance, filtered_data, data_out_fft2, 0);
    arm_cmplx_mag_f32(data_out_fft2, data_out_fft2, WAV_WRITE_SAMPLE_COUNT / 8);
    data_out_fft2[0] = 0;  // bias removal

    // Send data
    uint8_t start_marker = 0xAA;
    HAL_UART_Transmit(&huart2, &start_marker, 1, 10);
    HAL_UART_Transmit(&huart2, (uint8_t *)data_out_fft2, WAV_WRITE_SAMPLE_COUNT / 8 * sizeof(float32_t), 100);
    uint8_t end_marker = 0x55;
    HAL_UART_Transmit(&huart2, &end_marker, 1, 10);
    
    // Restart DMA
    HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)data_i2s, sizeof(data_i2s)/2);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
    half_i2s = 1;
    
    // Process the data here instead of in main loop
    for(int i = 0; i < WAV_WRITE_SAMPLE_COUNT / 4; i++ )
    {
        mic1_data1[i] = (float32_t)data_i2s[i * 2];
    }
    // Apply FIR filter
    arm_fir_f32(&fir_instance, mic1_data1, filtered_data, WAV_WRITE_SAMPLE_COUNT / 4);
    
    // Apply FFT to filtered data
    arm_rfft_fast_f32(&fft_audio_instance, filtered_data, data_out_fft1, 0);
    arm_cmplx_mag_f32(data_out_fft1, data_out_fft1, WAV_WRITE_SAMPLE_COUNT / 8);
    data_out_fft1[0] = 0;  // bias removal

    // Send data
    uint8_t start_marker = 0xAA;
    HAL_UART_Transmit(&huart2, &start_marker, 1, 10);
    HAL_UART_Transmit(&huart2, (uint8_t *)data_out_fft1, WAV_WRITE_SAMPLE_COUNT / 8 * sizeof(float32_t), 100);
    uint8_t end_marker = 0x55;
    HAL_UART_Transmit(&huart2, &end_marker, 1, 10);
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
