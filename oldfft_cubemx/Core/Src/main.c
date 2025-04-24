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
typedef struct {
  float32_t magnitude;
  uint16_t bin_index;
} FrequencyPeak;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TAG_TOP_TEN_FREQ
//#define TAG_FFT_ALL
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
  -0.00000000,    0.00000001,    0.00000006,    0.00000017,    0.00000036,    0.00000059,    0.00000082,    0.00000095,    0.00000091,    0.00000061,    -0.00000000,    -0.00000092,    -0.00000210,    -0.00000341,    -0.00000467,    -0.00000568,    -0.00000618,    -0.00000598,    -0.00000490,    -0.00000289,    -0.00000000,    0.00000359,    0.00000754,    0.00001144,    0.00001475,    0.00001697,    0.00001759,    0.00001627,    0.00001282,    0.00000729,    -0.00000000,    -0.00000847,    -0.00001733,    -0.00002559,    -0.00003222,    -0.00003623,    -0.00003678,    -0.00003336,    -0.00002581,    -0.00001443,    0.00000000,    0.00001625,    0.00003276,    0.00004773,    0.00005933,    0.00006591,    0.00006616,    0.00005935,    0.00004544,    0.00002515,    -0.00000000,    -0.00002782,    -0.00005559,    -0.00008032,    -0.00009906,    -0.00010921,    -0.00010884,    -0.00009695,    -0.00007373,    -0.00004055,    -0.00000000,    0.00004430,    0.00008801,    0.00012645,    0.00015512,    0.00017011,    0.00016867,    0.00014952,    0.00011316,    0.00006194,    -0.00000000,    -0.00006708,    -0.00013270,    -0.00018988,    -0.00023199,    -0.00025342,    -0.00025032,    -0.00022107,    -0.00016670,    -0.00009093,    0.00000000,    0.00009780,    0.00019284,    0.00027507,    0.00033501,    0.00036484,    0.00035929,    0.00031638,    0.00023789,    0.00012940,    -0.00000000,    -0.00013842,    -0.00027220,    -0.00038725,    -0.00047045,    -0.00051106,    -0.00050205,    -0.00044104,    -0.00033084,    -0.00017954,    -0.00000000,    0.00019120,    0.00037518,    0.00053262,    0.00064569,    0.00069999,    0.00068626,    0.00060166,    0.00045045,    0.00024399,    -0.00000000,    -0.00025886,    -0.00050705,    -0.00071855,    -0.00086957,    -0.00094109,    -0.00092110,    -0.00080623,    -0.00060264,    -0.00032591,    0.00000000,    0.00034473,    0.00067423,    0.00095410,    0.00115300,    0.00124611,    0.00121799,    0.00106470,    0.00079482,    0.00042930,    -0.00000000,    -0.00045299,    -0.00088497,    -0.00125089,    -0.00151002,    -0.00163024,    -0.00159182,    -0.00139010,    -0.00103674,    -0.00055946,    0.00000000,    0.00058930,    0.00115031,    0.00162469,    0.00195979,    0.00211432,    0.00206311,    0.00180053,    0.00134206,    0.00072382,    -0.00000000,    -0.00076169,    -0.00148618,    -0.00209827,    -0.00253020,    -0.00272893,    -0.00266220,    -0.00232294,    -0.00173120,    -0.00093361,    0.00000000,    0.00098245,    0.00191709,    0.00270703,    0.00326495,    0.00352232,    0.00343733,    0.00300049,    0.00223721,    0.00120716,    -0.00000000,    -0.00127199,    -0.00248401,    -0.00351063,    -0.00423824,    -0.00457718,    -0.00447192,    -0.00390852,    -0.00291826,    -0.00157698,    0.00000000,    0.00166724,    0.00326198,    0.00461940,    0.00558888,    0.00604985,    0.00592545,    0.00519275,    0.00388820,    0.00210755,    -0.00000000,    -0.00224332,    -0.00440556,    -0.00626390,    -0.00761110,    -0.00827678,    -0.00814656,    -0.00717693,    -0.00540436,    -0.00294719,    0.00000000,    0.00317973,    0.00629179,    0.00901877,    0.01105499,    0.01213639,    0.01206873,    0.01075141,    0.00819471,    0.00452830,    -0.00000000,    -0.00503579,    -0.01013954,    -0.01481654,    -0.01855339,    -0.02085825,    -0.02130162,    -0.01955452,    -0.01542062,    -0.00885970,    0.00000000,    0.01086174,    0.02327305,    0.03665075,    0.05031649,    0.06354052,    0.07559032,    0.08578055,    0.09352059,    0.09835593,    0.10000030,    0.09835593,    0.09352059,    0.08578055,    0.07559032,    0.06354052,    0.05031649,    0.03665075,    0.02327305,    0.01086174,    0.00000000,    -0.00885970,    -0.01542062,    -0.01955452,    -0.02130162,    -0.02085825,    -0.01855339,    -0.01481654,    -0.01013954,    -0.00503579,    -0.00000000,    0.00452830,    0.00819471,    0.01075141,    0.01206873,    0.01213639,    0.01105499,    0.00901877,    0.00629179,    0.00317973,    0.00000000,    -0.00294719,    -0.00540436,    -0.00717693,    -0.00814656,    -0.00827678,    -0.00761110,    -0.00626390,    -0.00440556,    -0.00224332,    -0.00000000,    0.00210755,    0.00388820,    0.00519275,    0.00592545,    0.00604985,    0.00558888,    0.00461940,    0.00326198,    0.00166724,    0.00000000,    -0.00157698,    -0.00291826,    -0.00390852,    -0.00447192,    -0.00457718,    -0.00423824,    -0.00351063,    -0.00248401,    -0.00127199,    -0.00000000,    0.00120716,    0.00223721,    0.00300049,    0.00343733,    0.00352232,    0.00326495,    0.00270703,    0.00191709,    0.00098245,    0.00000000,    -0.00093361,    -0.00173120,    -0.00232294,    -0.00266220,    -0.00272893,    -0.00253020,    -0.00209827,    -0.00148618,    -0.00076169,    -0.00000000,    0.00072382,    0.00134206,    0.00180053,    0.00206311,    0.00211432,    0.00195979,    0.00162469,    0.00115031,    0.00058930,    0.00000000,    -0.00055946,    -0.00103674,    -0.00139010,    -0.00159182,    -0.00163024,    -0.00151002,    -0.00125089,    -0.00088497,    -0.00045299,    -0.00000000,    0.00042930,    0.00079482,    0.00106470,    0.00121799,    0.00124611,    0.00115300,    0.00095410,    0.00067423,    0.00034473,    0.00000000,    -0.00032591,    -0.00060264,    -0.00080623,    -0.00092110,    -0.00094109,    -0.00086957,    -0.00071855,    -0.00050705,    -0.00025886,    -0.00000000,    0.00024399,    0.00045045,    0.00060166,    0.00068626,    0.00069999,    0.00064569,    0.00053262,    0.00037518,    0.00019120,    -0.00000000,    -0.00017954,    -0.00033084,    -0.00044104,    -0.00050205,    -0.00051106,    -0.00047045,    -0.00038725,    -0.00027220,    -0.00013842,    -0.00000000,    0.00012940,    0.00023789,    0.00031638,    0.00035929,    0.00036484,    0.00033501,    0.00027507,    0.00019284,    0.00009780,    0.00000000,    -0.00009093,    -0.00016670,    -0.00022107,    -0.00025032,    -0.00025342,    -0.00023199,    -0.00018988,    -0.00013270,    -0.00006708,    -0.00000000,    0.00006194,    0.00011316,    0.00014952,    0.00016867,    0.00017011,    0.00015512,    0.00012645,    0.00008801,    0.00004430,    -0.00000000,    -0.00004055,    -0.00007373,    -0.00009695,    -0.00010884,    -0.00010921,    -0.00009906,    -0.00008032,    -0.00005559,    -0.00002782,    -0.00000000,    0.00002515,    0.00004544,    0.00005935,    0.00006616,    0.00006591,    0.00005933,    0.00004773,    0.00003276,    0.00001625,    0.00000000,    -0.00001443,    -0.00002581,    -0.00003336,    -0.00003678,    -0.00003623,    -0.00003222,    -0.00002559,    -0.00001733,    -0.00000847,    -0.00000000,    0.00000729,    0.00001282,    0.00001627,    0.00001759,    0.00001697,    0.00001475,    0.00001144,    0.00000754,    0.00000359,    -0.00000000,    -0.00000289,    -0.00000490,    -0.00000598,    -0.00000618,    -0.00000568,    -0.00000467,    -0.00000341,    -0.00000210,    -0.00000092,    -0.00000000,    0.00000061,    0.00000091,    0.00000095,    0.00000082,    0.00000059,    0.00000036,    0.00000017,    0.00000006,    0.00000001,    -0.00000000
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
int compare_peaks(const void* a, const void* b) {
  FrequencyPeak* peak_a = (FrequencyPeak*)a;
  FrequencyPeak* peak_b = (FrequencyPeak*)b;
  if (peak_a->magnitude > peak_b->magnitude) return -1;
  if (peak_a->magnitude < peak_b->magnitude) return 1;
  return 0;
}

void send_top_frequencies(float32_t* fft_data, uint16_t fft_size, uint32_t sample_rate) {
  // We only need to consider half of the FFT bins (due to Nyquist)
  uint16_t useful_bins = fft_size / 2;
  FrequencyPeak peaks[useful_bins];
  uint16_t peak_count = 0;
  
  // Calculate frequency resolution
  float32_t freq_resolution = (float32_t)sample_rate / fft_size;
  
  // Convert FFT data to frequency peaks (skip DC at index 0)
  for (uint16_t i = 1; i < useful_bins; i++) {
      float32_t frequency = i * freq_resolution;
      if (frequency < 1000.0f) { // Only include frequencies less than 1000Hz
          peaks[peak_count].magnitude = fft_data[i];
          peaks[peak_count].bin_index = i;
          peak_count++;
      }
  }
  
  // Sort peaks by magnitude
  qsort(peaks, peak_count, sizeof(FrequencyPeak), compare_peaks);

  
  // Send header
  // char *header = "Top 10 Frequencies:\r\n";
  // send_uart(header, strlen(header));
  
  // Prepare buffer for each frequency line
  char buffer[100];
  
  // Send top 10 frequencies
  for(int i = 0; i < 1 && i < peak_count; i++) {
      float32_t frequency = peaks[i].bin_index * freq_resolution;
      
      // Format each line manually
      char freq_str[20];
      char mag_str[20];
      
      // Convert float to string with fixed precision
      int freq_int = (int)frequency;
      int freq_dec = (int)((frequency - freq_int) * 10);
      int mag_int = (int)peaks[i].magnitude;
      int mag_dec = (int)((peaks[i].magnitude - mag_int) * 10);
      
      // Build the output string manually
      strcpy(buffer, "");
      
      // // Add index
      // if(i+1 < 10) {
      //     strcat(buffer, " ");  // Add space for alignment if single digit
      // }
      
      // // Convert index to string and append
      // char idx_str[5];
      // idx_str[0] = '0' + (i+1)/10;  // Tens digit or 0
      // idx_str[1] = '0' + (i+1)%10;  // Ones digit
      // idx_str[2] = '.';
      // idx_str[3] = ' ';
      // idx_str[4] = '\0';
      // if(idx_str[0] == '0') idx_str[0] = ' ';  // Remove leading 0
      // strcat(buffer, idx_str);
      
      // Add frequency
      strcat(buffer, "Freq: ");
      
      // Convert frequency to string
      freq_str[0] = '0' + freq_int/1000 % 10;  // Thousands
      freq_str[1] = '0' + freq_int/100 % 10;   // Hundreds
      freq_str[2] = '0' + freq_int/10 % 10;    // Tens
      freq_str[3] = '0' + freq_int % 10;       // Ones
      freq_str[4] = '.';
      freq_str[5] = '0' + freq_dec;            // Decimal
      freq_str[6] = ' ';
      freq_str[7] = 'H';
      freq_str[8] = 'z';
      freq_str[9] = ',';
      freq_str[10] = ' ';
      freq_str[11] = '\0';
      
      // Remove leading zeros
      if(freq_str[0] == '0') { 
          freq_str[0] = ' ';
          if(freq_str[1] == '0') {
              freq_str[1] = ' ';
              if(freq_str[2] == '0') {
                  freq_str[2] = ' ';
              }
          }
      }
      
      strcat(buffer, freq_str);
      
      // Add magnitude
      strcat(buffer, "Mag: ");
      
      // Convert magnitude to string
      mag_str[0] = '0' + mag_int/1000 % 10;  // Thousands
      mag_str[1] = '0' + mag_int/100 % 10;   // Hundreds
      mag_str[2] = '0' + mag_int/10 % 10;    // Tens
      mag_str[3] = '0' + mag_int % 10;       // Ones
      mag_str[4] = '.';
      mag_str[5] = '0' + mag_dec;            // Decimal
      mag_str[6] = '\r';
      mag_str[7] = '\n';
      mag_str[8] = '\0';
      
      // Remove leading zeros
      if(mag_str[0] == '0') { 
          mag_str[0] = ' ';
          if(mag_str[1] == '0') {
              mag_str[1] = ' ';
              if(mag_str[2] == '0') {
                  mag_str[2] = ' ';
              }
          }
      }
      
      strcat(buffer, mag_str);
      
      // Send the line
      send_uart(buffer, strlen(buffer));
  }
  
  // Send final line break
  char *footer = "\r\n";
  send_uart(footer, strlen(footer));
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

// RxHalfCpltCallback modification
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
    half_i2s = 1;
    
    // Process the data
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

    // Send data based on defined tags
#ifdef TAG_TOP_TEN_FREQ
    send_top_frequencies(data_out_fft1, WAV_WRITE_SAMPLE_COUNT / 4, 16000);
#endif

#ifdef TAG_FFT_ALL
    uint8_t start_marker = 0xAA;
    HAL_UART_Transmit(&huart2, &start_marker, 1, 10);
    HAL_UART_Transmit(&huart2, (uint8_t *)data_out_fft1, WAV_WRITE_SAMPLE_COUNT / 8 * sizeof(float32_t), 100);
    uint8_t end_marker = 0x55;
    HAL_UART_Transmit(&huart2, &end_marker, 1, 10);
#endif
}

// RxCpltCallback modification
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
    full_i2s = 1;
    
    for(int i = 0; i < WAV_WRITE_SAMPLE_COUNT / 4; i++ )
    {
        mic1_data2[i] = (float32_t)data_i2s[i * 2 + WAV_WRITE_SAMPLE_COUNT / 2];
    }
    
    arm_fir_f32(&fir_instance, mic1_data2, filtered_data, WAV_WRITE_SAMPLE_COUNT / 4);
    
    arm_rfft_fast_f32(&fft_audio_instance, filtered_data, data_out_fft2, 0);
    arm_cmplx_mag_f32(data_out_fft2, data_out_fft2, WAV_WRITE_SAMPLE_COUNT / 8);
    data_out_fft2[0] = 0;  // bias removal

    // Send data based on defined tags
#ifdef TAG_TOP_TEN_FREQ
    send_top_frequencies(data_out_fft2, WAV_WRITE_SAMPLE_COUNT / 4, 16000);
#endif

#ifdef TAG_FFT_ALL
    uint8_t start_marker = 0xAA;
    HAL_UART_Transmit(&huart2, &start_marker, 1, 10);
    HAL_UART_Transmit(&huart2, (uint8_t *)data_out_fft2, WAV_WRITE_SAMPLE_COUNT / 8 * sizeof(float32_t), 100);
    uint8_t end_marker = 0x55;
    HAL_UART_Transmit(&huart2, &end_marker, 1, 10);
#endif

    // Restart DMA
    HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)data_i2s, sizeof(data_i2s)/2);
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
