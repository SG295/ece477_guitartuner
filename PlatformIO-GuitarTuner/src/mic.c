#include "mic.h"
#include "oled.h" // need for testing

#define BUFFER_SIZE 4096
#define SAMPLE_RATE 41800.0f

// #define TESTING_OLED

// FFT Inits
arm_rfft_fast_instance_f32 fft;
uint32_t *active_samps;
uint32_t samples_A[BUFFER_SIZE] = {0}; // array to hold all the samples for FFT processing 
uint32_t samples_B[BUFFER_SIZE] = {0};
float32_t input_buffer[BUFFER_SIZE/2];      // for time-domain data
float32_t output_buffer[BUFFER_SIZE/2];     // for FFT output data 
float32_t mag[BUFFER_SIZE / 4];           // for magnitudes
uint32_t max_index;
float32_t max_value;

void clock_enable()
{
    RCC -> CR |= RCC_CR_HSION; // enable external oscillator
    while(!(RCC->CR & RCC_CR_HSIRDY)); // wait until it's ready

    // Set PLL source to HSE - sets it for PLLI2S as well - and set PLLM for PLLI2S calc
    RCC -> PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI | (8 << RCC_PLLCFGR_PLLM_Pos);
    
    // (8MHz * (192/8))/5 = 38.4MHz 
    RCC -> PLLI2SCFGR = (192 << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (1 << RCC_PLLI2SCFGR_PLLI2SR_Pos);
    RCC -> CR |= RCC_CR_PLLI2SON; // enable PLLI2S
    while(!(RCC->CR & RCC_CR_PLLI2SRDY)); // wait until it's ready
}   

void init_gpio_mic()
{
    // PB10 - I2S2CLK; PB12 - I2S2WS; PC3 - I2S2SD
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // Pin configurations - set to alt funct
    GPIOB -> MODER &= ~0x3300000;   // Clear 10 and 12 
    GPIOB -> MODER |= 0x2200000;    // Set 10 and 12 to AF
    GPIOC -> MODER &= ~0xC0;        // Clear 3
    GPIOC -> MODER |= 0x80;         // Set 3 to AF

    // Alternate function mode - AF5
    GPIOB -> AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL12);
    GPIOB -> AFR[1] |= ((0x5 << GPIO_AFRH_AFSEL10_Pos) | (0x5 << GPIO_AFRH_AFSEL12_Pos)); 
    GPIOC -> AFR[0] &= ~(GPIO_AFRL_AFSEL3);
    GPIOC -> AFR[0] |= (0x5 << GPIO_AFRL_AFSEL3_Pos); 
}

void init_i2s_mic()
{
    RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN; // enable SPI2 clock

    SPI2 -> I2SCFGR &= ~SPI_I2SCFGR_I2SE; // disable channel

    SPI2 -> I2SCFGR |= SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_DATLEN_0 | SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_CHLEN;

    /*
    NOTE: Prescalar was off, not sure why, investigate moving forward. 
    */
    SPI2 -> I2SPR |= 0x1 | SPI_I2SPR_MCKOE; // <-- CHECK THIS, should get 38.4MHz/(6*2) = 3.2MHz/64 = 50kHz sample rate

    SPI2 -> CR2 |= SPI_CR2_RXDMAEN; // | SPI_CR2_RXNEIE; // enable DMA transfers whenever RXNE flag is set

    SPI2 -> I2SCFGR |= SPI_I2SCFGR_I2SE; // enable channel

    // NVIC_EnableIRQ(SPI2_IRQn); 
}

// uint32_t samples_A[BUFFER_SIZE] = {0}; // array to hold all the samples for FFT processing 
// uint32_t samples_B[BUFFER_SIZE] = {0};
// float input_buffer[BUFFER_SIZE];      // for time-domain data
// float output_buffer[BUFFER_SIZE];     // for FFT output data 
// float mag[BUFFER_SIZE / 2];           // for magnitudes
// uint32_t max_index;
// float max_value;

void i2s_dma()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_DMA1EN; // DMA1 Channel 0, Stream 3 is SPI2_RX

    DMA1_Stream3 -> CR &= ~(DMA_SxCR_EN); // turn off stream and ensure channel is 0 for SPI2 
    while(DMA1_Stream3 -> CR & DMA_SxCR_EN); // ensure enable is off
    DMA1_Stream3 -> CR &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR);
    // memory size to 32 bits, peripheral size to 32 bits, memory increment, circular mode, and transfer completed interrupt enable
    DMA1_Stream3 -> CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE; 
    DMA1_Stream3 -> NDTR = BUFFER_SIZE; // transfer 100 items each transfer
    DMA1_Stream3 -> PAR = (uint32_t)&(SPI2->DR); // peripheral address
    DMA1_Stream3 -> M0AR = (uint32_t)samples_A; // memory address 1
    // DMA1_Stream3 -> M1AR = (uint32_t)samples_B; // memory address 2
    // DMA1_Stream3 -> CR |= DMA_SxCR_DBM; // double buffer mode
    DMA1_Stream3 -> CR &= ~DMA_SxCR_DBM; // double buffer mode
    NVIC_EnableIRQ(DMA1_Stream3_IRQn); // enable stream 3 interrupt
    NVIC_SetPriority(DMA1_Stream3_IRQn, 1);
}

void i2s_dma_disable() // might not need this but could be good to have...we'll see :)
{
    DMA1_Stream3 -> CR &= ~DMA_SxCR_EN;
}

void i2s_dma_enable() // might not need this but could be good to have...we'll see :)
{
    DMA1_Stream3 -> CR |= DMA_SxCR_EN;
}

void uart_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA2 to alternate function (AF7 = USART2)
    GPIOA->MODER &= ~(3 << (2 * 2));  // Clear mode
    GPIOA->MODER |= (2 << (2 * 2));   // AF mode
    GPIOA->AFR[0] |= (7 << (4 * 2));  // AF7 for PA2

    USART2->CR1 &= ~USART_CR1_UE;
    uint16_t uartdiv = SystemCoreClock / (115200*4); // not sure why this didn't work, was off by factor of 4 (28.8kHz)
    USART2->BRR = uartdiv; // (((91) << USART_BRR_DIV_Mantissa_Pos) | ((3) << USART_BRR_DIV_Fraction_Pos));
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // Enable transmitter and USART
}

void uart_send_char(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void uart_send_string(const char *s) {
    while (*s) uart_send_char(*s++);
}

#ifdef TESTING_OLED
uint32_t current_sample = 0;
int calculate_average(uint32_t array[], int size) {
    int sum = 0;
    for (int i = 0; i < size; i++) {
        current_sample = array[i];
        array[i] = (__RBIT(current_sample & 0xFFFF0000) << 16) | (__RBIT(current_sample & 0xFFFF) >> 16); // reverse lower and upper 16 seperately since 24 bit data
        sum += array[i]; 
    }
    return ((sum / size)>>8);
  }
#endif

void DMA1_Stream3_IRQHandler(void)
{
    i2s_dma_disable();
    // uint32_t sanity_check = DMA1_Stream3->CR & DMA_SxCR_CT;
    if(DMA1->LISR & DMA_LISR_TCIF3) // check that it's a stream 3 transfer complete interrupt
    {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF3; // clear flag
        // uint32_t sanity_check = DMA1_Stream3->CR & DMA_SxCR_CT;
        active_samps = samples_A;
        // if(DMA1_Stream3->CR & DMA_SxCR_CT) // if CT is 1, current target is mem 1
        // {
        //     active_samps = samples_A; // so that means A should be done...I think :)
        // }
        // else // else is 0 and target is mem 0 
        // {
        //     active_samps = samples_A;
        // }

        // ------FFT------
        // --- Convert raw mic samples to float [-1, 1] ---
        for (int i = 0; i < BUFFER_SIZE/2; i++) {
            float32_t s = (process_sample(active_samps[i*2])); // / 32768;
            // char buf[20];
            // sprintf(buf, "%ld", s);
            // uart_send_string(buf);
            input_buffer[i] = s; // / 131072.0f; // Normalize 18-bit signed because ARM library expects input like this
        }
        // --- Apply real FFT (in-place) ---
        arm_rfft_fast_init_f32(&fft, BUFFER_SIZE/2);
        arm_rfft_fast_f32(&fft, input_buffer, output_buffer, 0);

        // --- Compute magnitude ---
        // for (int i = 0; i < BUFFER_SIZE / 4; i++) {
        //     float real = output_buffer[2 * i];
        //     float imag = output_buffer[(2 * i) + 1];
        //     mag[i] = sqrtf((real * real) + (imag * imag));
        // }
        arm_cmplx_mag_f32(output_buffer, mag, BUFFER_SIZE/4);

        char buf1[20];
        char buf2[20];
        for(int i=0; i < BUFFER_SIZE/64; i++)
        {
            uart_send_string("Frequency");
            float32_t bin = ((float32_t)(i*SAMPLE_RATE) / (BUFFER_SIZE/2));
            sprintf(buf1, "%d", (int)bin);
            uart_send_string(buf1);
            uart_send_string(": ");
            sprintf(buf2, "%d", (int)mag[i]);
            uart_send_string(buf2);
            uart_send_string("\r\n");
        }

        arm_max_f32(mag, BUFFER_SIZE / 2, &max_value, &max_index);
        float detected_freq = ((float32_t)max_index * SAMPLE_RATE) / (BUFFER_SIZE/2);
        char freq_msg[40];
        sprintf(freq_msg, "Peak Frequency: %d Hz\r\n", (int)detected_freq);
        uart_send_string(freq_msg);
        // // --- Find the bin with the maximum magnitude ---
        // int peak_index = 1;  // Start at 1 to skip DC (bin 0, super low values like 0Hz that are common)
        // float peak_value = mag[1];

        // for (int i = 2; i < (BUFFER_SIZE / 4) - 1; i++) {
        //     if (mag[i] > peak_value) {
        //         peak_value = mag[i];
        //         peak_index = i;
        //     }
        // }

        // // --- Parabolic Interpolation --- 
        // float alpha = mag[peak_index - 1];
        // float beta  = mag[peak_index];
        // float gamma = mag[peak_index + 1];
        
        // float bin_offset = 0.5f * (alpha - gamma) / ((alpha - 2.0f) * (beta + gamma));
        // float interpolated_bin = peak_index + bin_offset;

        // float bin_width = SAMPLE_RATE / (float)1024;  // Fs / N
        // float frequency = interpolated_bin * bin_width;
        // // float frequency = peak_index * (SAMPLE_RATE / ((float)1024));
        
        // // --- OLED display (or UART) ---
        // char output_freq[20];
        // int int_frequency = (int)frequency;
        // sprintf(output_freq, "%d\n", int_frequency);
        // // uart_send_string("MAX FREQ:");
        // // uart_send_string(output_freq);
        // OLED_DrawString(0, 30, B_Color, BLACK, "            ", 12);
        // OLED_DrawString(0, 30, B_Color, BLACK, output_freq, 12);
    }
    // nano_wait(1000000000);
    i2s_dma_enable();
}

float32_t process_sample(uint32_t raw_data)
{
    int32_t sample = (((raw_data & 0xFFFF) << 2) | ((raw_data >> 30) & 0x3)) & 0x3FFFF; // raw_data & 0x3FFFF; 
    if (sample & 0x20000) 
    {
        sample |= 0xFFFC0000; // sign extend if needed since data is 2s compliment
    }
    float32_t float_samp = (float32_t)sample / 131072.0f;
    return float_samp; 
}