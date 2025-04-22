#include "mic.h"
#include "oled.h" // need for testing

#define BUFFER_SIZE 1024

// #define TESTING_OLED

void clock_enable()
{
    RCC -> CR |= RCC_CR_HSEON; // enable external oscillator
    while(!(RCC->CR & RCC_CR_HSERDY)); // wait until it's ready

    // Set PLL source to HSE - sets it for PLLI2S as well - and set PLLM for PLLI2S calc
    RCC -> PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE | (8 << RCC_PLLCFGR_PLLM_Pos);
    
    // (8MHz * (192/8))/5 = 38.4MHz 
    RCC -> PLLI2SCFGR = (192 << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (5 << RCC_PLLI2SCFGR_PLLI2SR_Pos);
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
    SPI2 -> I2SPR |= 0x2; // <-- CHECK THIS, should get 38.4MHz/(6*2) = 3.2MHz/64 = 50kHz sample rate

    SPI2 -> CR2 |= SPI_CR2_RXDMAEN; // | SPI_CR2_RXNEIE; // enable DMA transfers whenever RXNE flag is set

    SPI2 -> I2SCFGR |= SPI_I2SCFGR_I2SE; // enable channel

    // NVIC_EnableIRQ(SPI2_IRQn); 
}

uint32_t samples_A[BUFFER_SIZE] = {0}; // array to hold all the samples for FFT processing 
uint32_t samples_B[BUFFER_SIZE] = {0};
float input_buffer[BUFFER_SIZE];      // for time-domain data
float mag[BUFFER_SIZE / 2];           // for magnitudes
uint32_t max_index;
float max_value;

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
    DMA1_Stream3 -> CR |= DMA_SxCR_DBM; // double buffer mode
    DMA1_Stream3 -> M1AR = (uint32_t)samples_B; // memory address 2
    NVIC_EnableIRQ(DMA1_Stream3_IRQn); // enable stream 3 interrupt
}

void i2s_dma_disable() // might not need this but could be good to have...we'll see :)
{
    DMA1_Stream3 -> CR &= ~DMA_SxCR_EN;
}

void i2s_dma_enable() // might not need this but could be good to have...we'll see :)
{
    DMA1_Stream3 -> CR |= DMA_SxCR_EN;
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
    if(DMA1->LISR & DMA_LISR_TCIF3) // check that it's a stream 3 transfer complete interrupt
    {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF3; // clear flag
        uint32_t *active_samps;
        if(DMA1_Stream3->CR & DMA_SxCR_CT) // if CT is 1, current target is mem 1
        {
            active_samps = samples_A; // so that means A should be done...I think :)
        }
        else // else is 0 and target is mem 0 
        {
            active_samps = samples_B;
        }
        #ifdef TESTING_OLED
        char output_freq[20];
        int avrg = calculate_average(samples_A, 100);
        sprintf(output_freq, "%d", avrg);
        OLED_DrawString(0, 60, B_Color, BLACK, output_freq, 16);
        #endif

        // ------FFT------
        // Convert raw mic samples to float [-1, 1]
        for (int i = 0; i < BUFFER_SIZE; i++) {
            int32_t s = process_sample(active_samps[i]);
            input_buffer[i] = (float)s / 8388608.0f; // Normalize 24-bit signed
        }

        // Apply real FFT (in-place)
        arm_rfft_fast_instance_f32 fft;
        arm_rfft_fast_init_f32(&fft, BUFFER_SIZE);
        arm_rfft_fast_f32(&fft, input_buffer, input_buffer, 0);

        // Optionally compute magnitude
        for (int i = 0; i < BUFFER_SIZE / 2; i++) {
            float real = input_buffer[2 * i];
            float imag = input_buffer[2 * i + 1];
            mag[i] = sqrtf(real * real + imag * imag);
        }

        arm_max_f32(mag, BUFFER_SIZE / 2, &max_value, &max_index);

        char output_freq[20];
        sprintf(output_freq, "%d", (int)max_value);
        OLED_DrawString(0, 60, B_Color, BLACK, output_freq, 12);
    }
}

int32_t process_sample(uint32_t raw_data)
{
    int32_t sample = raw_data >> 8; 
    if (sample & 0x800000) sample |= 0xFF000000; // sign extend if needed
    return sample; 
}