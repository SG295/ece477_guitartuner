#include "mic.h"

#define TESTING_OLED
#ifdef TESTING_OLED
#include "oled.h" // need for testing
#endif

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

    SPI2 -> CR2 |= SPI_CR2_RXDMAEN; // enable DMA transfers whenever RXNE flag is set

    SPI2 -> I2SCFGR |= SPI_I2SCFGR_I2SE; // enable channel
}

int32_t samples[1000]; // array to hold all the samples for FFT processing 

void i2s_dma()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_DMA1EN; // DMA1 Channel 0, Stream 3 is SPI2_RX

    DMA1_Stream3 -> CR &= ~(DMA_SxCR_EN); // turn off stream and ensure channel is 0 for SPI2 
    while(DMA1_Stream3 -> CR & DMA_SxCR_EN); // ensure enable is off
    DMA1_Stream3 -> CR &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR);
    // memory size to 32 bits, peripheral size to 32 bits, memory increment, circular mode, and transfer completed interrupt enable
    DMA1_Stream3 -> CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE; 
    DMA1_Stream3 -> NDTR = 1000; // transfer 100 items each transfer
    DMA1_Stream3 -> PAR = (uint32_t)&(SPI2->DR); // peripheral address
    DMA1_Stream3 -> M0AR = (uint32_t)samples; // memory address
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
int calculate_average(int32_t array[], int size) {
    int sum = 0;
    for (int i = 0; i < size; i++) {
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

        #ifdef TESTING_OLED
        char output_freq[20];
        int avrg = calculate_average(samples, 1000);
        sprintf(output_freq, "%d", avrg);
        OLED_DrawString(0, 60, B_Color, BLACK, output_freq, 16);
        #endif

        // code here to run FFT
    }
}

int32_t get_sample()
{
    while(!(SPI2->SR & SPI_SR_RXNE));
    int32_t sample = SPI2->DR; 
    return sample >> 8; 
}