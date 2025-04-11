#include "mic.h"

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
    SPI2 -> I2SPR |= 0x6; // <-- CHECK THIS, should get 38.4MHz/(6*2) = 3.2MHz/64 = 50kHz sample rate

    SPI2 -> I2SCFGR |= SPI_I2SCFGR_I2SE; // enable channel
}

int32_t get_sample()
{
    while(!(SPI2->SR & SPI_SR_RXNE));
    int32_t sample = SPI2->DR; 
    return sample >> 8; 
}