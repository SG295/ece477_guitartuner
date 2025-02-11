#include "stm32f407xx.h"
#include "oled.h"

#ifdef TESTING
void initb()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB -> MODER &= ~0x30000; // Clear PB8
    GPIOB -> MODER |= 0x10000; // PB8 - Output

    GPIOB -> BSRR = (1 << 8); // Set PB8 High 
}

void initd()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN; 

    GPIOD -> MODER &= ~0x3F000000; // clear PD12-14
    GPIOD -> MODER |= 0x55000000; // set output PD12-14

    // GPIOD -> BSRR = (1 << 12) | (1 << 13) | (1 << 14); // set high
}

void init_buttons()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN; 

    // Turn on PD7, 5, 3 and as INPUTS for buttons
    GPIOD -> MODER &= ~0xCCC0; // clear to ensure input
    GPIOD -> PUPDR &= ~0xCCCC; 
    GPIOD -> PUPDR |= 0x8888; // Pull down 

}
#endif

void init_spi1()
{
    // CS - PA4, SCK - PA5, MISO - PA6, MOSI - PA7, DC - PA8, RST - PA9
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA -> MODER &= ~0xFFF00; // clear 4-9
    GPIOA -> MODER |= 0x5A900; // set 5-7 to AF, and then 4, 8-9 to output
    GPIOA -> AFR[0] &= ~0xFFF00000; // clear AF 5-7
    GPIOA -> AFR[0] |= 0x55500000; // set AF 5-7 to 5

    SPI1 -> CR1 &= ~SPI_CR1_SPE; // Disable channel before config
    SPI1 -> CR1 &= ~SPI_CR1_DFF; // Ensure data frame is 8 bit
    SPI1 -> CR1 &= ~(SPI_CR1_BR); 
    SPI1 -> CR1 |= SPI_CR1_BR_2 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1 -> CR1 |= SPI_CR1_SPE;
}

int main(void)
{
    init_spi1(); 

    OLED_Setup(); 

    OLED_Clear(BLUE);

}