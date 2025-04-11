#include "uart.h"

void init_uart()
{
    // Clock to GPIOA and USART2
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
    // Set pins to alt funct mode
    GPIOA -> MODER &= ~0xF0;
    GPIOA -> MODER |= 0xA0;
    // Set AFR to AF7 - USART2_TX, PA2; USART2_RX, PA3 
    GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
    GPIOA -> AFR[0] |= ((0x7 << GPIO_AFRL_AFSEL2_Pos) | (0x7 << GPIO_AFRL_AFSEL3_Pos));
}