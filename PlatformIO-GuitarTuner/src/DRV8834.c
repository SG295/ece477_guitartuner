#include "DRV8834.h"

void init_DRV()
{
    // PA0 - STEP; PA1 - DIR
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    
    GPIOA -> MODER &= ~0xF; // Clear 0 and 1
    GPIOA -> MODER |= 0x4; // Set 1 to output
}