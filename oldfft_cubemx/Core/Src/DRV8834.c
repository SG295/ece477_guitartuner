#include "DRV8834.h"

void init_DRV()
{
    // PA0 - STEP; PA1 - DIR
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    
    GPIOA -> MODER &= ~0xF; // Clear 0 and 1
    GPIOA -> MODER |= 0x5; // Sets 0 and 1 to output
}

void drive_motor(int16_t steps, u8 dir)
{
    if(dir != 0)
    {
        GPIOA -> BSRR = (1 << 1);
    }
    else
    {
        GPIOA -> BSRR = (1 << 1) << 16;
    }

    // GPIOA -> BSRR = dir ? (1 << 1) : ((1 << 1) << 16); 

    for(int16_t i = 0; i < steps; i++)
    {
        GPIOA -> BSRR = (1 << 0);
        nano_wait(2500000); // 1ms delay
        GPIOA -> BSRR = (1 << 0) << 16;
        nano_wait(2500000); // 1ms delay
    }
}