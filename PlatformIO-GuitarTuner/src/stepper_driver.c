#include "stepper_driver.h"

void init_DRV()
{
    // PA0 - STEP; PA1 - DIR
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    
    GPIOA -> MODER &= ~0xF; // Clear 0 and 1
    GPIOA -> MODER |= 0x5; // Sets 0 and 1 to output
}

void drive_motor(int16_t steps)
{
    if(steps >= 0)
    {
        GPIOA -> BSRR = (1 << 1);
    }
    else
    {
        GPIOA -> BSRR = (1 << 1) << 16;
    }

    for(u8 i = 0; i < steps; i++)
    {
        GPIOA -> BSRR = (1 << 0);
        nano_wait(1000000); // 1ms delay
        GPIOA -> BSRR = (1 << 0) << 16;
        nano_wait(1000000); // 1ms delay
    }
}

void drive_motor_rpm(int16_t steps, int16_t rpm)
{
    if(steps >= 0)
    {
        GPIOA -> BSRR = (1 << 1);
    }
    else
    {
        GPIOA -> BSRR = (1 << 17); //= (1 << 1) << 16
    }
    steps = abs(steps);

    for(int16_t i = 0; i < steps; i++)
    {
        GPIOA -> BSRR = (1 << 0);
		//200 steps/rot
		//rpm rot/min
		//1/60 min/sec
		//1/1000000000 sec/nanosec
		//2 delays/step

		//1/200 rot/steps
		//1/rpm = min/rot
		//60 sec/min
		//1000000000 nanosec/sec
		//1/2 step/delay

        nano_wait(150000000/rpm); // 150000000/rpm
        GPIOA -> BSRR = (1 << 0) << 16;
        nano_wait(150000000/rpm); // 150ms/rpm delay
    }
}

void step_motor(int16_t dir)
{
    if(dir)
    {
        GPIOA -> BSRR = (1 << 1);
    }
    else
    {
        GPIOA -> BSRR = (1 << 1) << 16;
    }

    GPIOA -> BSRR = (1 << 0);
    nano_wait(1000000); // 1ms delay
    GPIOA -> BSRR = (1 << 0) << 16;
    nano_wait(1000000); // 1ms delay
    
}