// #include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

#include "stdlib.h"
#include <stdio.h>
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;

void nano_wait(int t); // FROM ECE362 LABS

void init_DRV(); 
void drive_motor(int16_t steps, u8 dir);