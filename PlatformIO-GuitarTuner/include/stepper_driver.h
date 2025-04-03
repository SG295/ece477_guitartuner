#include "stm32f407xx.h"
#include "stdlib.h"
#include <stdio.h>
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;

void nano_wait(int t); // FROM ECE362 LABS

void init_DRV(); 
void set_direction(u8 dir);
void drive_motor(int16_t steps);
void drive_motor_rpm(int16_t steps, int16_t rpm);
void step_motor(int16_t dir);