#include "stm32f407xx.h"
#include "stdlib.h"
#include <stdio.h>
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;

void init_DRV(); 
void set_direction(u8 dir);