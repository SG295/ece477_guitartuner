#include "stdlib.h"
#include <stdio.h>
#include <stdint.h>
// #include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "BQ27441.h"
#include "DRV8834.h"
#include "oled.h"

typedef enum {
    STANDARD_TUNING_1,
    STANDARD_TUNING_2,
    STANDARD_TUNING_3,
    STANDARD_TUNING_4,
    STANDARD_TUNING_5,
    STANDARD_TUNING_6,
    MAIN_MENU,
    FREE_SPIN,
    DIGITAL_TUNER,
    BATTERY_CHECK,
    BOOT
} state_t; 

extern u8 direct;

extern state_t state;

void init_tim2();
void init_tim3();
void init_tim4();
void init_exti();

void i2s_dma_disable();

void i2s_dma_enable();

/*
EACH MOTOR STEP AFFECTS FREQ by roughly 0.15Hz
*/