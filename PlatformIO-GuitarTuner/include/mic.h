#include "stm32f407xx.h"
#include "stdlib.h"
#include "arm_math.h" // CMSIS DSP
#include "arm_const_structs.h" // CMSIS DSP
#include <stdio.h>
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;

void nano_wait(int t); // FROM ECE362 LABS

void clock_enable();
void init_gpio_mic();
void init_i2s_mic();
void i2s_dma(); 
void i2s_dma_enable();
void i2s_dma_disable();
int32_t process_sample(uint32_t raw_data); 

void uart_init(void);
void uart_send_char(char c);
void uart_send_string(const char *s);