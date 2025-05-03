#include "extitimer.h"

void nano_wait(int t); // FROM ECE362 LABS - might need to tweak

// --- GLOBAL VARIABLE DECLARATIONS -------------------------------------------------------------------
u8 buttons = 0; // 8 bit 'register' to hold flags for each buttons at a certain bit
// button 1 ^= 1 << 0;
// button 2 ^= 1 << 1; 
// and so on...

uint8_t held = 0;

state_t state = BOOT; 

u16 charge_buffer;
char data_c[2];
char output_batt[20];
int32_t sample;

// const float standard_tuning[6] =
// {
//     82.41,  // E
//     110,    // A
//     146.83, // D
//     196,    // G
//     246.94, // B
//     329.63  // E
// };

// u8 tuning_i = 0;

char *directions[] = {"0", "1"};
u8 direct = 0; 

// Mic Variables
int magnitude = 0;
float curr_freq = 0;
int int_freq = 0;
float freq_diff = 0; 

// --- CONTROL FLAGS & TESTING FUNCTS --------------------------------------------------------------------
// #define TESTING
#define BATTERY_CONNECTED
// #define MIC_CONNECTED 

// --- CRUCIAL FUNCTION DEFINITIONS -------------------------------------------------------------------
void init_tim2()
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN; //enable clock 

    TIM2 -> PSC = 16800-1;
    TIM2 -> ARR =  2000-1; // 2500-1; // creates .25 second timer for mic readings
    TIM2 -> DIER |= TIM_DIER_UIE; 
    TIM2 -> CR1 |= TIM_CR1_ARPE; 
    NVIC_EnableIRQ(TIM2_IRQn);
    // TIM2 -> CR1 |= TIM_CR1_CEN; //enable
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
}

void init_tim3()
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN; //enable clock 

    TIM3 -> PSC = 16800-1;
    TIM3 -> ARR = 1000-1; 
    TIM3 -> DIER |= TIM_DIER_UIE; 
    TIM3 -> CR1 |= TIM_CR1_ARPE; 
    NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 1);
}

// void TIM3_IRQHandler(void)
// {
//     TIM3 -> SR &= ~TIM_SR_UIF;
//     if((GPIOC->IDR & (1 << 1)) == 0) // held, active low buttons
//     {
//         OLED_DrawString(0, 30, WHITE, BLACK, "*", 12);
//         if(state == FREE_SPIN)
//         {
//             drive_motor(30, direct);
//         }
//         else
//         {
//             #ifdef MIC_CONNECTED
//             // i2s_dma_enable(); // allow mic to read 
//             // if curr_freq is OVER, - value, if UNDER, + value
//             freq_diff = standard_tuning[state] - curr_freq;
//             // Logic for standard tuning states here!
//             if(curr_freq <= 400 && curr_freq >= 30) // add base range
//             {
//                 if(abs(curr_freq) >= 20) // big spin 
//                 {
//                     if(curr_freq > 0) // positive
//                     {
//                         drive_motor(20, 1);
//                     }
//                     else // negative
//                     {
//                         drive_motor(20, 0);
//                     }
//                 }
//                 else if(abs(curr_freq) >= 10)
//                 {
//                     if(curr_freq > 0) // positive
//                     {
//                         drive_motor(10, 1);
//                     }
//                     else // negative
//                     {
//                         drive_motor(10, 0);
//                     }
//                 }
//                 else if(abs(curr_freq) >= 5)
//                 {
//                     if(curr_freq > 0) // positive
//                     {
//                         drive_motor(5, 1);
//                     }
//                     else // negative
//                     {
//                         drive_motor(5, 0);
//                     }
//                 }
//                 else
//                 {
//                     if(curr_freq > 0) // positive
//                     {
//                         drive_motor(2, 1);
//                     }
//                     else // negative
//                     {
//                         drive_motor(2, 0);
//                     }
//                 }
//             }
//             else if (curr_freq > 400)
//             {
//                 // tune way down
//             }
//             else // curr_freq < 30
//             {
//                 // tune way up
//             }
//             #endif
//         }
//     }
//     else 
//     {
//         TIM3 -> CR1 &= ~TIM_CR1_CEN; // disable timer
//         OLED_DrawString(0, 30, WHITE, BLACK, "  ", 12);
//         i2s_dma_disable(); // stop mic from reading to memory 
//     }
// }

void init_tim4()
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN; //enable clock 

    TIM4 -> PSC = 16800-1;
    TIM4 -> ARR = 250-1; 
    TIM4 -> DIER |= TIM_DIER_UIE; 
    TIM4 -> CR1 |= TIM_CR1_ARPE; 
    NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
}

void TIM4_IRQHandler(void)
{
    TIM4 -> SR &= ~TIM_SR_UIF;
    switch(state)
    {
        // OLED_DrawArrow(29, 90, B_Color, 0); // first
        // OLED_DrawArrow(31, 80, B_Color, 0); // second
        // OLED_DrawArrow(34, 69, B_Color, 0); // third
        // OLED_DrawArrow(36, 59, B_Color, 0); // fourth
        // OLED_DrawArrow(39, 48, B_Color, 0); // fifth
        // OLED_DrawArrow(41, 38, B_Color, 0); // sixth
        case STANDARD_TUNING_1: // E
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_2;
                OLED_DrawArrow(29, 90, BLACK, 0);
                OLED_DrawArrow(31, 80, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                GPIOC -> BSRR = (1 << 7) << 16;
                state = MAIN_MENU;
                OLED_WriteMenu();
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_2: // A
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_3;
                OLED_DrawArrow(31, 80, BLACK, 0);
                OLED_DrawArrow(34, 69, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_1;
                OLED_DrawArrow(29, 90, B_Color, 0);
                OLED_DrawArrow(31, 80, BLACK, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_3: // D
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_4;
                OLED_DrawArrow(34, 69, BLACK, 0);
                OLED_DrawArrow(36, 59, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_2;
                OLED_DrawArrow(31, 80, B_Color, 0);
                OLED_DrawArrow(34, 69, BLACK, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_4: // G
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_5;
                OLED_DrawArrow(36, 59, BLACK, 0);
                OLED_DrawArrow(39, 48, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_3;
                OLED_DrawArrow(36, 59, BLACK, 0);
                OLED_DrawArrow(34, 69, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_5: // B
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_6;
                OLED_DrawArrow(39, 48, BLACK, 0);
                OLED_DrawArrow(41, 38, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_4;
                OLED_DrawArrow(39, 48, BLACK, 0);
                OLED_DrawArrow(36, 59, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_6: // E
            if(buttons & (1 << 0)) // right button press
            {
                state = MAIN_MENU;
                OLED_WriteMenu();
                buttons &= ~(1<<0);
                GPIOC -> BSRR = (1 << 7) << 16;
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_5;
                OLED_DrawArrow(41, 38, BLACK, 0);
                OLED_DrawArrow(39, 48, B_Color, 0);
                OLED_DrawString(84, 60, B_Color, BLACK, "       ", 12);
                OLED_DrawString(90, 72, B_Color, BLACK, "    ", 12);
                buttons &= ~(1<<2);
            }
            break;
        case MAIN_MENU:
            if(buttons & (1 << 0)) // right button press
            {   
                state = STANDARD_TUNING_1;
                GPIOC -> BSRR = (1 << 7);
                OLED_DrawString(16, 0, WHITE, BLACK, "             ", 16);
                OLED_DrawString(32, 16, WHITE, BLACK, "      ", 16);
                OLED_DrawString(0, 0, WHITE, BLACK, "Pluck, hold trigger,", 12);
                OLED_DrawString(8, 14, WHITE, BLACK, "and repeat to tune", 12);
                OLED_DrawGuitar();
                OLED_DrawString(0, 111, WHITE, BLACK, "<", 16);
                OLED_DrawString(119, 111, WHITE, BLACK, ">", 16);
                OLED_DrawArrow(29, 90, B_Color, 0);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = BATTERY_CHECK;
                OLED_Clear(BLACK);
                OLED_DrawString(12, 0, B_Color, BLACK, "Battery Stats", 16); 
                #ifdef BATTERY_CONNECTED
                i2c_send_address(BQ27441_COMMAND_FULL_CAPACITY); // full cap
                i2c_read_address(2, data_c);
                charge_buffer = (data_c[1] << 8) | data_c[0];
                sprintf(output_batt, "%d", charge_buffer);
                // Full Charge
                OLED_DrawString(0, 32, WHITE, BLACK, "Full Cap:", 12);
                OLED_DrawString(60, 32, WHITE, BLACK, output_batt, 12);
                OLED_DrawString(90, 32, WHITE, BLACK, "mAh", 12);
                
                i2c_send_address(BQ27441_COMMAND_REM_CAPACITY); // curr cap
                i2c_read_address(2, data_c);
                charge_buffer = (data_c[1] << 8) | data_c[0];
                sprintf(output_batt, "%d", charge_buffer);
                // Current Charge
                OLED_DrawString(0, 48, WHITE, BLACK, "Rem Cap:", 12);
                OLED_DrawString(60, 48, WHITE, BLACK, output_batt, 12);
                OLED_DrawString(90, 48, WHITE, BLACK, "mAh", 12);

                i2c_send_address(BQ27441_COMMAND_SOC); // percent charge
                i2c_read_address(2, data_c);
                charge_buffer = (data_c[1] << 8) | data_c[0];
                sprintf(output_batt, "%d", charge_buffer);
                // Percent Charge
                OLED_DrawString(0, 64, WHITE, BLACK, "Percent:", 12);
                OLED_DrawString(60, 64, WHITE, BLACK, output_batt, 12);
                OLED_DrawString(90, 64, WHITE, BLACK, "%", 12);
                #endif
                buttons &= ~(1<<2);
            }
            else if(buttons & (1 << 4)) // top button press
            {   
                state = FREE_SPIN;
                GPIOC -> BSRR = (1 << 7); 
                OLED_Clear(BLACK);
                OLED_DrawString(26, 0, B_Color, BLACK, "Free Spin", 16);   
                OLED_DrawString(20, 16, WHITE, BLACK, "Press right to", 12);
                OLED_DrawString(12, 28, WHITE, BLACK, "toggle direction", 12); 
                OLED_DrawString(0, 56, A_Color, BLACK, "Direction:", 16); 
                OLED_DrawString(82, 56, A_Color, BLACK, "0", 16);   
                buttons &= ~(1<<4);
                // i2s_dma_enable(); // allow mic readings
            }
            else if(buttons & (1 << 5)) // down button press
            {   
                state = DIGITAL_TUNER;
                OLED_Clear(BLACK);
                OLED_DrawString(12, 0, B_Color, BLACK, "Digital Tuner", 16);
                OLED_DrawString(0, 20, A_Color, BLACK, "Frequency:", 16);
                // int_freq = (int)curr_freq;
                // sprintf(output_batt, "%d", int_freq);
                // OLED_DrawString(80, 20, WHITE, BLACK, output_batt, 16);
                OLED_DrawString(104, 20, WHITE, BLACK, "Hz", 16);
                OLED_DrawString(0, 36+6, WHITE, BLACK, "Standard Tuning:", 12);
                OLED_DrawString(0, 48+6, WHITE, BLACK, "E - 82Hz", 12);
                OLED_DrawString(0, 60+6, WHITE, BLACK, "A - 110Hz", 12);
                OLED_DrawString(0, 72+6, WHITE, BLACK, "D - 147Hz", 12);
                OLED_DrawString(0, 84+6, WHITE, BLACK, "G - 196Hz", 12);
                OLED_DrawString(0, 96+6, WHITE, BLACK, "B - 247Hz", 12);
                OLED_DrawString(0, 108+6, WHITE, BLACK, "e - 330Hz", 12);
                buttons &= ~(1<<5);
                TIM2 -> CNT = 0; 
                TIM2 -> CR1 |= TIM_CR1_CEN;
            }
            break;
        case FREE_SPIN:
            if(buttons & (1 << 5)) // down button press
            {   
                state = MAIN_MENU;
                OLED_WriteMenu();  
                buttons &= ~(1<<5);
                GPIOC -> BSRR = (1 << 7) << 16; 
            }
            else if(buttons & (1 << 0)) // right button press
            {   
                direct ^= 1; // toggle direction 
                OLED_DrawString(82, 56, A_Color, BLACK, directions[direct], 16);
                buttons &= ~(1<<0);
            }
            break;
        case BATTERY_CHECK:
            if(buttons & (1 << 0)) // right button press
            {   
                state = MAIN_MENU;
                OLED_WriteMenu();
                buttons &= ~(1<<0);
            }
            break;
        case DIGITAL_TUNER:
            if(buttons & (1 << 4)) // top button press
            {   
                state = MAIN_MENU;
                OLED_WriteMenu();    
                buttons &= ~(1<<4);
                TIM2 -> CR1 &= ~TIM_CR1_CEN;
            }
            break; 
        default:
            state = MAIN_MENU;
            OLED_WriteMenu();
            buttons = 0; // clear all buttons, shouldn't get here
    }
    if((state == FREE_SPIN) || (state == STANDARD_TUNING_1) || (state == STANDARD_TUNING_2) || (state == STANDARD_TUNING_3) || (state == STANDARD_TUNING_4) || (state == STANDARD_TUNING_5) || (state == STANDARD_TUNING_6))
    {
        EXTI -> IMR |= EXTI_IMR_MR1; // if state allows motor spin, turn on that EXTI
    }
    else
    {
        EXTI -> IMR &= ~EXTI_IMR_MR1; // otherwise, ENSURE it's off
    }
    TIM4 -> CR1 &= ~TIM_CR1_CEN;
    buttons = 0; // clear incase a press with no clear occured at state
}

void init_exti()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN; 

    GPIOC -> MODER &= ~0x3F3F; // Clear PC0-2 and PC4-6 to set as inputs
    GPIOC -> PUPDR &= ~0x3F3F; // Clear
    GPIOC -> PUPDR |= 0x1515; // Set to '01' for pull UP, have external, but extra sure line will be high

    SYSCFG -> EXTICR[0] &= ~0xFFF; // Clear 0-2 inputs
    SYSCFG -> EXTICR[0] |= 0x222; // Set 0-2 for bus C
    SYSCFG -> EXTICR[1] &= ~0xFFF; // Clear 4-6 inputs
    SYSCFG -> EXTICR[1] |= 0x222; // Set 4-6 for bus C

    EXTI -> FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1 | EXTI_FTSR_TR2 | EXTI_FTSR_TR4 | EXTI_FTSR_TR5 | EXTI_FTSR_TR6; // Set to falling edge trigger
    EXTI -> IMR |= EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR6 | EXTI_IMR_MR0 | EXTI_IMR_MR2; // Unmask interrupts in IMR - EXCEPT 1 WHICH IS TRIGGER

    NVIC -> ISER[0] |= (1<<EXTI0_IRQn) | (1<<EXTI1_IRQn) | (1<<EXTI2_IRQn) | (1<<EXTI4_IRQn) | (1<<EXTI9_5_IRQn); // Enable interrupts in vector table
    // Set priority to highest
    NVIC_SetPriority(EXTI0_IRQn, 0);
    NVIC_SetPriority(EXTI1_IRQn, 0);
    NVIC_SetPriority(EXTI2_IRQn, 0);
    NVIC_SetPriority(EXTI4_IRQn, 0);
    NVIC_SetPriority(EXTI9_5_IRQn, 0);
}

void EXTI0_IRQHandler() // Full capacity
{
    
    EXTI -> PR |= EXTI_PR_PR0; // Clear pending bit
    buttons |= 1 << 0;
    // GPIOD -> BSRR = (1 << 12);
    // OLED_Clear(BLACK);
    // OLED_DrawString(0, 63, WHITE, BLACK, R, 12);
    TIM4 -> CR1 |= TIM_CR1_CEN;
}

void EXTI1_IRQHandler() // Remaining capacity 
{
    EXTI -> PR |= EXTI_PR_PR1; // Clear pending bit
    buttons |= 1 << 1;
    // OLED_Clear(BLACK);
    // OLED_DrawString(0, 63, WHITE, BLACK, M, 12);
    // EXTI -> IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR2;
    // TIM3 -> CNT = 0; //reset count to 0 
    // TIM3 -> CR1 |= TIM_CR1_CEN; //enable
    TIM3 -> CR1 |= TIM_CR1_CEN;
}

void EXTI2_IRQHandler() // State of charge 
{  
    EXTI -> PR |= EXTI_PR_PR2; // Clear pending bit
    buttons |= 1 << 2;
    // GPIOD -> BSRR = (1 << 14);
    // OLED_Clear(BLACK);
    // OLED_DrawString(0, 63, WHITE, BLACK, L, 12);
    TIM4 -> CR1 |= TIM_CR1_CEN;
}

void EXTI4_IRQHandler()
{
    EXTI -> PR |= EXTI_PR_PR4; // Clear pending bit
    buttons |= 1 << 4;
    // OLED_Clear(BLACK);
    // OLED_DrawString(0, 63, WHITE, BLACK, Tp, 12);
    // EXTI -> IMR &= ~(EXTI_IMR_MR4); 
    TIM4 -> CR1 |= TIM_CR1_CEN;
}

void EXTI9_5_IRQHandler()
{
    if(EXTI -> PR & EXTI_PR_PR5)
    {
        EXTI -> PR |= EXTI_PR_PR5; // Clear pending bit for 5
        buttons |= 1 << 5;
        // OLED_Clear(BLACK);
        // OLED_DrawString(0, 63, WHITE, BLACK, Bt, 12);
        TIM4 -> CR1 |= TIM_CR1_CEN;
    }
    else if(EXTI -> PR & EXTI_PR_PR6)
    {
        EXTI -> PR |= EXTI_PR_PR6; // Clear pending bit for 6
        buttons |= 1 << 6;
        // OLED_Clear(BLACK);
        // OLED_DrawString(0, 63, WHITE, BLACK, L, 12);
        TIM4 -> CR1 |= TIM_CR1_CEN;
    }
}

void i2s_dma_disable() // might not need this but could be good to have...we'll see :)
{
    DMA1_Stream3 -> CR &= ~DMA_SxCR_EN;
}

void i2s_dma_enable() // might not need this but could be good to have...we'll see :)
{
    DMA1_Stream3 -> CR |= DMA_SxCR_EN;
}