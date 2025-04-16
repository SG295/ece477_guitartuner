#include "stm32f407xx.h"
#include "oled.h"
#include "BQ27441.h"
#include "DRV8834.h"
#include "mic.h"

#define TESTING
#define BATT_TESTING
#ifdef TESTING

void nano_wait(int t); // FROM ECE362 LABS

// const char *R = "Right Button Pressed";
// const char *M = "Middle Button Pressed";
// const char *L = "Left Button Pressed";
const char *R = "Right"; // "(mAh): "; 
const char *M = "Middle"; // (mAh): ";
const char *L = "Left"; // (Perc): ";
const char *Tp = "Top"; // (Perc): ";
const char *Bt = "Bottom"; // (Perc): ";
const char *p = "HELD";

#ifdef BATT_TESTING

const u8 batt_comms[3] = { // for battery prototyping
    BQ27441_COMMAND_FULL_CAP_FIL,
    BQ27441_COMMAND_REM_CAPACITY,
    BQ27441_COMMAND_SOC
};

char *directions[] = {"0", "1"};
u8 direct = 0; 

u8 index_batt = 0; 

#endif

u8 buttons = 0; // 8 bit 'register' to hold flags for each buttons at a certain bit
/*
button 1 ^= 1 << 0;
button 2 ^= 1 << 1; 
and so on... 
*/

uint8_t held = 0;

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

state_t state = BOOT; 

const uint8_t arrow_left_pos = 22;
const uint8_t arrow_right_pos = 92;

u16 data_buffer;
char data_c[2];
char output_batt[20];
int32_t sample;
void i2c_send_address(uint8_t address);
void i2c_read_address(uint8_t reads, char *data);

void write_menu();

const float standard_tuning[6] =
{
    82.41,  // E
    110,    // A
    146.83, // D
    196,    // G
    246.94, // B
    329.63  // E
};

u8 tuning_i = 0;

void initb()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB -> MODER &= ~0x30000; // Clear PB8
    GPIOB -> MODER |= 0x10000; // PB8 - Output

    GPIOB -> BSRR = (1 << 8); // Set PB8 High 
}

void initd()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN; 

    GPIOD -> MODER &= ~0x3F000000; // clear PD12-14
    GPIOD -> MODER |= 0x55000000; // set output PD12-14

    // GPIOD -> BSRR = (1 << 12) | (1 << 13) | (1 << 14); // set high
}

void togglexn(GPIO_TypeDef *port, int pos)
{
    port -> ODR ^= (1 << pos);
}
#endif

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
}

void init_tim2()
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN; //enable clock 

    TIM2 -> PSC = 16800-1;
    TIM2 -> ARR =  2000-1; // 2500-1; // creates .25 second timer for mic readings
    TIM2 -> DIER |= TIM_DIER_UIE; 
    TIM2 -> CR1 |= TIM_CR1_ARPE; 
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2 -> CR1 |= TIM_CR1_CEN; //enable
}

void TIM2_IRQHandler(void)
{
    TIM2 -> SR &= ~TIM_SR_UIF;

    if(index_batt==0) {
        OLED_DrawString(0, 63, WHITE, BLACK, R, 12);
        OLED_DrawString(0, 80, WHITE, BLACK, "        ", 12);
        OLED_DrawString(28, 80, WHITE, BLACK, "mAh", 12);
    }
    else if(index_batt==1) {
        OLED_DrawString(0, 63, WHITE, BLACK, M, 12);
        OLED_DrawString(0, 80, WHITE, BLACK, "        ", 12);
        OLED_DrawString(28, 80, WHITE, BLACK, "mAh", 12);
    }
    else {
        OLED_DrawString(0, 63, WHITE, BLACK, L, 12);
        OLED_DrawString(0, 80, WHITE, BLACK, "        ", 12);
        OLED_DrawString(24, 80, WHITE, BLACK, "%", 12);
    }
    i2c_send_address(batt_comms[index_batt]);// BQ27441_COMMAND_REM_CAPACITY);
    i2c_read_address(2, data_c);
    data_buffer = (data_c[1] << 8) | data_c[0];
    sprintf(output_batt, "%d", data_buffer);
    OLED_DrawString(0, 80, WHITE, BLACK, output_batt, 12);
    // sample = get_sample(); // for mix reading
    index_batt+=1;
    if(index_batt > 2)
    {
        index_batt = 0; 
    }
}

void init_tim3()
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN; //enable clock 

    TIM3 -> PSC = 16800-1;
    TIM3 -> ARR = 100-1; 
    TIM3 -> DIER |= TIM_DIER_UIE; 
    TIM3 -> CR1 |= TIM_CR1_ARPE; 
    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    TIM3 -> SR &= ~TIM_SR_UIF;

    if((GPIOC->IDR & (1 << 1)) == 0) // held, active low buttons
    {
        held = 1; 
        OLED_DrawString(0, 30, WHITE, BLACK, "*", 12);
        if(state == FREE_SPIN)
        {
            drive_motor(200);
        }
        else
        {
            // Logic for standard tuning states here!
        }
    }
    else 
    {
        TIM3 -> CR1 &= ~TIM_CR1_CEN; // disable timer
        held = 0; // reset held
        OLED_DrawString(0, 30, WHITE, BLACK, "  ", 12);
    }
}

void init_tim4()
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN; //enable clock 

    TIM4 -> PSC = 16800-1;
    TIM4 -> ARR = 250-1; 
    TIM4 -> DIER |= TIM_DIER_UIE; 
    TIM4 -> CR1 |= TIM_CR1_ARPE; 
    NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void)
{
    TIM4 -> SR &= ~TIM_SR_UIF;
    switch(state)
    {
        case STANDARD_TUNING_1: // E
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_2;
                OLED_DrawArrow(arrow_left_pos, 57, BLACK, 0);
                OLED_DrawArrow(arrow_left_pos, 57+(17), B_Color, 0);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = MAIN_MENU;
                write_menu();
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_2: // A
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_3;
                OLED_DrawArrow(arrow_left_pos, 57+(17), BLACK, 0);
                OLED_DrawArrow(arrow_left_pos, 57+(17*2), B_Color, 0);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_1;
                OLED_DrawArrow(arrow_left_pos, 57+(17), BLACK, 0);
                OLED_DrawArrow(arrow_left_pos, 57, B_Color, 0);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_3: // D
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_4;
                OLED_DrawArrow(arrow_right_pos, 57, B_Color, 1);
                OLED_DrawArrow(arrow_left_pos, 57+(17*2), BLACK, 0);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_2;
                OLED_DrawArrow(arrow_left_pos, 57+(17*2), BLACK, 0);
                OLED_DrawArrow(arrow_left_pos, 57+(17), B_Color, 0);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_4: // G
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_5;
                OLED_DrawArrow(arrow_right_pos, 57+(17), B_Color, 1);
                OLED_DrawArrow(arrow_right_pos, 57, BLACK, 1);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_3;
                OLED_DrawArrow(arrow_right_pos, 57, BLACK, 1);
                OLED_DrawArrow(arrow_left_pos, 57+(17*2), B_Color, 0);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_5: // B
            if(buttons & (1 << 0)) // right button press
            {
                state = STANDARD_TUNING_6;
                OLED_DrawArrow(arrow_right_pos, 57+(17*2), B_Color, 1);
                OLED_DrawArrow(arrow_right_pos, 57+(17), BLACK, 1);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_4;
                OLED_DrawArrow(arrow_right_pos, 57, B_Color, 1);
                OLED_DrawArrow(arrow_right_pos, 57+(17), BLACK, 1);
                buttons &= ~(1<<2);
            }
            break;
        case STANDARD_TUNING_6: // E
            if(buttons & (1 << 0)) // right button press
            {
                state = MAIN_MENU;
                write_menu();
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = STANDARD_TUNING_5;
                OLED_DrawArrow(arrow_right_pos, 57+(17), B_Color, 1);
                OLED_DrawArrow(arrow_right_pos, 57+(17*2), BLACK, 1);
                buttons &= ~(1<<2);
            }
            break;
        case MAIN_MENU:
            if(buttons & (1 << 0)) // right button press
            {   
                state = STANDARD_TUNING_1;
                OLED_DrawString(16, 0, WHITE, BLACK, "             ", 16);
                OLED_DrawString(12, 0, WHITE, BLACK, "Hold Trigger  ", 16);
                OLED_DrawString(0, 16, WHITE, BLACK, "to allow tuning", 16);
                OLED_DrawGuitar();
                OLED_DrawString(0, 111, WHITE, BLACK, "<", 16);
                OLED_DrawString(119, 111, WHITE, BLACK, ">", 16);
                OLED_DrawArrow(arrow_left_pos, 57, B_Color, 0);
                buttons &= ~(1<<0);
            }
            else if(buttons & (1 << 2)) // left button press
            {   
                state = BATTERY_CHECK;
                OLED_Clear(BLACK);
                OLED_DrawString(0, 63, WHITE, BLACK, "Battery Chk", 12);    
                buttons &= ~(1<<2);
            }
            else if(buttons & (1 << 4)) // top button press
            {   
                state = FREE_SPIN;
                OLED_Clear(BLACK);
                OLED_DrawString(26, 0, B_Color, BLACK, "Free Spin", 16);   
                OLED_DrawString(20, 16, WHITE, BLACK, "Press right to", 12);
                OLED_DrawString(12, 28, WHITE, BLACK, "toggle direction", 12); 
                OLED_DrawString(0, 56, A_Color, BLACK, "Direction:", 16); 
                OLED_DrawString(82, 56, A_Color, BLACK, "0", 16);   
                buttons &= ~(1<<4);
            }
            else if(buttons & (1 << 5)) // down button press
            {   
                state = DIGITAL_TUNER;
                OLED_Clear(BLACK);
                OLED_DrawString(0, 63, WHITE, BLACK, "Digital Tuner", 12);    
                buttons &= ~(1<<5);
            }
            break;
        case FREE_SPIN:
            if(buttons & (1 << 5)) // down button press
            {   
                state = MAIN_MENU;
                write_menu();  
                buttons &= ~(1<<5);
            }
            else if(buttons & (1 << 0)) // right button press
            {   
                direct ^= 1; // toggle direction 
                OLED_DrawString(82, 56, A_Color, BLACK, directions[direct], 16);  
            }
            break;
        case BATTERY_CHECK:
            if(buttons & (1 << 0)) // right button press
            {   
                state = MAIN_MENU;
                write_menu();
                buttons &= ~(1<<0);
            }
            break;
        case DIGITAL_TUNER:
            if(buttons & (1 << 4)) // top button press
            {   
                state = MAIN_MENU;
                write_menu();    
                buttons &= ~(1<<4);
            }
            break; 
        default:
            state = MAIN_MENU;
            write_menu();
            buttons = 0; // clear all buttons, shouldn't get here
    }
    // OLED_Clear(BLACK);
    // OLED_DrawString(0, 63, WHITE, BLACK, Tp, 12);
    if((state == FREE_SPIN) || (state == STANDARD_TUNING_1) || (state == STANDARD_TUNING_2) || (state == STANDARD_TUNING_3) || (state == STANDARD_TUNING_4) || (state == STANDARD_TUNING_5) || (state == STANDARD_TUNING_6))
    {
        EXTI -> IMR |= EXTI_IMR_MR1; // if state allows motor spin, turn on that EXTI
    }
    else
    {
        EXTI -> IMR &= ~EXTI_IMR_MR1; // otherwise, ENSURE it's off
    }
    TIM4 -> CR1 &= ~TIM_CR1_CEN;
    // EXTI -> IMR |= (EXTI_IMR_MR4);
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

void init_spi1()
{
    // CS - PA4, SCK - PA5, MISO - PA6, MOSI - PA7, DC - PA8, RST - PA9
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
    //RCC -> CFGR |= RCC_CFGR_PPRE2_DIV2; // Div 168Mhz by 2 = 84MHz
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA -> MODER &= ~0xFFF00; // clear 4-9
    GPIOA -> MODER |= 0x5A900; // set 5-7 to AF, and then 4, 8-9 to output
    GPIOA -> AFR[0] &= ~0xFFF00000; // clear AF 5-7
    GPIOA -> AFR[0] |= 0x55500000; // set AF 5-7 to 5

    SPI1 -> CR1 &= ~SPI_CR1_SPE; // Disable channel before config
    SPI1 -> CR1 &= ~SPI_CR1_DFF; // Ensure data frame is 8 bit
    SPI1 -> CR1 &= ~(SPI_CR1_BR); 
    SPI1 -> CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1 -> CR1 |= SPI_CR1_SPE;
}

void init_i2c_BQ27441() // I2C1 and AF4
{
    RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC -> CFGR |= RCC_CFGR_PPRE1_DIV4; // Div 168MHz by 4 = 42MHz
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB -> MODER &= ~0xF000; // Clear 6 and 7
    GPIOB -> MODER |= 0xA000; // 6 and 7 to AF mode
    GPIOB -> OTYPER |= 0xC0; // Open drain for 6 and 7
    GPIOB -> OSPEEDR |= 0xF000; // High speed for 6 and 7 <- CHECK THIS
    GPIOB -> PUPDR |= 0x5000; // PULL UP
    GPIOB -> AFR[0] &= ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7); // Clear AFR
    GPIOB -> AFR[0] |= (GPIO_AFRL_AFRL6_2 | GPIO_AFRL_AFRL7_2); // Set AFR to AF4 for I2C1

    I2C1 -> CR1 |= I2C_CR1_SWRST; // Reset I2C
    I2C1 -> CR1 &= ~I2C_CR1_SWRST;
    I2C1 -> CR1 &= ~I2C_CR1_PE; // Turn off channel 
    // UNSURE \/
    I2C1 -> CR2 |= (0x10<<0); // set to be 16MHz
    I2C1 -> CCR = (0x14<<0); // T_low/high = 1.25us, T_PCLK1 = 62.5ns, 1.25us/62.5ns=20 or 0x14
    I2C1 -> TRISE = 0x5; // (300ns / 62.5ns) + 1 = 5.8, use just int part so 5
    I2C1 -> CR1 |= I2C_CR1_PE; // Enable channel
    
}

uint16_t reg;

void i2c_send_address(uint8_t address)
{
    I2C1 -> CR1 |= I2C_CR1_ACK | I2C_CR1_START; // send start and ACK
    while(!(I2C1->SR1 & I2C_SR1_SB)); // wait until SB goes high to ensure start condition generated
    I2C1 -> DR = BQ72441_I2C_WRITE; // send address <--
    while(!(I2C1->SR1 & I2C_SR1_ADDR)); // wait for address to be sent
    reg = I2C1->SR1 | I2C1->SR2; // clear both SRs by setting them to arbitrary variable
    while(!(I2C1->SR1 & I2C_SR1_TXE)); // ensure DR is empty
    I2C1->DR = address; 
    while(!(I2C1->SR1 & I2C_SR1_BTF)); // ensure data is transferred (byte transfer finished)
}

void i2c_read_address(uint8_t reads, char *data) // should go right after a send address!!!
{
    I2C1 -> CR1 |= I2C_CR1_START; // send start
    while(!(I2C1->SR1 & I2C_SR1_SB)); // wait until SB goes high to ensure start condition generated
    I2C1 -> DR = BQ72441_I2C_READ; // send address <-- 
    while(!(I2C1->SR1 & I2C_SR1_ADDR)); // wait for address to be sent
    reg = I2C1->SR1 | I2C1->SR2; // clear both SRs by setting them to arbitrary variable
    
    for(uint8_t i = 0; i < reads; i++, data++)
    {
        if(i + 1 == reads)
        {
            I2C1 -> CR1 &= ~I2C_CR1_ACK;
            I2C1 -> CR1 |= I2C_CR1_STOP;
        }
        while(!(I2C1->SR1 & I2C_SR1_RXNE));
        *data = I2C1->DR;
    }
}

void write_menu()
{
    OLED_Clear(BLACK);
    OLED_DrawString(12, 0, WHITE, BLACK, "ECE477 Guitar", 16);
    OLED_DrawString(44, 16, WHITE, BLACK, "Tuner", 16);
    OLED_DrawString(20, 115, B_Color, BLACK, "\\/ Digi Tuner", 12);
    OLED_DrawString(20, 95, B_Color, BLACK, "< Batt Stats", 12);
    OLED_DrawString(20, 75, B_Color, BLACK, "> Auto Tune", 12);
    OLED_DrawString(20, 55, B_Color, BLACK, "/\\ Free Spin", 12);
    /*
            ECE GUITAR
               TUNER
            /\ Free Spin
    
    < Batt Stats      Auto Tune >
    
            \/ Digi Tuner
    */

}

int main(void)
{
    init_spi1(); 
    init_exti();
    init_tim4(); 
    init_i2c_BQ27441(); 
    init_DRV();
    init_gpio_mic();
    clock_enable(); 
    init_i2s_mic();
    init_tim3();

    OLED_Setup(); 
    OLED_Clear(BLACK); 

    init_tim3();
    init_tim4(); 

    // OLED_DrawGuitar();

    // OLED_DrawArrow(arrow_left_pos, 57, B_Color, 0);

    // OLED_DrawArrow(arrow_right_pos, 57+(17*2), B_Color, 1);

    // Check battery charge and prevent boot if too low:
    // i2c_send_address(BQ27441_COMMAND_SOC); // get percent charge
    // i2c_read_address(2, data_c);
    // data_buffer = (data_c[1] << 8) | data_c[0];
    data_buffer = 10; // hardcode for testing
    if(data_buffer <= 3) // if percent charge is less than 3
    {
        OLED_DrawString(32, 50, C_Color, BLACK, "LOW BATT", 16);
        OLED_DrawString(34, 66, WHITE, BLACK, "Power off", 12);
        OLED_DrawString(34, 78, WHITE, BLACK, "and charge", 12);
        while(1); // hold here until power off
    }
    else
    {
        state = MAIN_MENU; 
        write_menu();
    }

    for(;;)
    {
        // wait for interrupts \(^-^)/ 
    }
}
// Notes: 
//  I2C: PB6 (SCL) and PB7 (SDA) for Battery Management
//  SPI: CS - PA4, SCK - PA5, MISO - PA6, MOSI - PA7, DC - PA8, RST - PA9