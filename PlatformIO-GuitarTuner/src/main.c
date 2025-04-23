#include "stm32f407xx.h"
#include "oled.h"
#include "BQ27441.h"
#include "stepper_driver.h"

#define TESTING
#ifdef TESTING

void nano_wait(int t); // FROM ECE362 LABS

// --- GLOBAL VARIABLE DECLARATIONS -------------------------------------------------------------------
u8 buttons = 0; // 8 bit 'register' to hold flags for each buttons at a certain bit
// button 1 ^= 1 << 0;
// button 2 ^= 1 << 1; 
// and so on...

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


const char *R = "Right Button Pressed";
const char *M = "Middle Button Pressed";
const char *L = "Left Button Pressed";

const uint8_t arrow_left_pos = 22;
const uint8_t arrow_right_pos = 92;

u16 charge_buffer;
char data_c[2];
char output_batt[20];
void i2c_send_address(uint8_t address);
void i2c_read_address(uint8_t reads, char *data);

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

char *directions[] = {"0", "1"};
u8 direct = 0; 

// Mic Variables
int magnitude = 0;
float curr_freq = 0;
int int_freq = 0;
float freq_diff = 0; 

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

void init_buttons()
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 

    // Turn on PD7, 5, 3 and as INPUTS for buttons
    GPIOB -> MODER &= ~0xCCC00000; // clear to ensure input
    // GPIOB -> PUPDR &= ~0xCCC00000; 
    // GPIOB -> PUPDR |= 0x88800000; // Pull down 

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
    EXTI -> IMR |= EXTI_IMR_MR1 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR6 | EXTI_IMR_MR0 | EXTI_IMR_MR2; // Unmask interrupts in IMR

    NVIC -> ISER[0] |= (1<<EXTI0_IRQn) | (1<<EXTI1_IRQn) | (1<<EXTI2_IRQn) | (1<<EXTI4_IRQn) | (1<<EXTI9_5_IRQn); // Enable interrupts in vector table
}

void EXTI0_IRQHandler()
{
    
    EXTI -> PR |= EXTI_PR_PR0; // Clear pending bit
    GPIOD -> BSRR = (1 << 12);
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, R, 12);
    GPIOD -> BSRR = (1 << 12) << 16;
    drive_motor(100);
}

void EXTI1_IRQHandler()
{
    EXTI -> PR |= EXTI_PR_PR1; // Clear pending bit
    GPIOD -> BSRR = (1 << 13);
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, M, 12);
    EXTI -> IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR2;
    GPIOD -> BSRR = (1 << 13) << 16;
}

void EXTI2_IRQHandler()
{  
    data_buffer = 0;
    EXTI -> PR |= EXTI_PR_PR2; // Clear pending bit
    GPIOD -> BSRR = (1 << 14);
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, L, 12);
    GPIOD -> BSRR = (1 << 14) << 16;
    // i2c_send_address(BQ27441_COMMAND_SOC);// BQ27441_COMMAND_REM_CAPACITY);
    // i2c_read_address(2, data_c);
    // data_buffer = (data_c[1] << 8) | data_c[0];
    // sprintf(output_batt, "%d", data_buffer);
    // OLED_DrawString(0, 80, WHITE, BLACK, output_batt, 12);
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
        OLED_DrawString(0, 30, WHITE, BLACK, "*", 12);
        if(state == FREE_SPIN)
        {
            drive_motor(30, direct);
        }
        else
        {
            #ifdef MIC_CONNECTED
            // if curr_freq is OVER, - value, if UNDER, + value
            freq_diff = standard_tuning[state] - curr_freq;
            // Logic for standard tuning states here!
            if(curr_freq <= 400 && curr_freq >= 30) // add base range
            {
                if(abs(curr_freq) >= 20) // big spin 
                {
                    if(curr_freq > 0) // positive
                    {
                        drive_motor(20, 1);
                    }
                    else // negative
                    {
                        drive_motor(20, 0);
                    }
                }
                else if(abs(curr_freq) >= 10)
                {
                    if(curr_freq > 0) // positive
                    {
                        drive_motor(10, 1);
                    }
                    else // negative
                    {
                        drive_motor(10, 0);
                    }
                }
                else if(abs(curr_freq) >= 5)
                {
                    if(curr_freq > 0) // positive
                    {
                        drive_motor(5, 1);
                    }
                    else // negative
                    {
                        drive_motor(5, 0);
                    }
                }
                else
                {
                    if(curr_freq > 0) // positive
                    {
                        drive_motor(2, 1);
                    }
                    else // negative
                    {
                        drive_motor(2, 0);
                    }
                }
            }
            else if (curr_freq > 400)
            {
                // tune way down
            }
            else // curr_freq < 30
            {
                // tune way up
            }
            #endif
        }
    }
    else 
    {
        TIM3 -> CR1 &= ~TIM_CR1_CEN; // disable timer
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
                OLED_DrawString(12, 0, B_Color, BLACK, "Digital Tuner", 16);
                OLED_DrawString(0, 20, A_Color, BLACK, "Frequency:", 16);
                // int_freq = (int)curr_freq;
                sprintf(output_batt, "%d", int_freq);
                OLED_DrawString(80, 20, WHITE, BLACK, output_batt, 16);
                OLED_DrawString(104, 20, WHITE, BLACK, "Hz", 16);
                OLED_DrawString(0, 36+6, WHITE, BLACK, "Standard Tuning:", 12);
                OLED_DrawString(0, 48+6, WHITE, BLACK, "e - 330Hz", 12); // "E - 82Hz", 12);
                OLED_DrawString(0, 60+6, WHITE, BLACK, "B - 247Hz", 12); // "A - 110Hz", 12);
                OLED_DrawString(0, 72+6, WHITE, BLACK, "G - 196Hz", 12); // "D - 147Hz", 12);
                OLED_DrawString(0, 84+6, WHITE, BLACK, "D - 147Hz", 12); // "G - 196Hz", 12);
                OLED_DrawString(0, 96+6, WHITE, BLACK, "A - 110Hz", 12); // "B - 247Hz", 12);
                OLED_DrawString(0, 108+6, WHITE, BLACK, "E - 82Hz", 12); // "e - 330Hz", 12);
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
                buttons &= ~(1<<0);
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
void EXTI4_IRQHandler()
{
    EXTI -> PR |= EXTI_PR_PR4; // Clear pending bit
}

void EXTI9_5_IRQHandler()
{
    if(EXTI -> PR & EXTI_PR_PR5_Pos)
    {
        EXTI -> PR |= EXTI_PR_PR5; // Clear pending bit for 5
    }
    else if(EXTI -> PR & EXTI_PR_PR6_Pos)
    {
        EXTI -> PR |= EXTI_PR_PR6; // Clear pending bit for 6
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

int main(void)
{
    init_spi1(); 
    initd();
    init_exti();
    init_i2c_BQ27441(); 
    init_DRV();

    OLED_Setup(); 
    OLED_Clear(BLACK); 

    const char *S = "ECE477 Guitar Tuner";

    OLED_DrawString(0, 0, WHITE, BLACK, S, 12);

    // const char *T = "Press middle button";

    // OLED_DrawString(0, 52, WHITE, BLACK, T, 12);

    // const char *H = "to begin standard";

    // OLED_DrawString(0, 64, WHITE, BLACK, H, 12);

    // const char *X = "tuning.";

    // OLED_DrawString(0, 76, WHITE, BLACK, X, 12);

    // OLED_DrawGuitar();

    // OLED_DrawArrow(arrow_left_pos, 57, B_Color, 0);

    // OLED_DrawArrow(arrow_right_pos, 57+(17*2), B_Color, 1);

    //drive_motor(100); 
    drive_motor_rpm(400, 120);
    //for(int16_t i=3; i < 6; i++){
        //drive_motor_rpm(400, i*20);
    //}
    drive_motor_rpm(-200, 60);
    //drive_motor_rpm(100, 30);
    //drive_motor_rpm(-50, 10);
    //drive_motor_rpm(50, 10);
    //drive_motor_rpm(-20, 10);
    //drive_motor_rpm(20, 10);
    GPIOB->MODER &= ~(3UL << (4 * 2));       // Clear mode bits
    GPIOB->MODER |=  (1UL << (4 * 2));       // Set to output mode (01)

    // Set output type to push-pull (default, optional)
    GPIOB->OTYPER &= ~(1UL << 4);

    // Set output speed to medium (optional)
    GPIOB->OSPEEDR |= (1UL << (4 * 2));      

    // Set no pull-up, pull-down
    GPIOB->PUPDR &= ~(3UL << (4 * 2)); 
    

    for(;;)
    {
        //GPIOA -> BSRR = (1 << 1);
        GPIOB->ODR ^= (1UL << 4);    // Toggle PB4
        nano_wait(1000000000);

        drive_motor_rpm(-200, 60);
        
    }
}
// PB6 (SCL) and PB7 (SDA) for Battery Management