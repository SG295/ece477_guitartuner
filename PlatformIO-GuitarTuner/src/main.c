#include "stm32f407xx.h"
#include "oled.h"
#include "BQ27441.h"

#define TESTING
#ifdef TESTING

void nano_wait(int t); // FROM ECE362 LABS

const char *R = "Right Button Pressed";
const char *M = "Middle Button Pressed";
const char *L = "Left Button Pressed";
const uint8_t reads_c = 2;

const uint8_t arrow_left_pos = 22;
const uint8_t arrow_right_pos = 92;

char data_c[2];
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
    EXTI -> IMR |= EXTI_IMR_MR1 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR6; // Unmask interrupts in IMR

    NVIC -> ISER[0] |= (1<<EXTI0_IRQn) | (1<<EXTI1_IRQn) | (1<<EXTI2_IRQn) | (1<<EXTI4_IRQn) | (1<<EXTI9_5_IRQn); // Enable interrupts in vector table
}

void EXTI0_IRQHandler()
{
    
    EXTI -> PR |= EXTI_PR_PR0; // Clear pending bit
    GPIOD -> BSRR = (1 << 12);
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, R, 12);
    GPIOD -> BSRR = (1 << 12) << 16;
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
    EXTI -> PR |= EXTI_PR_PR2; // Clear pending bit
    GPIOD -> BSRR = (1 << 14);
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, L, 12);
    GPIOD -> BSRR = (1 << 14) << 16;
    i2c_send_address(BQ27441_COMMAND_VOLTAGE);
    i2c_read_address(reads_c,data_c);
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
        *data = I2C2->DR;
    }
}

int main(void)
{
    init_spi1(); 
    initd();
    init_exti();
    init_i2c_BQ27441(); 

    OLED_Setup(); 
    OLED_Clear(BLACK); 

    const char *S = "ECE477 Guitar Tuner";

    OLED_DrawString(0, 0, WHITE, BLACK, S, 12);

    const char *T = "Press middle button";

    OLED_DrawString(0, 52, WHITE, BLACK, T, 12);

    const char *H = "to begin standard";

    OLED_DrawString(0, 64, WHITE, BLACK, H, 12);

    const char *X = "tuning.";

    OLED_DrawString(0, 76, WHITE, BLACK, X, 12);

    OLED_DrawGuitar();

    OLED_DrawArrow(arrow_left_pos, 57, B_Color, 0);

    OLED_DrawArrow(arrow_right_pos, 57+(17*2), B_Color, 1);

    for(;;)
    {

    }
}
// PB6 (SCL) and PB7 (SDA) for Battery Management