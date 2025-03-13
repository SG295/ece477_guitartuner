#include "stm32f407xx.h"
#include "oled.h"

#define TESTING
#ifdef TESTING

void nano_wait(int t); // FROM ECE362 LABS

const char *R = "Right Button Pressed";
const char *M = "Middle Button Pressed";
const char *L = "Left Button Pressed";

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

    GPIOD -> BSRR = (1 << 12) | (1 << 13) | (1 << 14); // set high
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
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, R, 12);
}

void EXTI1_IRQHandler()
{
    EXTI -> PR |= EXTI_PR_PR1; // Clear pending bit
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, M, 12);
    EXTI -> IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR2;
}

void EXTI2_IRQHandler()
{
    EXTI -> PR |= EXTI_PR_PR2; // Clear pending bit
    OLED_Clear(BLACK);
    OLED_DrawString(0, 63, WHITE, BLACK, L, 12);
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

int main(void)
{
    init_spi1(); 
    // init_buttons();
    // initd();
    init_exti();

    OLED_Setup(); 

    // OLED_Clear(C_Color);

    // nano_wait(1000000000);

    // OLED_Clear(B_Color);

    // nano_wait(1000000000);

    // OLED_Clear(A_Color);

    // nano_wait(1000000000);
    
    OLED_Clear(BLACK); 

    // nano_wait(1000000000);

    const char *S = "ECE477 Guitar Tuner";

    OLED_DrawString(0, 0, WHITE, BLACK, S, 12);

    const char *T = "Press middle button";

    OLED_DrawString(0, 52, WHITE, BLACK, T, 12);

    const char *H = "to begin standard";

    OLED_DrawString(0, 64, WHITE, BLACK, H, 12);

    const char *X = "tuning.";

    OLED_DrawString(0, 76, WHITE, BLACK, X, 12);

    OLED_DrawArrow(1, 90, B_Color);

    for(;;)
    {
        // if(!(GPIOB->IDR & (1 << 11)))
        // {
        //     // togglexn(GPIOD, 12);
        //     // nano_wait(1000000);
        //     const char *R = "Right Button Pressed";
        //     OLED_Clear(BLACK);
        //     OLED_DrawString(0, 63, WHITE, BLACK, R, 12);
        // }
        // if(!(GPIOB->IDR & (1 << 13)))
        // {
        //     // togglexn(GPIOD, 13);
        //     const char *M = "Middle Button Pressed";
        //     OLED_Clear(BLACK);
        //     OLED_DrawString(0, 63, WHITE, BLACK, M, 12);
        // }
        // if(!(GPIOB->IDR & (1 << 15)))
        // {
        //     // togglexn(GPIOD, 14);
        //     // nano_wait(1000000);
        //     const char *L = "Left Button Pressed";
        //     OLED_Clear(BLACK);
        //     OLED_DrawString(0, 63, WHITE, BLACK, L, 12);
        // }
    }
}