#include "BQ27441.h"

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
    I2C1 -> CCR = (0x50<<0); // T_low/high = 1.25us, T_PCLK1 = 62.5ns, 1.25us/62.5ns=20 or 0x14
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