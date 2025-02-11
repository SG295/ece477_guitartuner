//===========================================================================
// oled.c: Adapted from 362's "lcd.c" helper function as well as...
//===========================================================================

#include "oled.h"

void nano_wait(int t); // FROM ECE362 LABS

oled_dev_t oleddev; 

#define SPI SPI1

#define CS_NUM  4
#define CS_BIT  (1<<CS_NUM)
#define CS_HIGH do { GPIOA->BSRR = GPIO_BSRR_BS_4; } while(0)
#define CS_LOW do { GPIOA->BSRR = GPIO_BSRR_BR_4; } while(0)
#define RESET_NUM 9
#define RESET_BIT (1<<RESET_NUM)
#define RESET_HIGH do { GPIOA->BSRR = GPIO_BSRR_BS_9; } while(0)
#define RESET_LOW  do { GPIOA->BSRR = GPIO_BSRR_BR_9; } while(0)
#define DC_NUM 8
#define DC_BIT (1<<DC_NUM)
#define DC_HIGH do { GPIOA->BSRR = GPIO_BSRR_BS_8; } while(0)
#define DC_LOW  do { GPIOA->BSRR = GPIO_BSRR_BR_8; } while(0)

// Set the CS pin low if val is non-zero.
// Note that when CS is being set high again, wait on SPI to not be busy.
static void oled_select(int val)
{
    if (val == 0) {
        while(SPI1->SR & SPI_SR_BSY);
        CS_HIGH;
    } else {
        while((GPIOA->ODR & (CS_BIT)) == 0) {
            ; // If CS is already low, this is an error.  Loop forever.
            // This has happened because something called a drawing subroutine
            // while one was already in process.  For instance, the main()
            // subroutine could call a long-running LCD_DrawABC function,
            // and an ISR interrupts it and calls another LCD_DrawXYZ function.
            // This is a common mistake made by students.
            // This is what catches the problem early.
        }
        CS_LOW;
    }
}

// If val is non-zero, set nRESET low to reset the display.
static void oled_reset(int val)
{
    if (val) {
        RESET_LOW;
    } else {
        RESET_HIGH;
    }
}

// If val is 1, select registers, otherwise select data
static void oled_reg_select(int val)
{
    if (val == 1) { // select registers
        DC_LOW; // clear
    } else { // select data
        DC_HIGH; // set
    }
}

void OLED_Reset(void)
{
    oleddev.reset(1);
    nano_wait(100000000);
    oleddev.reset(0);
    nano_wait(50000000); 
}

void OLED_WR_REG(uint8_t data) // CHECK THESE
{
    while((SPI->SR & SPI_SR_BSY) != 0); // ensure no other operation is running
    oleddev.reg_select(1); //DC goes to 0
    *((volatile uint8_t*)&SPI->DR) = data; // write data to the data register
    // NOTE: does this just send right away? 
}

void OLED_WR_DATA(uint8_t data)
{
    while((SPI->SR & SPI_SR_BSY) != 0); // ensure no other operation is running
    oleddev.reg_select(0); //DC goes to 1
    *((volatile uint8_t*)&SPI->DR) = data; // write data to the data register
    // NOTE: does this just send right away? 
}

// Prepare to write 16-bit data to the OLED
void OLED_WriteData16_Prepare()
{
    oleddev.reg_select(0); //ensure is in data mode
    SPI -> CR1 |= SPI_CR1_DFF; //config data frame to 16 bits
}

// Write 16-bit data
void OLED_WriteData16(u16 data)
{
    while((SPI->SR & SPI_SR_TXE) == 0); // wait if a transmission is happening
    SPI->DR = data; //set to data
}

// Finish writing 16-bit data
void OLED_WriteData16_End()
{
    SPI->CR1 &= ~SPI_CR1_DFF; // data frame back to 8 bits
}

// Set a register and write 8-bits (OR MORE) to it
void OLED_WriteRegMult(uint8_t OLED_Reg, uint16_t *OLED_RegValues, uint8_t NumRegs)
{
    OLED_WR_REG(OLED_Reg);
    for(int i = 0; i < NumRegs; i++)
    {
        OLED_WR_DATA(OLED_RegValues[0]);
    }
}

void OLED_WriteRegOnce(uint8_t OLED_Reg, uint16_t OLED_RegValue)
{
    OLED_WR_REG(OLED_Reg);
    OLED_WR_DATA(OLED_RegValue);
}

void OLED_WriteRAM_Prepare(void)
{
    OLED_WR_REG(oleddev.wramcmd);
}

void OLED_direction(uint8_t direction)
{
    oleddev.setxcmd = 0x15; // Col
    oleddev.setycmd = 0x75; // Row
    oleddev.wramcmd = 0x5C; // Ram W
    // switch(direction)
    // {
    //     case 0:
    //         oleddev.width = OLED_W;
    //         oleddev.height = OLED_H;
    //         OLED_WriteReg();
    // }
}

// Initalization sequence
void OLED_Init(void (*reset)(int), void (*select)(int), void(*reg_select)(int))
{
    oleddev.reset = oled_reset;
    oleddev.select = oled_select;
    oleddev.reg_select = oled_reg_select;
    if(reset)       {oleddev.reset = reset;}
    if(select)      {oleddev.select = select;}
    if(reg_select)  {oleddev.reg_select = reg_select;}
    oleddev.select(1); 
    OLED_Reset(); 

    //Init Sequenece for OLED SSD1351 - DOUBLE CHECK SOME OF THIS IF HAVING ISSUES (ref: https://github.com/afiskon/stm32-ssd1351/blob/master/Lib/ssd1351/ssd1351.c#L5)
    OLED_WriteRegOnce(0xFD, 0x12); // COMMANDLOCK - unlocks OLED driver
    OLED_WriteRegOnce(0xFD, 0xB1); // COMMANDLOCK - makes commands accessible
    OLED_WR_REG(0xAE); // SETSLEEPMODE - Set sleep mode ON, display OFF
    OLED_WR_REG(0xB3); // CLKDIV - Reset is 1, don't write anything else here
    // OLED_WR_REG(0xF1); // NOT SURE ABOUT THIS
    OLED_WriteRegOnce(0xCA, 0x7F); // MUXRATIO - set MUX ratio to default of 127
    OLED_WriteRegOnce(0xA0, 0x74); // SETREMAP - swaps color sequence (check this), scan dir, enable COM split
    uint16_t dataRowCol[] = {0x00, 0x7F}; // check this \/
    OLED_WriteRegMult(0x15, dataRowCol, sizeof(dataRowCol)); // SETCOL - sets to start of 0, end of 127
    OLED_WriteRegMult(0x75, dataRowCol, sizeof(dataRowCol)); // SETROW - sim ^, set strt 0, end 127
    OLED_WriteRegOnce(0xA1, 0x00); // STARTLINE - set to default value of 0
    OLED_WriteRegOnce(0xA2, 0x00); // DISPLAYOFFSET - set to 0
    OLED_WriteRegOnce(0xB5, 0x00); // SETGPIO - set GP0 and 1 to HiZ and input disabled
    OLED_WriteRegOnce(0xAB, 0x01); // FUNCTSEL - enable Vdd regulator, sel 8 bit para interface
    OLED_WriteRegOnce(0xB1, 0x32); // PRECHARGE -  phase 1 period of 5 DCLKS, phase 2 period of 3 DCLKS
    OLED_WriteRegOnce(0xBE, 0x05); // VCOMH - set to default of 0.82xVcc for COM deselt voltage
    OLED_WR_REG(0xA6); // DISPLAYMODE - set to normal display mode :D 
    uint16_t dataContrast[] = {0xC8, 0x80, 0xC8};
    OLED_WriteRegMult(0xC1, dataContrast, sizeof(dataContrast)); // SETCONTRAST - sets the contrast of each color value
    OLED_WriteRegOnce(0xC7, 0x0F); // MASTERCONTRAST - default value
    uint16_t dataSet[] = {0xA0, 0xB5, 0x55};
    OLED_WriteRegMult(0xB4, dataSet, sizeof(dataSet)); // SETSVL - External VSL, reset hard coded from manual
    OLED_WriteRegOnce(0xB6, 0x01); // PRECHARGE2 - set precharge period to to 1 DCLKS
    OLED_WR_REG(0xAF); // SETSLEEPMODE - Set sleep mode OFF, display ON 

    oleddev.setxcmd = 0x15; // Col
    oleddev.setycmd = 0x75; // Row
    oleddev.wramcmd = 0x5C; // Ram W
    oleddev.height = OLED_H;
    oleddev.width = OLED_W; 

    oleddev.select(0); 
}