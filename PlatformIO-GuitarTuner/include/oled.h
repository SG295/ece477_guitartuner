#include "stm32f407xx.h"
#include "stdlib.h"
#include <stdio.h>
#include <stdint.h>

//===========================================================================
// oled.h: Adapted from 362's "lcd.h" header file which referenced LCDwiki
//===========================================================================

// shorthand notation for 8-bit and 16-bit unsigned integers
typedef uint8_t u8;
typedef uint16_t u16;

typedef struct
{
    uint16_t width; 
    uint16_t height;
    uint16_t id; 
    uint8_t dir; 
    uint16_t wramcmd;
    uint16_t setxcmd;
    uint16_t setycmd;
    void (*reset)(int);
    void (*select)(int);
    void (*reg_select)(int);
} oled_dev_t;


// The OLED device.
// This will be initialized by OLED_direction() so that the
// width and height will be appropriate for the rotation.
// The setxcmd and setycmd will be set so that cursor selection
// is defined properly for the rotation.
extern oled_dev_t oleddev;

// Rotation:
// 0: rotate 0
// 1: rotate: 90
// 2: rotate: 180
// 3: rotate 270
#define ROTATION       0

// The dimensions of the display.
#define OLED_W 128
#define OLED_H 128

// Some popular colors
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define YELLOW      0XFFE0
#define GBLUE       0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define BROWN       0XBC40
#define BRRED       0XFC07
#define GRAY        0X8430
#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458
#define LIGHTGREEN  0X841F
#define LIGHTGRAY   0XEF5B
#define LGRAY       0XC618
#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12

// void OLED_Setup(void);
void OLED_Init(void (*reset)(int), void (*select)(int), void (*reg_select)(int));
// void OLED_Clear(u16 Color);
// void OLED_DrawPoint(u16 x,u16 y,u16 c);
// void OLED_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 c);
// void OLED_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 c);
// void OLED_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 c);
// void OLED_Circle(u16 xc, u16 yc, u16 r, u16 fill, u16 c);
// void OLED_DrawTriangle(u16 x0,u16 y0, u16 x1,u16 y1, u16 x2,u16 y2, u16 c);
// void OLED_DrawFillTriangle(u16 x0,u16 y0, u16 x1,u16 y1, u16 x2,u16 y2, u16 c);
// void OLED_DrawChar(u16 x,u16 y,u16 fc, u16 bc, char num, u8 size, u8 mode);
// void OLED_DrawString(u16 x,u16 y, u16 fc, u16 bg, const char *p, u8 size, u8 mode);

//===========================================================================
// C Picture data structure.
//===========================================================================
typedef const struct {
    unsigned int   width;
    unsigned int   height;
    unsigned int   bytes_per_pixel; // 2:RGB16, 3:RGB, 4:RGBA
    unsigned char  pixel_data[0]; // variable length array
} Picture;

void OLED_DrawPicture(u16 x0, u16 y0, const Picture *pic);