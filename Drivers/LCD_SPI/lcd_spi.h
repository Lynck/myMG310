#ifndef LCD_SPI_H_
#define LCD_SPI_H_

#include <stdint.h>

#define LCD_SPI_WIDTH  (128U)
#define LCD_SPI_HEIGHT (160U)

#define LCD_COLOR_BLACK   (0x0000U)
#define LCD_COLOR_WHITE   (0xFFFFU)
#define LCD_COLOR_RED     (0xF800U)
#define LCD_COLOR_GREEN   (0x07E0U)
#define LCD_COLOR_BLUE    (0x001FU)
#define LCD_COLOR_CYAN    (0x07FFU)
#define LCD_COLOR_YELLOW  (0xFFE0U)
#define LCD_COLOR_MAGENTA (0xF81FU)

void LCD_SPI_Init(void);
void LCD_SPI_Fill(uint16_t color);
void LCD_SPI_DrawString(uint16_t x, uint16_t y, const char *text,
                        uint16_t foreground, uint16_t background);

#endif
