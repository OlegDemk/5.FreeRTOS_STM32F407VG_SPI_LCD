/*
 * spi_ili9341.h
 *
 *  Created on: Dec 30, 2021
 *      Author: odemki
 */

#ifndef INC_LCD_SPI_ILI9341_H_
#define INC_LCD_SPI_ILI9341_H_

#include "stm32f4xx_hal.h"
#include "LCD/fonts.h"
//#include <cstdlib.h>

void TFT9341_ini(uint16_t w_size, uint16_t h_size);
void TFT9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void TFT9341_FillScreen(uint16_t color);
void TFT9341_DrawPixel(int x, int y, uint16_t color);
void TFT9341_DrawCircle(uint16_t x0, uint16_t y0, int r, uint16_t color);

void speed_test(void);

#define RESET_ACTIVE() 	HAL_GPIO_WritePin(GPIOE,RESET_LCD_Pin,GPIO_PIN_RESET)
#define RESET_IDLE() 	HAL_GPIO_WritePin(GPIOE,RESET_LCD_Pin,GPIO_PIN_SET)
#define CS_ACTIVE() 	HAL_GPIO_WritePin(GPIOE,CS_LCD_Pin,GPIO_PIN_RESET)
#define CS_IDLE() 		HAL_GPIO_WritePin(GPIOE,CS_LCD_Pin,GPIO_PIN_SET)
#define DC_COMMAND() 	HAL_GPIO_WritePin(GPIOE,DC_LCD_Pin,GPIO_PIN_RESET)
#define DC_DATA() 		HAL_GPIO_WritePin(GPIOE,DC_LCD_Pin,GPIO_PIN_SET)

#define TFT9341_MADCTL_MY  0x80
#define TFT9341_MADCTL_MX  0x40
#define TFT9341_MADCTL_MV  0x20
#define TFT9341_MADCTL_ML  0x10
#define TFT9341_MADCTL_RGB 0x00
#define TFT9341_MADCTL_BGR 0x08
#define TFT9341_MADCTL_MH  0x04
#define TFT9341_ROTATION (TFT9341_MADCTL_MX | TFT9341_MADCTL_BGR)
#define	TFT9341_BLACK   0x0000
#define	TFT9341_BLUE    0x001F
#define	TFT9341_RED     0xF800
#define	TFT9341_GREEN   0x07E0
#define TFT9341_CYAN    0x07FF
#define TFT9341_MAGENTA 0xF81F
#define TFT9341_YELLOW  0xFFE0
#define TFT9341_WHITE   0xFFFF
//-------------------------------------------------------------------
#define swap(a,b) {int16_t t=a;a=b;b=t;}

#endif /* INC_LCD_SPI_ILI9341_H_ */
