/*
 * LCD.c
 *
 *  Created on: Apr 7, 2021
 *      Author: odemki
 */


// How add new menu. < -----------------------------------------
/*
For add new menu need.
1. Create char array with names created menu items.
2. Create new MENU_ITEM and fill in it.
3. Create function for print new menu.
4. Create new variable: menu_level_X_X_position_pointer (pointer on menu item).
	Add its variable in switch(pressed_key).
5. Create new variable: menu_level_X_X. (Variabls for optimisation drawing on LCD).
*/

#include "main.h"

#include "lcd/LCD.h"
#include "lcd/ILI9341_Touchscreen.h"
#include "lcd/ILI9341_STM32_Driver.h"
#include "lcd/ILI9341_GFX.h"
#include "lcd/snow_tiger.h"

#include <string.h>
#include <stdio.h>

extern RNG_HandleTypeDef hrng;

// -----------------------------------------------------------------------
void LCD_init(void)
{
	ILI9341_Init();
	ILI9341_Fill_Screen(BLACK);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);          // was  SCREEN_HORIZONTAL_2
	ILI9341_Fill_Screen(BLACK);
}
// -----------------------------------------------------------------------
void lcd_test_print (void)
{
	ILI9341_Draw_Text("TEST !!!", 30, 200, GREEN, 4, BLACK);

	ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 150, 150, GREEN);
	ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 100, 100, BLUE);
	ILI9341_Draw_Filled_Rectangle_Coord(50, 50, 200, 200, DARKCYAN);
	ILI9341_Draw_Filled_Rectangle_Coord(70, 70, 200, 200, CYAN);
	ILI9341_Draw_Filled_Rectangle_Coord(70, 70, 150, 150, PINK);
	ILI9341_Draw_Filled_Rectangle_Coord(60, 20, 100, 100, BLUE);
	ILI9341_Draw_Filled_Rectangle_Coord(80, 100, 200, 200, RED);
}
// -----------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
/*
 * Random generate circles
 */
void speed_test_LCD(int number_of_tests)
{
	int i =0;
	while(i <= number_of_tests)
	{
		i ++;
		//ILI9341_Draw_Text("", i, 200, YELLOW, 4, BLACK);
		uint32_t random_num = 0;
		uint16_t xr = 0;
		uint16_t yr = 0;
		uint16_t radiusr = 0;
		uint16_t colourr = 0;

		random_num = HAL_RNG_GetRandomNumber(&hrng);
		xr = random_num;
		random_num = HAL_RNG_GetRandomNumber(&hrng);
		yr = random_num;
		random_num = HAL_RNG_GetRandomNumber(&hrng);
		radiusr = random_num;
		random_num = HAL_RNG_GetRandomNumber(&hrng);
		colourr = random_num;

		xr &= 0x01FF;
		yr &= 0x01FF;
		radiusr &= 0x001F;
		colourr &= 0xFFFF;
		ILI9341_Draw_Filled_Circle(xr, yr, radiusr, colourr);
		//ILI9341_Draw_Pixel(xr, yr, WHITE);
	}

}
//-------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------

