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
	ILI9341_Draw_Text( "TEST 1234567890 !!!", 5,0, WHITE, 2, BLACK);
}
// -----------------------------------------------------------------------


// -----------------------------------------------------------------------

