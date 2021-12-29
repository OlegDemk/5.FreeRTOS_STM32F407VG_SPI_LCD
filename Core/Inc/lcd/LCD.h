/*
 * LCD.h
 *
 *  Created on: Apr 7, 2021
 *      Author: odemki
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_


void LCD_init(void);
void lcd_test_print(void);
////////////////////////////////////////////////////////////
// MICRO-MENU

void menu(void);                         // FUNCTION FOR TEST !!!!!!!!!!!!!!!!!!!!!!!!!!!!

#include <stddef.h>

 /** Type define for a menu item. Menu items should be initialized via the helper
  *  macro \ref MENU_ITEM(), not created from this type directly in user-code.
  */
 typedef const struct Menu_Item {
  const struct Menu_Item *Next; /**< Pointer to the next menu item of this menu item */
  const struct Menu_Item *Previous; /**< Pointer to the previous menu item of this menu item */
  const struct Menu_Item *Parent; /**< Pointer to the parent menu item of this menu item */
  const struct Menu_Item *Child; /**< Pointer to the child menu item of this menu item */
  void (*SelectCallback)(void); /**< Pointer to the optional menu-specific select callback of this menu item */
  void (*EnterCallback)(void); /**< Pointer to the optional menu-specific enter callback of this menu item */
  const char Text[]; /**< Menu item text to pass to the menu display callback function */
 } Menu_Item_t;		// Назва типу даних - const struct Menu_Item

 /** Creates a new menu item entry with the specified links and callbacks.
  *
  *  \param[in] Name      Name of the menu entry, must be unique.
  *  \param[in] Next      Name of the next linked menu item, or \ref NULL_MENU if no menu link.
  *  \param[in] Previous  Name of the previous linked menu item, or \ref NULL_MENU if no menu link.
  *  \param[in] Parent    Name of the parent linked menu item, or \ref NULL_MENU if no menu link.
  *  \param[in] Child     Name of the child linked menu item, or \ref NULL_MENU if no menu link.
  *  \param[in] SelectFunc  Function callback to execute when the menu item is selected, or \c NULL for no callback.
  *  \param[in] EnterFunc   Function callback to execute when the menu item is entered, or \c NULL for no callback.
  */
 #define MENU_ITEM(Name, Next, Previous, Parent, Child, SelectFunc, EnterFunc, Text) \
  extern Menu_Item_t const Next;     \
  extern Menu_Item_t const Previous; \
  extern Menu_Item_t const Parent;   \
  extern Menu_Item_t const Child;  \
  Menu_Item_t const Name = {&Next, &Previous, &Parent, &Child, SelectFunc, EnterFunc, Text}

 /** Relative navigational menu entry for \ref Menu_Navigate(), to move to the menu parent. */
 #define MENU_PARENT         (Menu_GetCurrentMenu()->Parent)

 /** Relative navigational menu entry for \ref Menu_Navigate(), to move to the menu child. */
 #define MENU_CHILD          (Menu_GetCurrentMenu()->Child)

 /** Relative navigational menu entry for \ref Menu_Navigate(), to move to the next linked menu item. */
 #define MENU_NEXT           (Menu_GetCurrentMenu()->Next)

 /** Relative navigational menu entry for \ref Menu_Navigate(), to move to the previous linked menu item. */
 #define MENU_PREVIOUS       (Menu_GetCurrentMenu()->Previous)

 /** Null menu entry, used in \ref MENU_ITEM() definitions where no menu link is to be made. */
 extern Menu_Item_t NULL_MENU;

 /** Retrieves the currently selected meny item.
  *
  *  \return Pointer to the currently selected meny item.
  */
 Menu_Item_t* Menu_GetCurrentMenu(void);

 /** Navigates to an absolute or relative menu entry.
  *
  * \param[in] NewMenu  Pointer to the absolute menu item to select, or one of \ref MENU_PARENT,
  *                     \ref MENU_CHILD, \ref MENU_NEXT or \ref MENU_PREVIOUS for relative navigation.
  */
 void Menu_Navigate(Menu_Item_t* const NewMenu);

 /** Configures the menu text write callback function, fired for all menu items. Within this callback
  *  function the user should implement code to display the current menu text stored in \ref MENU_ITEM_STORAGE
  *  memory space.
  *
  *  \ref WriteFunc  Pointer to a callback function to execute for each selected menu item.
  */
 void Menu_SetGenericWriteCallback(void (*WriteFunc)(const char* Text));

 /** Enters the currently selected menu item, running its configured callback function (if any). */
 void Menu_EnterCurrentItem(void);

////////////////////////////////////////////////////////////

#endif /* INC_LCD_H_ */
