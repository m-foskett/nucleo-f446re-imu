/*
 * LCD1602.h
 *
 *  Created on: Feb 7, 2024
 *      Author: Mark Foskett
 */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

// Function: void LCD_Initialise(void)
// Parameters:
//	- void
// Returns:
//	- void
// Purpose:
//	- Initialise the LCD1602 module with the 4-bit initialisation sequence specified in the data sheet
void LCD_Initialise(void);

// Function: void LCD_PlaceCursor(int row, int col)
// Parameters:
//	- int row: the desired row to place the cursor (0 - 1)
//	- int col: the desired column to place the cursor (0 - 15)
// Returns:
//	- void
// Purpose:
//	- Place the cursor at the position specified by row and col
void LCD_PlaceCursor(int row, int col);

// Function: void LCD_Clear(void)
// Parameters:
//	- void
// Returns:
//	- void
// Purpose:
//	- Clear the display of the LCD1602 module
void LCD_Clear(void);

// Function: void LCD_SendCommand(char command)
// Parameters:
//	- char command: the command to send to the LCD1602 module, must be listed in the instruction set
// Returns:
//	- void
// Purpose:
//	- Send a command to the instruction register of the LCD1602 module
void LCD_SendCommand(char command);

// Function: void LCD_SendData(char data)
// Parameters:
//	- char data: the data to be written to the LCD1602 module
// Returns:
//	- void
// Purpose:
//	- Send a byte of data to the data register of the LCD1602 module
void LCD_SendData(char data);

// Function: void LCD_SendString(char *string)
// Parameters:
//	- char *string: pointer to a string of chars to be written to the LCD1602 module
// Returns:
//	- void
// Purpose:
//	- Send a string of data to the data register of the LCD1602 module
void LCD_SendString(char *string);



#endif /* INC_LCD1602_H_ */
