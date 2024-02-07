/*
 * LCD1602.c
 *
 *  Created on: Feb 7, 2024
 *      Author: Mark Foskett
 */

#include <LCD1602.h>

#include "stm32f4xx_hal.h"

// LCD Pinout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// - Modify accordingly to match the project setup
#define RS_PIN GPIO_PIN_10
#define RS_PORT GPIOB
#define RW_PIN GPIO_PIN_8
#define RW_PORT GPIOA
#define EN_PIN GPIO_PIN_9
#define EN_PORT GPIOA
#define D4_PIN GPIO_PIN_7
#define D4_PORT GPIOC
#define D5_PIN GPIO_PIN_6
#define D5_PORT GPIOB
#define D6_PIN GPIO_PIN_7
#define D6_PORT GPIOA
#define D7_PIN GPIO_PIN_6
#define D7_PORT GPIOA

// Timer Handler ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define timer htim1

// Extern declaration of timer
extern TIM_HandleTypeDef timer;
// Function: void delay(uint16_t micros)
// Parameters:
//	- uint16_t micros: the desired delay in microseconds
// Returns:
//	- void
// Purpose: Generate a small delay in microseconds
void delay(uint16_t micros){
	// Macros to set and get the value of a counter using Timer 1
	__HAL_TIM_SET_COUNTER(&timer, 0);
	while(__HAL_TIM_GET_COUNTER(&timer) < micros);
}

// Enumerations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
enum RegisterSelect {
	INSTRUCTION,
	DATA,
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Function Definitions

// Private Function: void lcd_send(char data, RegisterSelect register)
// Parameters:
//	- char data: the data to be sent to the chosen register
//	- enum RegisterSelect register: the selected register to be written into
// Returns:
//	- void
// Purpose: Write data to the selected register of the LCD1602 module
void lcd_send(char data, enum RegisterSelect reg){
	// Select the register to be written to
	HAL_GPIO_WritePin(RS_PORT, RS_PIN, reg);

	// Write the data to the pins
	HAL_GPIO_WritePin(D4_PORT, D4_PIN, ((data>>0) & 0x01));
	HAL_GPIO_WritePin(D5_PORT, D5_PIN, ((data>>1) & 0x01));
	HAL_GPIO_WritePin(D6_PORT, D6_PIN, ((data>>2) & 0x01));
	HAL_GPIO_WritePin(D7_PORT, D7_PIN, ((data>>3) & 0x01));

	// Send the data to the LCD1602 module by quickly toggling the Enable Pin
	// Delay: adjust until the LCD1602 module displays correctly
	int commandDelay = 50;
	HAL_GPIO_WritePin(EN_PORT, EN_PIN, 1);
	delay(commandDelay);
	HAL_GPIO_WritePin(EN_PORT, EN_PIN, 0);
	delay(commandDelay);
};

void LCD_Initialise(void){
	// Set the READ/WRITE SELECTION to be REGISTER WRITE (R/W=0)
	HAL_GPIO_WritePin(RW_PORT, RW_PIN, 0);
	// 4-bit Initialisation Sequence ~~~~~~~~~~~~~~~~~~~~~~~~
	// Power ON
	HAL_Delay(50); // Need a delay > 15ms
	LCD_SendCommand(0x30); // FUNCTION SET
	HAL_Delay(5); // Need a delay > 4.1ms
	LCD_SendCommand(0x30); // FUNCTION SET
	HAL_Delay(1); // Need a delay > 100us
	LCD_SendCommand(0x30); // FUNCTION SET

	// Display Setup Initialisation
	HAL_Delay(10); // Add a 10ms delay before and after swapping to 4-bit interface
	LCD_SendCommand(0x20); // FUNCTION SET: Swap to 4-bit Interface (DB5=1, DL/DB4=0)
	HAL_Delay(10);
	LCD_SendCommand(0x28); // FUNCTION SET: DL=0 (4-bit mode), N=1 (2 line display) F=0 (5x7 style font)
	HAL_Delay(1); // Add a 1ms delay between subsequent commands
	LCD_SendCommand(0x08); // DISPLAY SWITCH: Turn the Display OFF (D=0, C=0, B=0)
	HAL_Delay(1);
	LCD_SendCommand(0x01); // SCREEN CLEAR
	HAL_Delay(1);
	HAL_Delay(1);
	LCD_SendCommand(0x06); // INPUT SET: Increment Mode for Cursor (I/D=1), No Cursor Shift (S=0)
	HAL_Delay(1);
	LCD_SendCommand(0x0C); // DISPLAY SWITCH: Turn the Display ON (D=1, C=0, B=0)
	HAL_Delay(1);
}

void LCD_PlaceCursor(int row, int col){
	switch(row){
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}
	// Send the cursor position command
	LCD_SendCommand(col);
}

void LCD_Clear(void){
	// Send the SCREEN CLEAR command (0x01) to the Instruction Register
	LCD_SendCommand(0x01);
	// Add a 2 millisecond delay after sending the SCREEN CLEAR command
	HAL_Delay(2);
}

void LCD_SendCommand(char command){
	// Variables to manipulate the command byte to send 4 bits at a time
	char upperNibble;
	char lowerNibble;
	// Send the upper nibble
	upperNibble = (command>>4) & 0x0F;
	lcd_send(upperNibble, INSTRUCTION);
	// Send the lower nibble
	lowerNibble = command & 0x0F;
	lcd_send(lowerNibble, INSTRUCTION);
};

void LCD_SendData(char data){
	// Variables to manipulate the command byte to send 4 bits at a time
	char upperNibble;
	char lowerNibble;
	// Send the upper nibble
	upperNibble = (data>>4) & 0x0F;
	lcd_send(upperNibble, DATA);
	// Send the lower nibble
	lowerNibble = data & 0x0F;
	lcd_send(lowerNibble, DATA);
};

void LCD_SendString(char *string){
	// Loop through the entire string and send each char to the Data Register of the LCD1602 module
	while(*string){
		LCD_SendData(*string++);
	}
}



















