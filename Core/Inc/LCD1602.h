/*
 * LCD1602.h
 *
 *  Created on: Feb 7, 2024
 *      Author: Mark Foskett
 */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

// Address of the MPU-6050 IMU
// 	- Needs to be left shifted by 1 as it is a 7-bit address
static const uint8_t IMU_ADDR = 0x68 << 1;
// PWR_MGMT_1 Register
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
// WHO_AM_I register for the MPU6050
static const uint8_t REG_WHO_AM_I = 0x75;
// Sample Rate Divider Register
static const uint8_t REG_SMPRT_DIV = 0x19;
// Gyroscope Configuration Register
static const uint8_t REG_GYRO_CONFIG = 0x1B;
// Accelerometer Configuration Register
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
// Accelerometer Measurement Register
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
// Gyroscope Measurement Register
static const uint8_t REG_GYRO_XOUT_H = 0x43;

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
