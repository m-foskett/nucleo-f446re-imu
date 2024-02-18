/*
 * MPU6050.c
 *
 *  Created on: Feb 17, 2024
 *      Author: foske
 */

#include <MPU6050.h>
#include "stm32f4xx_hal.h"

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

// I2C Handler ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define I2C hi2c1
//
//// Extern declaration of I2C
extern I2C_HandleTypeDef I2C;

MPU6050_Initialisation_Status MPU6050_Init(void){
	uint8_t who;
	uint8_t data;
	HAL_StatusTypeDef ret; // Error Status Struct

	// Check if the MPU-6050 sensor is working by reading the WHO_AM_I register (0x75)
	//	- Expected Result: 0x68 (104)
	ret = HAL_I2C_Mem_Read(&I2C, IMU_ADDR, REG_WHO_AM_I, 1, &who, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return DEVICE_NOT_FOUND;
	}
	// If the device is available and working
	if(who == 114){
		// Wake up the MPU-6050 IMU by writing 0 to the Power Management 1 Register
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			return DEVICE_ASLEEP;
		}
		// Set the Sample Rate Divider(SMPLRT_DIV) used to generate the Sample Rate with the formula:
		//	Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		//	where Gyroscope Output Rate = 8kHz
		// Set SMPLRT_DIV >= 7 to avoid duplicate samples of the accelerometer which has an output rate of 1kHz
		data = 0x07;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_SMPRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			return ERROR_SMPLRT_DIV;
		}
		// Configure the Full Scale range for the Accelerometer and Gyroscope
		// Gyroscope: FS_SEL = 0,1,2,3 corresponds to Full Scale Range of +- 250,500,1000,2000 deg/s respectively
		// Accelerometer: AFS_SEL = 0,1,2,3 corresponds to Full Scale Range of +- 2,4,8,16 g
		// Self test of both devices is not set, +- 250deg/s & +-2g
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			return ERROR_GYRO_CONFIG;
		}
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			return ERROR_ACC_CONFIG;
		}
		return INIT_SUCCESS;
	} else {
		return WRONG_DEVICE;
	}
}

void MPU6050_Read_Sensor_Values(MPU6050_Values *sensorValues){
	uint8_t rawDataBuf[6]; // Buffer to store the raw acceleration values
	int16_t raw_accel_x;
	int16_t raw_accel_y;
	int16_t raw_accel_z;
	int16_t raw_gyro_x;
	int16_t raw_gyro_y;
	int16_t raw_gyro_z;
	float accel_sensitivity;
	float gyro_sensitivity;
	HAL_StatusTypeDef ret; // Error Status Struct

	// Read from the 6 acceleration registers starting at ACCEL_XOUT_H
	ret = HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_ACCEL_XOUT_H, 1, rawDataBuf, 6, HAL_MAX_DELAY);
	if(ret != HAL_OK){
//		strcpy((char*)buf, "Error Reading Accel Values!\r\n");
		sensorValues->status = ERROR_READING_ACC;
		return;
	} else {
		// Manipulate the received bytes to acquire the raw acceleration values
		raw_accel_x = (int16_t)(rawDataBuf[0] << 8 | rawDataBuf[1]);
		raw_accel_y = (int16_t)(rawDataBuf[2] << 8 | rawDataBuf[3]);
		raw_accel_z = (int16_t)(rawDataBuf[4] << 8 | rawDataBuf[5]);
	}

	// Read from the 6 gyroscope registers starting at GYRO_XOUT_H
	ret = HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_GYRO_XOUT_H, 1, rawDataBuf, 6, HAL_MAX_DELAY);
	if(ret != HAL_OK){
//		strcpy((char*)buf, "Error Reading Gyro Values!\r\n");
		sensorValues->status = ERROR_READING_GYRO;
		return;
	} else {
		// Manipulate the received bytes to acquire the raw acceleration values
		raw_gyro_x = (int16_t)(rawDataBuf[0] << 8 | rawDataBuf[1]);
		raw_gyro_y = (int16_t)(rawDataBuf[2] << 8 | rawDataBuf[3]);
		raw_gyro_z = (int16_t)(rawDataBuf[4] << 8 | rawDataBuf[5]);
	}
	// Convert the raw measurement values into scaled readings
	// Accelerometer: with FS_SEL = 0 -> +2g which corresponds to an accelerometer sensitivity per LSB of 16384 LSB/g
	// Gyroscope: with AFS_SEL = 0 -> +- 250 deg/s which corresponds to a gyroscope sensitivity per LSB of 131 LSB/deg/s
	accel_sensitivity = 16384;
	gyro_sensitivity = 131;

	sensorValues->accel_x = raw_accel_x / accel_sensitivity;
	sensorValues->accel_y = raw_accel_y / accel_sensitivity;
	sensorValues->accel_z = raw_accel_z / accel_sensitivity;

	sensorValues->gyro_x = raw_gyro_x / gyro_sensitivity;
	sensorValues->gyro_y = raw_gyro_y / gyro_sensitivity;
	sensorValues->gyro_z = raw_gyro_z / gyro_sensitivity;

	// Convert to string format
//	sprintf((char*)buf, "Ax: %.2fg Ay: %.2fg Az: %.2fg\r\nGx: %.2f*/s Gy: %.2f*/s Gz: %.2f*/s\r\n\n",
//			sensorValues->accel_x, sensorValues->accel_y, sensorValues->accel_z,
//			sensorValues->gyro_x, sensorValues->gyro_y, sensorValues->gyro_z);
	sensorValues->status = READ_SUCCESS;
}
