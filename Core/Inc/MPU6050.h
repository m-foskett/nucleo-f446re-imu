/*
 * MPU6050.h
 *
 *  Created on: Feb 17, 2024
 *      Author: foske
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

typedef enum MPU6050_Initialisation_Status {
	INIT_SUCCESS,
	DEVICE_NOT_FOUND,
	WRONG_DEVICE,
	DEVICE_ASLEEP,
	ERROR_SMPLRT_DIV,
	ERROR_GYRO_CONFIG,
	ERROR_ACC_CONFIG,
} MPU6050_Initialisation_Status;

typedef enum MPU6050_Measure_Status {
	READ_SUCCESS,
	ERROR_READING_GYRO,
	ERROR_READING_ACC,
} MPU6050_Measure_Status;

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	MPU6050_Measure_Status status;
} MPU6050_Values;

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

// Function: MPU6050_Initialisation_Status MPU6050_Init(void)
// Parameters:
//	- void
// Returns:
//	- enum MPU6050_Initialisation_Status: the status of the initialisation whether successful or failed for various reasons
// Purpose:
//	- Initialise and configure the MPU6050 to prepare for reading sensor data
MPU6050_Initialisation_Status MPU6050_Init(void);

// Function: void MPU6050_Read_Sensor_Values(MPU6050_Values *sensorValues)
// Parameters:
//	- MPU6050_Values *sensorValues: a pointer to the struct which contains all of the sensor data and the status of the data reading
// Returns:
//	- void
// Purpose:
//	- Read the sensor data from the MPU6050 device and also return the status of the data reading
void MPU6050_Read_Sensor_Values(MPU6050_Values *sensorValues);

#endif /* INC_MPU6050_H_ */
