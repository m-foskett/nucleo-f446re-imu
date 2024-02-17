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
