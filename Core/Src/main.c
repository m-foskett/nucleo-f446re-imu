/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} MPU6050_Values;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Address of the MPU-6050 IMU
// Might need to be left shifted by 1
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU6050_Init(uint8_t *buf){
	uint8_t who;
	uint8_t data;
	HAL_StatusTypeDef ret; // Error Status Struct
	// Check if the MPU-6050 sensor is working by reading the WHO_AM_I register (0x75)
	//	- Expected Result: 0x68 (104)
	ret = HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_WHO_AM_I, 1, &who, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		strcpy((char*)buf, "Error finding device!\r\n");
		return;
	}
	// If the device is available and working
	if(who == 0x68){
		// Wake up the MPU-6050 IMU by writing 0 to the Power Management 1 Register
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error waking up the device!\r\n");
			return;
		}
		// Set the Sample Rate Divider(SMPLRT_DIV) used to generate the Sample Rate with the formula:
		//	Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		//	where Gyroscope Output Rate = 8kHz
		// Set SMPLRT_DIV >= 7 to avoid duplicate samples of the accelerometer which has an output rate of 1kHz
		data = 0x07;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_SMPRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error setting the Sample Rate Divider!\r\n");
			return;
		}
		// Configure the Full Scale range for the Accelerometer and Gyroscope
		// Gyroscope: FS_SEL = 0,1,2,3 corresponds to Full Scale Range of +- 250,500,1000,2000 deg/s respectively
		// Accelerometer: AFS_SEL = 0,1,2,3 corresponds to Full Scale Range of +- 2,4,8,16 g
		// Self test of both devices is not set, +- 250deg/s & +-2g
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error configuring the gyroscope full scale range!\r\n");
			return;
		}
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error configuring the accelerometer full scale range!\r\n");
			return;
		}
	}
	strcpy((char*)buf, "Successful Initialisation of MPU6050!\r\n");
}

void MPU6050_Read_Sensor_Values(MPU6050_Values *sensorValues, uint8_t *buf){
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
		strcpy((char*)buf, "Error Reading Accel Values!\r\n");
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
		strcpy((char*)buf, "Error Reading Gyro Values!\r\n");
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
	sprintf((char*)buf, "Accelerometer Readings\r\nAccel_x: %.2fg\r\n\nAccel_y: %.2fg\r\n", sensorValues->accel_x, sensorValues->accel_y);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t buf[70]; // Serial Buffer
	MPU6050_Values sensorValues;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Initialise the MPU6050 IMU
  MPU6050_Init(buf);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Convert the received data into a readable value
	  MPU6050_Read_Sensor_Values(&sensorValues, buf);
	  // Transmit the message(data or error) in blocking mode
	  // 	- Casting buffer as the function requires pointer to char array
	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  HAL_Delay(500); // Half second delay between repeated transmissions for 2 samples/second
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
