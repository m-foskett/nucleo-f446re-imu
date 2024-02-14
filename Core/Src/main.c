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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <LCD1602.h>
#include <stdbool.h>
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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId lcdTaskHandle;
osThreadId imuTaskHandle;
osThreadId uartTaskHandle;
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
static void MX_TIM1_Init(void);
void startLCDTask(void const * argument);
void startIMUTask(void const * argument);
void startUARTTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool MPU6050_Init(uint8_t *buf){
	uint8_t who;
	uint8_t data;
	HAL_StatusTypeDef ret; // Error Status Struct
	// Check if the MPU-6050 sensor is working by reading the WHO_AM_I register (0x75)
	//	- Expected Result: 0x68 (104)
	ret = HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_WHO_AM_I, 1, &who, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		strcpy((char*)buf, "Error finding device!\r\n");
		return false;
	}
	// If the device is available and working
	if(who == 114){
		// Wake up the MPU-6050 IMU by writing 0 to the Power Management 1 Register
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error waking up the device!\r\n");
			return false;
		}
		// Set the Sample Rate Divider(SMPLRT_DIV) used to generate the Sample Rate with the formula:
		//	Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		//	where Gyroscope Output Rate = 8kHz
		// Set SMPLRT_DIV >= 7 to avoid duplicate samples of the accelerometer which has an output rate of 1kHz
		data = 0x07;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_SMPRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error setting the Sample Rate Divider!\r\n");
			return false;
		}
		// Configure the Full Scale range for the Accelerometer and Gyroscope
		// Gyroscope: FS_SEL = 0,1,2,3 corresponds to Full Scale Range of +- 250,500,1000,2000 deg/s respectively
		// Accelerometer: AFS_SEL = 0,1,2,3 corresponds to Full Scale Range of +- 2,4,8,16 g
		// Self test of both devices is not set, +- 250deg/s & +-2g
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error configuring the gyroscope full scale range!\r\n");
			return false;
		}
		data = 0x00;
		ret = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, REG_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
		if(ret != HAL_OK){
			strcpy((char*)buf, "Error configuring the accelerometer full scale range!\r\n");
			return false;
		}
		strcpy((char*)buf, "Successful Initialisation of MPU6050!\r\n");
		return true;
	} else {
		strcpy((char*)buf, "Wrong device found!\r\n");
		return false;
	}

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
	sprintf((char*)buf, "Ax: %.2fg Ay: %.2fg Az: %.2fg\r\nGx: %.2f*/s Gy: %.2f*/s Gz: %.2f*/s\r\n\n",
			sensorValues->accel_x, sensorValues->accel_y, sensorValues->accel_z,
			sensorValues->gyro_x, sensorValues->gyro_y, sensorValues->gyro_z);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Queue Handler
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
xQueueHandle St_Queue_Handler;

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // Start Timer 1 before initialising the LCD1602 module
  HAL_TIM_Base_Start(&htim1);

  // Initialise the LCD1602 module
  LCD_Initialise();
  // Place the Cursor at (0,0)
  LCD_PlaceCursor(0, 0);
  // Display a message to show that the LCD1602 module is initialised
  LCD_SendString("LCD Initialised");
  HAL_Delay(3000);
  LCD_Clear();

  // Initialise the MPU6050 IMU
  bool success = MPU6050_Init(buf);
  if(success){
	  LCD_SendString("MPU6050 Ready!");
	  HAL_Delay(3000);
	  LCD_Clear();
  };
  // Transmit the status of the MPU6050 initialisation
  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of lcdTask */
  osThreadDef(lcdTask, startLCDTask, osPriorityAboveNormal, 0, 128);
  lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, startIMUTask, osPriorityNormal, 0, 128);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, startUARTTask, osPriorityNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA7 PA8
                           PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startLCDTask */
/**
  * @brief  Function implementing the lcdTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startLCDTask */
void startLCDTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  // Display the MPU6050 accelerometer readings on the LCD1602 module
		sprintf((char*)buf, "Ax%.2f Ay%.2f",
				sensorValues.accel_x, sensorValues.accel_y);
		LCD_SendString((char*)buf);
		LCD_PlaceCursor(1, 5);
		sprintf((char*)buf, "Az%.2f", sensorValues.accel_z);
		LCD_SendString((char*)buf);
//		HAL_Delay(1000); // 1 sample/second
		osDelay(1000);
		LCD_Clear();
		// Display the MPU6050 gyroscope readings on the LCD1602 module
		sprintf((char*)buf, "Gx%.2f Gy%.2f",
				sensorValues.gyro_x, sensorValues.gyro_y);
		LCD_SendString((char*)buf);
		LCD_PlaceCursor(1, 5);
		sprintf((char*)buf, "Gz%.2f", sensorValues.gyro_z);
		LCD_SendString((char*)buf);
//		HAL_Delay(1000); // 1 sample/second, need to add CMSIS-RTOS to display and read at the same time via multi-threading
		osDelay(1000);
		LCD_Clear();
  }
  // Fallback cleanup of thread in case forever loop was exited accidentally
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startIMUTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startIMUTask */
void startIMUTask(void const * argument)
{
  /* USER CODE BEGIN startIMUTask */
  /* Infinite loop */
  for(;;)
  {
	  // Convert the received data into a readable value
	  MPU6050_Read_Sensor_Values(&sensorValues, buf);

	  osDelay(1000);
  }
  // Fallback cleanup of thread in case forever loop was exited accidentally
  osThreadTerminate(NULL);
  /* USER CODE END startIMUTask */
}

/* USER CODE BEGIN Header_startUARTTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startUARTTask */
void startUARTTask(void const * argument)
{
  /* USER CODE BEGIN startUARTTask */
  /* Infinite loop */
  for(;;)
  {
	// Transmit the message(data or error) in blocking mode
	// 	- Casting buffer as the function requires pointer to char array
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

    osDelay(1000);
  }
  // Fallback cleanup of thread in case forever loop was exited accidentally
  osThreadTerminate(NULL);
  /* USER CODE END startUARTTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
