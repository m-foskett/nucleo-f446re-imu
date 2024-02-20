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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <LCD1602.h>
#include <MPU6050.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Queue Handler
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
xQueueHandle St_Queue_Handler;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Task Handlers
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
xTaskHandle IMU_Measure_Task_Handler;
xTaskHandle LCD_Display_Task_Handler;
xTaskHandle Initialisation_Task_Handler;
//xTaskHandle UART_Display_Task_Handler;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Task Functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void IMU_Measure_Task(void *argument);
void LCD_Display_Task(void *argument);
void Initialisation_Task(void *argument);
//void UART_Display_Task(void *argument);

// malloc and free wrappers to avoid issues with sprintf
void *malloc( size_t xBytes )
{
     return pvPortMalloc( xBytes );
}

void free( void *pvBuffer )
{
     vPortFree( pvBuffer );
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
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
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // FreeRTOS Setup BEGIN
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Create Queue
	St_Queue_Handler = xQueueCreate(2, sizeof(MPU6050_Values));
	// Error Handling for queue creation
	if(St_Queue_Handler == 0){
	  char *str = "Structured Queue creation failed!\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	} else {
	  char *str = "Structured Queue creation successful!\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	}
	// Initialisation Task
	BaseType_t xReturnValue; // Return value for task creation
	// Initialisation_Task Creation
	xReturnValue = xTaskCreate(Initialisation_Task, "Initialisation", 128, NULL, 2, &Initialisation_Task_Handler);
	if(xReturnValue == pdPASS){
	  char *str = "Initialisation task was successfully created!\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	} else {
	  char *str = "Failed to create Initialisation Task!\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	}
	// Scheduler Start
	vTaskStartScheduler();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // FreeRTOS Setup END
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /* USER CODE END 2 */

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
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// FreeRTOS Task Definitions BEGIN
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void IMU_Measure_Task (void *argument){
	// Pointer to the sensor values struct
	MPU6050_Values *sensorValues;

	// Convert ms delay to ticks
	const TickType_t tickDelay = pdMS_TO_TICKS(1000);
	char *str;

	/* Infinite loop */
	for(;;)
	{
		// Returns the remaining stack space (in words) that available to the task since task execution
		// 	- Therefore multiply by 4 for a 32-bit machine to get bytes
		UBaseType_t stackHighWaterMark;
		stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

		// Allocate memory in the 'Portable Layer' of the FreeRTOS
		//	- Returns a pointer of type void which can be cast into a pointer of any form
		sensorValues = pvPortMalloc(sizeof(MPU6050_Values));
		// Get the sensor values from the MPU6050 module
		MPU6050_Read_Sensor_Values(sensorValues);
		// Post the sensor data to the queue. The item is queued by copy, not by reference
		//		BaseType_t xQueueSend(
		//				QueueHandle_t xQueue, - handle to the queue
		//				const void * pvItemToQueue, - pointer to the item to be placed on the queue
		//				TickType_t xTicksToWait - max amount of time the task should block waiting for available space on the queue
		//		);
		BaseType_t xReturnValue; // Return value for queue send
		xReturnValue = xQueueSend(St_Queue_Handler, &sensorValues, pdMS_TO_TICKS(2000));

		if(xReturnValue == pdTRUE){
		  str = "Queue Post!\r\n";
		  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
		} else {
		  str = "Failed to post!\r\n";
		  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		  vPortFree(sensorValues); // free the memory that failed to be added to the queue
		}
		vTaskDelay(tickDelay); // 1 second delay in ticks
	}
	// Fallback cleanup of thread in case forever loop was exited accidentally
	vTaskDelete(IMU_Measure_Task_Handler);
}

void LCD_Display_Task (void *argument){
	// Pointer to the sensor values struct to receive the queue data
	MPU6050_Values *rcv_sensorValues;
	// Convert ms delay to ticks
	const TickType_t tickDelay = pdMS_TO_TICKS(1000);
	// Char pointer for string to write to lcd
	char *dataString;

	// Char pointer for string to write to UART
//	char *str;

	/* Infinite loop */
	for(;;)
	{
		// Allocate memory in the 'Portable Layer' of the FreeRTOS
		//	- Returns a pointer of type void which can be cast into a pointer of any form
		//	- NB: Memory must be allocated like this otherwise sprintf will cause a hard fault error
//		str = pvPortMalloc(sizeof(char) * 50);
		// Receive the sensor data from the queue, received by copy so a buffer of adequate size must be provided
		//		BaseType_t xQueueReceive(
		//				QueueHandle_t xQueue, - handle to the queue
		//				void *pvBuffer, - pointer to the buffer to store the received item
		//				TickType_t xTicksToWait - max amount of time the task should block waiting for an item to receive
		//										  in the case of an empty queue
		//		);
		BaseType_t xReturnValue; // Return value for queue send
		xReturnValue = xQueueReceive(St_Queue_Handler, &rcv_sensorValues, pdMS_TO_TICKS(1000));

		if(xReturnValue == pdTRUE){
			char *str = "Successfully received sensor data from the queue!\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

			// Allocate memory in the 'Portable Layer' of the FreeRTOS
			//	- Returns a pointer of type void which can be cast into a pointer of any form
			//	- NB: Memory must be allocated like this otherwise sprintf will cause a hard fault error
			dataString = pvPortMalloc(sizeof(char) * 15);

			// Display the MPU6050 accelerometer readings on the LCD1602 module
			sprintf(dataString, "Ax%.2f Ay%.2f", rcv_sensorValues->accel_x, rcv_sensorValues->accel_y);
			HAL_UART_Transmit(&huart2, (uint8_t *)dataString, strlen(dataString), HAL_MAX_DELAY);
			LCD_SendString(dataString);
			LCD_PlaceCursor(1, 5);
			sprintf(dataString, "Az%.2f", rcv_sensorValues->accel_z);
			HAL_UART_Transmit(&huart2, (uint8_t *)dataString, strlen(dataString), HAL_MAX_DELAY);
			LCD_SendString(dataString);
			vTaskDelay(tickDelay); // 1 second delay in ticks to be able to see the data before clearing and writing new data
			LCD_Clear();
			// Display the MPU6050 gyroscope readings on the LCD1602 module
			sprintf(dataString, "Gx%.2f Gy%.2f", rcv_sensorValues->gyro_x, rcv_sensorValues->gyro_y);
			HAL_UART_Transmit(&huart2, (uint8_t *)dataString, strlen(dataString), HAL_MAX_DELAY);
			LCD_SendString(dataString);
			LCD_PlaceCursor(1, 5);
			sprintf(dataString, "Gz%.2f", rcv_sensorValues->gyro_z);
			HAL_UART_Transmit(&huart2, (uint8_t *)dataString, strlen(dataString), HAL_MAX_DELAY);
			LCD_SendString(dataString);
			vTaskDelay(tickDelay); // 1 second delay in ticks to be able to see the data before clearing and writing new data
			LCD_Clear();
		} else {
		  char *str = "Failed to receive sensor data from the queue!\r\n";
		  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
		}
		// Freeing memory
		vPortFree(rcv_sensorValues); // The sensor data struct pointer received from the queue
		// Freeing memory
		vPortFree(dataString); // Data string
		// Freeing memory
//		vPortFree(str); // UART string
	}
	// Fallback cleanup of thread in case forever loop was exited accidentally
	vTaskDelete(LCD_Display_Task_Handler);
}

void Initialisation_Task (void *argument){
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// LCD Setup BEGIN
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Start Timer 1 before initialising the LCD1602 module
	HAL_TIM_Base_Start(&htim1);
	// Initialise the LCD1602 module
	LCD_Initialise();
	// Char pointer for string to write to UART
	char *str;

	// Allocate memory in the 'Portable Layer' of the FreeRTOS
	//	- Returns a pointer of type void which can be cast into a pointer of any form
	//	- NB: Memory must be allocated like this otherwise sprintf will cause a hard fault error
	str = pvPortMalloc(sizeof(char) * 50);

	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	// Place the Cursor at (0,0)
	LCD_PlaceCursor(0, 0);
	// Display a message to show that the LCD1602 module has been initialised
	LCD_SendString("LCD Initialised");
	//  HAL_Delay(3000);
	// Convert ms delay to ticks
	const TickType_t tickDelay = pdMS_TO_TICKS(3000);
	vTaskDelay(tickDelay);
	LCD_Clear();
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// LCD Setup END
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// MPU6050 Setup BEGIN
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Initialise the MPU6050 IMU
	MPU6050_Initialisation_Status status = MPU6050_Init();
	// Error Handling
	switch(status){
	  case INIT_SUCCESS:
		  LCD_SendString("MPU6050 Ready!");
		  vTaskDelay(pdMS_TO_TICKS(3000));
		  LCD_Clear();
		  strcpy(str, "Successful Initialisation of MPU6050!\n");
		  break;
	  case WRONG_DEVICE:
		  strcpy(str, "Wrong device found!\n");
		  break;
	  case DEVICE_NOT_FOUND:
		  strcpy(str, "Error finding device!\n");
		  break;
	  case DEVICE_ASLEEP:
		  strcpy(str, "Error waking up the device!\n");
		  break;
	  case ERROR_SMPLRT_DIV:
		  strcpy(str, "Error setting the Sample Rate Divider!\n");
		  break;
	  case ERROR_GYRO_CONFIG:
		  strcpy(str, "Error configuring the gyroscope full scale range!\n");
		  break;
	  case ERROR_ACC_CONFIG:
		  strcpy(str, "Error configuring the accelerometer full scale range!\n");
		  break;
	  default:
		  strcpy(str, "Unknown error case!\n");
		  break;
	};
	// Transmit the status of the MPU6050 initialisation
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// MPU6050 Setup END
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Create Tasks
	// BaseType_t xTaskCreate(
	  //  TaskFunction_t pvTaskCode, - Pointer to the task entry function
	  //  const char * const pcName, - Descriptive name for the task
	  //  const configSTACK_DEPTH_TYPE uxStackDepth, - Stack depth in words to allocate for use as task's stack
	  //  void *pvParameters, - value passed as parameter to the created task
	  //  UBaseType_t uxPriority, - priority at which the created task will execute
	  //  TaskHandle_t *pxCreatedTask - used to pass a handle to the created task, optional and can be NULL
	//);
	BaseType_t xReturnValue; // Return value for task creation
	// IMU_Measure_Task Creation
	xReturnValue = xTaskCreate(IMU_Measure_Task, "IMU_Measure", 300, NULL, 2, &IMU_Measure_Task_Handler);
	if(xReturnValue == pdPASS){
		strcpy(str, "IMU_Measure task was successfully created!\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	} else {
		strcpy(str, "Failed to create IMU_Measure task!\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	}
	// LCD_Display_Task Creation
	xReturnValue = xTaskCreate(LCD_Display_Task, "LCD_Display", 300, NULL, 3, &LCD_Display_Task_Handler);
	if(xReturnValue == pdPASS){
		strcpy(str, "LCD_Display task was successfully created!\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	} else {
		strcpy(str, "Failed to create LCD_Display task!\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	}
	// UART_Display_Task Creation
	//  xReturnValue = xTaskCreate(UART_Display_Task, "UART_Display", 128, NULL, 2, &UART_Display_Task_Handler);
	//  if(xReturnValue == pdPASS){
	//	  char *str = "UART_Display task was successfully created!\n\n";
	//	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	//  } else {
	//	  char *str = "Failed to create UART_Display task!\n\n";
	//	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	//  }
	// Freeing memory
	vPortFree(str); // UART string

	// Delete initialisation task as it is no longer needed
	vTaskDelete(Initialisation_Task_Handler);
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// FreeRTOS Task Definitions END
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* USER CODE END 4 */
/* USER CODE BEGIN 5 */
///* USER CODE BEGIN Header_startUARTTask */
///**
//* @brief Function implementing the uartTask thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_startUARTTask */
//void startUARTTask(void const * argument)
//{
//  /* USER CODE BEGIN startUARTTask */
//  /* Infinite loop */
//  for(;;)
//  {
//	// Transmit the message(data or error) in blocking mode
//	// 	- Casting buffer as the function requires pointer to char array
//	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//
//    osDelay(1000);
//  }
//  // Fallback cleanup of thread in case forever loop was exited accidentally
//  osThreadTerminate(NULL);
//  /* USER CODE END startUARTTask */
//}
/* USER CODE END 5 */
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
