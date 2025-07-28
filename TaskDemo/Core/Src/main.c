/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEMO_TASK_PRIORITY_LOW      (tskIDLE_PRIORITY + 1)
#define DEMO_TASK_PRIORITY_NORMAL   (tskIDLE_PRIORITY + 2)
#define DEMO_TASK_PRIORITY_HIGH     (tskIDLE_PRIORITY + 3)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
/* Task handles */
TaskHandle_t xHighPriorityTaskHandle = NULL;
TaskHandle_t xMediumPriorityTaskHandle = NULL;
TaskHandle_t xLowPriorityTaskHandle = NULL;
TaskHandle_t xPeriodicTaskHandle = NULL;
TaskHandle_t xButtonTaskHandle = NULL;

/* Thread Local Storage (TLS) index */
static BaseType_t xTLSIndex = 0;

/* Task counters for demonstration */
static uint32_t ulHighPriorityCounter = 0;
static uint32_t ulMediumPriorityCounter = 0;
static uint32_t ulLowPriorityCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void HighPriorityTask(void *pvParameters);
static void MediumPriorityTask(void *pvParameters);
static void LowPriorityTask(void *pvParameters);
static void PeriodicTask(void *pvParameters);
static void ButtonTask(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */

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
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	printf("=== FreeRTOS Tasks Demonstration ===");
	printf("STM32F429I Discovery Board");
	printf("FreeRTOS Version: %s", tskKERNEL_VERSION_NUMBER);

	/* Create tasks with different priorities */
	xTaskCreate(HighPriorityTask, "HighPriTask", configMINIMAL_STACK_SIZE * 2, NULL,
			DEMO_TASK_PRIORITY_HIGH, &xHighPriorityTaskHandle);

	xTaskCreate(MediumPriorityTask, "MedPriTask", configMINIMAL_STACK_SIZE * 2, NULL,
			DEMO_TASK_PRIORITY_NORMAL, &xMediumPriorityTaskHandle);

	xTaskCreate(LowPriorityTask, "LowPriTask", configMINIMAL_STACK_SIZE * 2, NULL,
			DEMO_TASK_PRIORITY_LOW, &xLowPriorityTaskHandle);

	xTaskCreate(PeriodicTask, "PeriodicTask", configMINIMAL_STACK_SIZE * 2, NULL,
			DEMO_TASK_PRIORITY_LOW, &xPeriodicTaskHandle);

	xTaskCreate(ButtonTask, "ButtonTask", configMINIMAL_STACK_SIZE * 2, NULL,
			DEMO_TASK_PRIORITY_NORMAL, &xButtonTaskHandle);

	printf("All tasks created successfully");
	printf("Starting FreeRTOS scheduler...");

	/* Start the scheduler */
	vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief High Priority Task - Demonstrates highest priority task behavior
 * @param pvParameters: Task parameters (unused)
 */
static void HighPriorityTask(void *pvParameters)
{
	uint32_t *pulTaskID;

	/* Allocate memory for TLS data */
	pulTaskID = pvPortMalloc(sizeof(uint32_t));
	if (pulTaskID != NULL) {
		*pulTaskID = 1; /* Task ID for high priority task */
		/* Store task ID in Thread Local Storage */
		vTaskSetThreadLocalStoragePointer(NULL, xTLSIndex, pulTaskID);
	}

	printf("High Priority Task started (Priority: %lu)", uxTaskPriorityGet(NULL));

	for (;;) {
		/* Get TLS data */
		pulTaskID = (uint32_t *)pvTaskGetThreadLocalStoragePointer(NULL, xTLSIndex);

		ulHighPriorityCounter++;

		/* Toggle Green LED to show activity */

		printf("High Priority Task running (Count: %lu, TLS ID: %lu)", ulHighPriorityCounter, pulTaskID ? *pulTaskID : 0);

		/* Update LCD display */

		/* Block for 500ms - demonstrates BLOCKED state */
		vTaskDelay(pdMS_TO_TICKS(500));

		/* Demonstrate task suspension after 10 iterations */
		if (ulHighPriorityCounter >= 10) {
			printf("High Priority Task suspending itself");
			vTaskSuspend(NULL); /* Task enters SUSPENDED state */
		}
	}
}

/**
 * @brief Medium Priority Task - Demonstrates medium priority scheduling
 * @param pvParameters: Task parameters (unused)
 */
static void MediumPriorityTask(void *pvParameters)
{
	uint32_t *pulTaskID;

	/* Allocate memory for TLS data */
	pulTaskID = pvPortMalloc(sizeof(uint32_t));
	if (pulTaskID != NULL) {
		*pulTaskID = 2; /* Task ID for medium priority task */
		vTaskSetThreadLocalStoragePointer(NULL, xTLSIndex, pulTaskID);
	}

	printf("Medium Priority Task started (Priority: %lu)", uxTaskPriorityGet(NULL));

	for (;;) {
		pulTaskID = (uint32_t *)pvTaskGetThreadLocalStoragePointer(NULL, xTLSIndex);

		ulMediumPriorityCounter++;

		printf("Medium Priority Task running (Count: %lu, TLS ID: %lu)",
				ulMediumPriorityCounter, pulTaskID ? *pulTaskID : 0);

		/* This task runs when high priority task is blocked */
		vTaskDelay(pdMS_TO_TICKS(300));

		/* Demonstrate priority change */
		if (ulMediumPriorityCounter == 5) {
			printf("Medium Priority Task changing to low priority");
			vTaskPrioritySet(NULL, DEMO_TASK_PRIORITY_LOW);
		}

		/* Resume high priority task after it suspends */
		if (ulMediumPriorityCounter == 15 && xHighPriorityTaskHandle != NULL) {
			printf("Medium Priority Task resuming High Priority Task");
			vTaskResume(xHighPriorityTaskHandle);
			ulHighPriorityCounter = 0; /* Reset counter */
		}
	}
}

/**
 * @brief Low Priority Task - Demonstrates lowest priority task behavior
 * @param pvParameters: Task parameters (unused)
 */
static void LowPriorityTask(void *pvParameters)
{
	uint32_t *pulTaskID;

	/* Allocate memory for TLS data */
	pulTaskID = pvPortMalloc(sizeof(uint32_t));
	if (pulTaskID != NULL) {
		*pulTaskID = 3; /* Task ID for low priority task */
		vTaskSetThreadLocalStoragePointer(NULL, xTLSIndex, pulTaskID);
	}

	printf("Low Priority Task started (Priority: %lu)", uxTaskPriorityGet(NULL));

	for (;;) {
		pulTaskID = (uint32_t *)pvTaskGetThreadLocalStoragePointer(NULL, xTLSIndex);

		ulLowPriorityCounter++;

		/* Toggle Red LED to show activity */
		/*TODO */

		printf("Low Priority Task running (Count: %lu, TLS ID: %lu)",
				ulLowPriorityCounter, pulTaskID ? *pulTaskID : 0);

		/* This task runs when higher priority tasks are blocked */
		vTaskDelay(pdMS_TO_TICKS(1000));

		/* Demonstrate task deletion after some iterations */
		if (ulLowPriorityCounter >= 20) {
			printf("Low Priority Task deleting itself");
			vTaskDelete(NULL); /* Task is deleted */
		}
	}
}

/**
 * @brief Periodic Task - Demonstrates periodic execution pattern
 * @param pvParameters: Task parameters (unused)
 */
static void PeriodicTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(2000); /* 2 second period */

	/* Initialize the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();

	printf("Periodic Task started (Period: 2000ms)");

	for (;;) {
		/* Wait for the next cycle - demonstrates precise timing */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		printf("Periodic Task tick - System uptime: %lu ms",
				xTaskGetTickCount() * portTICK_PERIOD_MS);

		/* Display system information */
		printf("Free heap: %u bytes, Min free heap: %u bytes",
				xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

		printf("Number of tasks: %lu", uxTaskGetNumberOfTasks());
	}
}

/**
 * @brief Button Task - Demonstrates interrupt-driven task activation
 * @param pvParameters: Task parameters (unused)
 */
static void ButtonTask(void *pvParameters)
{
	printf("Button Task started - Press user button for interaction");

	for (;;) {
		/* Check button state (polling for simplicity) */
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
		{
			printf("Button pressed! Displaying task statistics...");

			/* Display current task information */
			char pcWriteBuffer[500];
			vTaskList(pcWriteBuffer);
			printf("Task List:\r\n%s", pcWriteBuffer);

//			/* Display runtime statistics */
//			vTaskGetRunTimeStats(pcWriteBuffer);
//			printf("Runtime Stats:\r\n%s", pcWriteBuffer);

			/* Wait for button release */
			while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
				vTaskDelay(pdMS_TO_TICKS(10));
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50)); /* Check button every 50ms */
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
	}
  /* USER CODE END 5 */
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
#ifdef USE_FULL_ASSERT
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
