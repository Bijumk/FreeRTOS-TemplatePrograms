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
#define DEMO_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE * 2)
#define DEMO_TASK_PRIORITY_LOW      (tskIDLE_PRIORITY + 1)
#define DEMO_TASK_PRIORITY_NORMAL   (tskIDLE_PRIORITY + 2)
#define DEMO_TASK_PRIORITY_HIGH     (tskIDLE_PRIORITY + 3)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
/* Queue handles */
QueueHandle_t xSimpleQueue;

/* Task handles */
TaskHandle_t xSenderTaskHandle = NULL;
TaskHandle_t xReceiverTaskHandle = NULL;
TaskHandle_t xButtonTaskHandle = NULL;

/* Counters */
static volatile uint32_t ulMessagesSent = 0;
static volatile uint32_t ulMessagesReceived = 0;

/* UART handle */
extern UART_HandleTypeDef huart1;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* Function prototypes */
static void SenderTask(void *pvParameters);
static void ReceiverTask(void *pvParameters);
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
  xSimpleQueue = xQueueCreate(5, sizeof(uint32_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  if (xSimpleQueue != NULL) {
         printf("Queues created successfully");
         printf("Simple Queue: 5 items of %u bytes each", sizeof(uint32_t));

         /* Create tasks */
         xTaskCreate(SenderTask, "Sender", DEMO_TASK_STACK_SIZE, NULL,
                     DEMO_TASK_PRIORITY_NORMAL, &xSenderTaskHandle);

         xTaskCreate(ReceiverTask, "Receiver", DEMO_TASK_STACK_SIZE, NULL,
                     DEMO_TASK_PRIORITY_NORMAL, &xReceiverTaskHandle);

         xTaskCreate(ButtonTask, "Button", DEMO_TASK_STACK_SIZE, NULL,
                     DEMO_TASK_PRIORITY_LOW, &xButtonTaskHandle);

         printf("All tasks created. Starting scheduler...");

         /* Start the scheduler */
         vTaskStartScheduler();
     } else {
         printf("Failed to create queues. Out of memory?");
     }
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
 * @brief Sender Task - Sends simple data to queue
 * @param pvParameters: Task parameters (unused)
 */
static void SenderTask(void *pvParameters)
{
    uint32_t ulValueToSend = 0;

    printf("Sender Task started");

    for (;;) {
        /* Send incrementing values to the queue */
        ulValueToSend++;

        if (xQueueSend(xSimpleQueue, &ulValueToSend, pdMS_TO_TICKS(100)) == pdPASS) {
            ulMessagesSent++;
            printf("Sender: Sent value %lu (Messages sent: %lu)",
                       ulValueToSend, ulMessagesSent);

            /* Toggle green LED to show sending activity */
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
        } else {
            printf("Sender: Failed to send value %lu (Queue full?)", ulValueToSend);
        }


        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

/**
 * @brief Receiver Task - Receives simple data from queue
 * @param pvParameters: Task parameters (unused)
 */
static void ReceiverTask(void *pvParameters)
{
    uint32_t ulReceivedValue;

    printf("Receiver Task started - waiting for messages...");

    for (;;) {
        /* Wait for data to arrive on the queue */
        if (xQueueReceive(xSimpleQueue, &ulReceivedValue, portMAX_DELAY) == pdPASS) {
            ulMessagesReceived++;
            printf("Receiver: Received value %lu (Messages received: %lu)",
                       ulReceivedValue, ulMessagesReceived);

            /* Toggle red LED to show receiving activity */
            /* TODO */

            /* Simulate processing time */
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}
/**
 * @brief Button Task - Shows queue status when button is pressed
 * @param pvParameters: Task parameters (unused)
 */
static void ButtonTask(void *pvParameters)
{
    printf("Button Task started - Press button for queue status");

    for (;;) {
        /* Check if button is pressed */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
            /* Display queue status */
            printf("=== Queue Status ===");
            printf("Simple Queue: %lu items", uxQueueMessagesWaiting(xSimpleQueue));

            printf("Simple Queue free spaces: %lu", uxQueueSpacesAvailable(xSimpleQueue));


            /* Wait for button release */
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
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
	  vTaskDelay(pdMS_TO_TICKS(1));
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
