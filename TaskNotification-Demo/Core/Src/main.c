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
/* Notification bit definitions */
#define NOTIFICATION_BIT_0    (1UL << 0)  /* Button pressed */
#define NOTIFICATION_BIT_1    (1UL << 1)  /* Timer event */
#define NOTIFICATION_BIT_2    (1UL << 2)  /* Data ready */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
/* Task handles */
TaskHandle_t xNotifierTaskHandle = NULL;
TaskHandle_t xWaiterTaskHandle = NULL;
TaskHandle_t xButtonTaskHandle = NULL;
TaskHandle_t xLcdUpdateTaskHandle = NULL;


/* Counters */
static volatile uint32_t ulNotificationsSent = 0;
static volatile uint32_t ulNotificationsReceived = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* Function prototypes */
static void NotifierTask(void *pvParameters);
static void WaiterTask(void *pvParameters);
static void ButtonTask(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *data, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
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
  printf("=== FreeRTOS Task Notification Demonstration ===\r\n");
      printf("STM32F429I Discovery Board\r\n");
      printf("Task notifications are lightweight and fast!\r\n");

      /* Create tasks */
      xTaskCreate(
          NotifierTask,           /* Task function */
          "Notifier",             /* Task name */
          configMINIMAL_STACK_SIZE, /* Stack size */
          NULL,                   /* Parameters to pass to the task */
          tskIDLE_PRIORITY + 1,   /* Task priority */
          &xNotifierTaskHandle    /* Task handle */
      );

      xTaskCreate(
          WaiterTask,             /* Task function */
          "Waiter",               /* Task name */
          configMINIMAL_STACK_SIZE, /* Stack size */
          NULL,                   /* Parameters to pass to the task */
          tskIDLE_PRIORITY + 2,   /* Higher priority */
          &xWaiterTaskHandle      /* Task handle */
      );

      xTaskCreate(
          ButtonTask,             /* Task function */
          "Button",               /* Task name */
          configMINIMAL_STACK_SIZE, /* Stack size */
          NULL,                   /* Parameters to pass to the task */
          tskIDLE_PRIORITY + 2,   /* Higher priority */
          &xButtonTaskHandle      /* Task handle */
      );


      printf("All tasks created. Starting scheduler...\r\n");

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Notifier Task - Sends various types of notifications
 * @param pvParameters: Task parameters (unused)
 */
static void NotifierTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */
    uint32_t ulNotificationType = 0;

    printf("Notifier Task: Started\r\n");

    for (;;) {
        /* Cycle through different notification types */
        ulNotificationType = (ulNotificationType + 1) % 3;

        switch (ulNotificationType) {
            case 0:
                /* Send simple notification (binary semaphore style) */
                if (xTaskNotifyGive(xWaiterTaskHandle) == pdPASS) {
                    ulNotificationsSent++;
                    printf("Notifier: Sent simple notification (count style)\r\n");
                }
                break;

            case 1:
                /* Send bit notification - Timer event */
                if (xTaskNotify(xWaiterTaskHandle, NOTIFICATION_BIT_1, eSetBits) == pdPASS) {
                    ulNotificationsSent++;
                    printf("Notifier: Sent timer event notification (bit 1)\r\n");
                }
                break;

            case 2:
                /* Send bit notification - Data ready */
                if (xTaskNotify(xWaiterTaskHandle, NOTIFICATION_BIT_2, eSetBits) == pdPASS) {
                    ulNotificationsSent++;
                    printf("Notifier: Sent data ready notification (bit 2)\r\n");
                }
                break;
        }

        /* Toggle green LED to show activity */
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * @brief Waiter Task - Waits for and processes notifications
 * @param pvParameters: Task parameters (unused)
 */
static void WaiterTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */
    uint32_t ulNotificationValue;

    printf("Waiter Task: Started - waiting for notifications...\r\n");

    for (;;) {
        /* Wait for any notification with timeout */
        if (xTaskNotifyWait(0x00,           /* Don't clear bits on entry */
                           0xFFFFFFFF,      /* Clear all bits on exit */
                           &ulNotificationValue,
                           pdMS_TO_TICKS(5000)) == pdPASS) {

            ulNotificationsReceived++;

            /* Check if it was a simple notification (counting semaphore style) */
            if (ulNotificationValue > 0) {
                printf("Waiter: Received notification value: %lu\r\n", ulNotificationValue);

                /* Check specific bits */
                if (ulNotificationValue & NOTIFICATION_BIT_0) {
                    printf("Waiter: Button press detected!\r\n");
                }
                if (ulNotificationValue & NOTIFICATION_BIT_1) {
                    printf("Waiter: Timer event processed\r\n");
                }
                if (ulNotificationValue & NOTIFICATION_BIT_2) {
                    printf("Waiter: Data ready processed\r\n");
                }
            }

            /* Toggle red LED to show processing */
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);

        } else {
            printf("Waiter: Timeout waiting for notification\r\n");
        }
    }
}

/**
 * @brief Button Task - Sends notification when button is pressed
 * @param pvParameters: Task parameters (unused)
 */
static void ButtonTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */

    printf("Button Task: Started - Press button to send notification\r\n");

    for (;;) {
        /* Check if button is pressed */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
            /* Send button press notification */
            if (xTaskNotify(xWaiterTaskHandle, NOTIFICATION_BIT_0, eSetBits) == pdPASS) {
                ulNotificationsSent++;
                printf("Button: Sent button press notification (bit 0)\r\n");
            }

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
