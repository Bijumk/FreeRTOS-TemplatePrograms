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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
/* Task handles */
TaskHandle_t xSuspendableTaskHandle = NULL;
TaskHandle_t xDeletableTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;

/* Counters */
static volatile uint32_t ulSuspendableTaskCounter = 0;
static volatile uint32_t ulDeletableTaskCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* Function prototypes */
static void SuspendableTask(void *pvParameters);
static void DeletableTask(void *pvParameters);
static void ControlTask(void *pvParameters);
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
  printf("=== FreeRTOS Task Control Demonstration ===\r\n");
      printf("STM32F429I Discovery Board\r\n");
      printf("FreeRTOS Version: %s\r\n", tskKERNEL_VERSION_NUMBER);

      /* Create tasks */
      xTaskCreate(
          SuspendableTask,        /* Task function */
          "Suspendable",          /* Task name */
          configMINIMAL_STACK_SIZE, /* Stack size */
          NULL,                   /* Parameters to pass to the task */
          tskIDLE_PRIORITY + 1,   /* Task priority */
          &xSuspendableTaskHandle /* Task handle */
      );

      xTaskCreate(
          DeletableTask,          /* Task function */
          "Deletable",            /* Task name */
          configMINIMAL_STACK_SIZE, /* Stack size */
          NULL,                   /* Parameters to pass to the task */
          tskIDLE_PRIORITY + 1,   /* Task priority */
          &xDeletableTaskHandle   /* Task handle */
      );

      xTaskCreate(
          ControlTask,            /* Task function */
          "Control",              /* Task name */
          configMINIMAL_STACK_SIZE * 2, /* Stack size */
          NULL,                   /* Parameters to pass to the task */
          tskIDLE_PRIORITY + 2,   /* Higher priority for control */
          &xControlTaskHandle     /* Task handle */
      );

      printf("All tasks created successfully. Starting scheduler...\r\n");

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
 * @brief Suspendable Task - Toggles LED and can be suspended/resumed
 * @param pvParameters: Task parameters (unused)
 */
static void SuspendableTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */

    printf("Suspendable Task: Started\r\n");

    for (;;) {
        ulSuspendableTaskCounter++;
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13); /* Toggle Green LED */
        printf("Suspendable Task: Running (Count: %lu)\r\n", ulSuspendableTaskCounter);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Deletable Task - Toggles LED and can be deleted
 * @param pvParameters: Task parameters (unused)
 */
static void DeletableTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */

    printf("Deletable Task: Started\r\n");

    for (;;) {
        ulDeletableTaskCounter++;
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);; /* Toggle Red LED */
        printf("Deletable Task: Running (Count: %lu)\r\n", ulDeletableTaskCounter);
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

/**
 * @brief Control Task - Controls the state of other tasks based on button presses
 * @param pvParameters: Task parameters (unused)
 */
static void ControlTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */
    static uint8_t control_state = 0;

    printf("Control Task: Started. Press button to control tasks.\r\n");

    for (;;) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
            /* Debounce */
            vTaskDelay(pdMS_TO_TICKS(50));
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            control_state++;

            switch (control_state) {
                case 1:
                    if (xSuspendableTaskHandle != NULL) {
                        vTaskSuspend(xSuspendableTaskHandle);
                        printf("Control Task: Suspendable Task SUSPENDED.\r\n");
                    }
                    break;
                case 2:
                    if (xSuspendableTaskHandle != NULL) {
                        vTaskResume(xSuspendableTaskHandle);
                        printf("Control Task: Suspendable Task RESUMED.\r\n");
                    }
                    break;
                case 3:
                    if (xDeletableTaskHandle != NULL) {
                        vTaskDelete(xDeletableTaskHandle);
                        xDeletableTaskHandle = NULL; /* Clear handle after deletion */
                        printf("Control Task: Deletable Task DELETED.\r\n");
                    }
                    break;
                default:
                    printf("Control Task: Resetting demo. Restarting...\r\n");
                    HAL_NVIC_SystemReset(); /* Reset system to restart demo */
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
    osDelay(1);
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
