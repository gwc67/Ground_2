/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_LX */
osThreadId_t Task_LXHandle;
const osThreadAttr_t Task_LX_attributes = {
  .name = "Task_LX",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Ground_UART */
osThreadId_t Ground_UARTHandle;
const osThreadAttr_t Ground_UART_attributes = {
  .name = "Ground_UART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Touch */
osThreadId_t UART_TouchHandle;
const osThreadAttr_t UART_Touch_attributes = {
  .name = "UART_Touch",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void APPTask_LX(void *argument);
void Ground_DT_UART(void *argument);
void UART_DT_Touch(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of Task_LX */
  Task_LXHandle = osThreadNew(APPTask_LX, NULL, &Task_LX_attributes);

  /* creation of Ground_UART */
  Ground_UARTHandle = osThreadNew(Ground_DT_UART, NULL, &Ground_UART_attributes);

  /* creation of UART_Touch */
  UART_TouchHandle = osThreadNew(UART_DT_Touch, NULL, &UART_Touch_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_APPTask_LX */
/**
  * @brief  Function implementing the Task_LX thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_APPTask_LX */
__weak void APPTask_LX(void *argument)
{
  /* USER CODE BEGIN APPTask_LX */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END APPTask_LX */
}

/* USER CODE BEGIN Header_Ground_DT_UART */
/**
* @brief Function implementing the Ground_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ground_DT_UART */
__weak void Ground_DT_UART(void *argument)
{
  /* USER CODE BEGIN Ground_DT_UART */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ground_DT_UART */
}

/* USER CODE BEGIN Header_UART_DT_Touch */
/**
* @brief Function implementing the UART_Touch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_DT_Touch */
__weak void UART_DT_Touch(void *argument)
{
  /* USER CODE BEGIN UART_DT_Touch */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART_DT_Touch */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

