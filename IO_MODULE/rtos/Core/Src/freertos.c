/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "BS.h"
#include "CXxEN.h"
#include "CXxIN.h"
#include "input.h"
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
/* Definitions for CX1ENTask */
osThreadId_t CX1ENTaskHandle;
const osThreadAttr_t CX1ENTask_attributes = {
  .name = "CX1ENTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CX2ENTask */
osThreadId_t CX2ENTaskHandle;
const osThreadAttr_t CX2ENTask_attributes = {
  .name = "CX2ENTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Term_IC */
osThreadId_t Term_ICHandle;
const osThreadAttr_t Term_IC_attributes = {
  .name = "Term_IC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Cpu_IC */
osThreadId_t Cpu_ICHandle;
const osThreadAttr_t Cpu_IC_attributes = {
  .name = "Cpu_IC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCX1ENTask(void *argument);
void StartLEDTask(void *argument);
void StartCX2ENTask(void *argument);
void StartTerm_IC(void *argument);
void StartCpu_IC(void *argument);

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
  /* creation of CX1ENTask */
  CX1ENTaskHandle = osThreadNew(StartCX1ENTask, NULL, &CX1ENTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of CX2ENTask */
  CX2ENTaskHandle = osThreadNew(StartCX2ENTask, NULL, &CX2ENTask_attributes);

  /* creation of Term_IC */
  Term_ICHandle = osThreadNew(StartTerm_IC, NULL, &Term_IC_attributes);

  /* creation of Cpu_IC */
  Cpu_ICHandle = osThreadNew(StartCpu_IC, NULL, &Cpu_IC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCX1ENTask */
/**
  * @brief  Function implementing the CX1ENTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCX1ENTask */
void StartCX1ENTask(void *argument)
{
  /* USER CODE BEGIN StartCX1ENTask */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(MCU_CX1P_GPIO_Port,MCU_CX1P_Pin);
    osDelay(5);
  }
  /* USER CODE END StartCX1ENTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		osDelay(500);
    
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartCX2ENTask */
/**
* @brief Function implementing the CX2ENTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCX2ENTask */
void StartCX2ENTask(void *argument)
{
  /* USER CODE BEGIN StartCX2ENTask */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(MCU_CX2P_GPIO_Port,MCU_CX2P_Pin);
    osDelay(10);
  }
  /* USER CODE END StartCX2ENTask */
}

/* USER CODE BEGIN Header_StartTerm_IC */
/**
* @brief Function implementing the Term_IC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTerm_IC */
void StartTerm_IC(void *argument)
{
  /* USER CODE BEGIN StartTerm_IC */
  /* Infinite loop */
  for(;;)
  {
		TERM_SIGNAL_IC(); 
    osDelay(10);
  }
  /* USER CODE END StartTerm_IC */
}

/* USER CODE BEGIN Header_StartCpu_IC */
/**
* @brief Function implementing the Cpu_IC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCpu_IC */
void StartCpu_IC(void *argument)
{
  /* USER CODE BEGIN StartCpu_IC */
  /* Infinite loop */
  for(;;)
  {
		CPU_SIGNAL_IC();
    osDelay(10);
  }
  /* USER CODE END StartCpu_IC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

