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
/* Definitions for FINTask */
osThreadId_t FINTaskHandle;
const osThreadAttr_t FINTask_attributes = {
  .name = "FINTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PT100_ADCTask */
osThreadId_t PT100_ADCTaskHandle;
const osThreadAttr_t PT100_ADCTask_attributes = {
  .name = "PT100_ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CI_ADCTask */
osThreadId_t CI_ADCTaskHandle;
const osThreadAttr_t CI_ADCTask_attributes = {
  .name = "CI_ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AudioTask */
osThreadId_t AudioTaskHandle;
const osThreadAttr_t AudioTask_attributes = {
  .name = "AudioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartFINTask(void *argument);
void StartPT100_ADCTask(void *argument);
void StartCI_ADCTask(void *argument);
void StartAudioTask(void *argument);

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
  /* creation of FINTask */
  FINTaskHandle = osThreadNew(StartFINTask, NULL, &FINTask_attributes);

  /* creation of PT100_ADCTask */
  PT100_ADCTaskHandle = osThreadNew(StartPT100_ADCTask, NULL, &PT100_ADCTask_attributes);

  /* creation of CI_ADCTask */
  CI_ADCTaskHandle = osThreadNew(StartCI_ADCTask, NULL, &CI_ADCTask_attributes);

  /* creation of AudioTask */
  AudioTaskHandle = osThreadNew(StartAudioTask, NULL, &AudioTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartFINTask */
/**
  * @brief  Function implementing the FINTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFINTask */
void StartFINTask(void *argument)
{
  /* USER CODE BEGIN StartFINTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFINTask */
}

/* USER CODE BEGIN Header_StartPT100_ADCTask */
/**
* @brief Function implementing the PT100_ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPT100_ADCTask */
void StartPT100_ADCTask(void *argument)
{
  /* USER CODE BEGIN StartPT100_ADCTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPT100_ADCTask */
}

/* USER CODE BEGIN Header_StartCI_ADCTask */
/**
* @brief Function implementing the CI_ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCI_ADCTask */
void StartCI_ADCTask(void *argument)
{
  /* USER CODE BEGIN StartCI_ADCTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCI_ADCTask */
}

/* USER CODE BEGIN Header_StartAudioTask */
/**
* @brief Function implementing the AudioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void *argument)
{
  /* USER CODE BEGIN StartAudioTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAudioTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

