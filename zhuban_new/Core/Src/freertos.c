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
#include "tim.h"
#include "queue.h"
#include "adc.h"
#include "string.h"
#include "usart.h"
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
//鍓嶅洓璺负娓╁害閲囬泦锛屽悗鍥涜矾涓虹數娴侀噰锟??
extern  uint32_t adc1_value[ADC1_BUFFER_SIZE];
extern  uint8_t aRxBuffer[RXBUFFERSIZE];
//鐢垫祦閲囬泦
extern  uint32_t adc3_value[ADC3_BUFFER_SIZE];
extern  float adc1Average[ADC1_CHANNEL_COUNT];
extern  float adc3Average[ADC3_CHANNEL_COUNT];
 uint32_t Freq1 = 0, Freq2 = 0;
 
 
/* USER CODE END Variables */
/* Definitions for FINTask */
osThreadId_t FINTaskHandle;
const osThreadAttr_t FINTask_attributes = {
  .name = "FINTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AudioTask */
osThreadId_t AudioTaskHandle;
const osThreadAttr_t AudioTask_attributes = {
  .name = "AudioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FINQueue */
osMessageQueueId_t FINQueueHandle;
const osMessageQueueAttr_t FINQueue_attributes = {
  .name = "FINQueue"
};
/* Definitions for ADCQueue */
osMessageQueueId_t ADCQueueHandle;
const osMessageQueueAttr_t ADCQueue_attributes = {
  .name = "ADCQueue"
};
/* Definitions for AudioQueue */
osMessageQueueId_t AudioQueueHandle;
const osMessageQueueAttr_t AudioQueue_attributes = {
  .name = "AudioQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartFINTask(void *argument);
void StartADCTask(void *argument);
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

  /* Create the queue(s) */
  /* creation of FINQueue */
  FINQueueHandle = osMessageQueueNew (2, sizeof(uint16_t), &FINQueue_attributes);

  /* creation of ADCQueue */
  ADCQueueHandle = osMessageQueueNew (2, sizeof(uint32_t), &ADCQueue_attributes);

  /* creation of AudioQueue */
  AudioQueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &AudioQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of FINTask */
  FINTaskHandle = osThreadNew(StartFINTask, NULL, &FINTask_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(StartADCTask, NULL, &ADCTask_attributes);

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
  uint16_t GPIO_Pin;
  uint32_t start_time_freq1 = 0, start_time_freq2 = 0;
  int state_freq1 = 0, state_freq2 = 0;
  
  /* Infinite loop */
  for(;;)
  {
    if(xQueueReceive(FINQueueHandle,&GPIO_Pin, portMAX_DELAY)==pdPASS){
      if (GPIO_Pin == GPIO_PIN_6)
      {
        if (state_freq1 == 0)
        {
          start_time_freq1 = __HAL_TIM_GET_COUNTER(&htim2); // 鑾峰彇璧凤拷?锟芥椂锟???
          state_freq1 = 1;
        }
        else
        {
          uint32_t end_time_freq1 = __HAL_TIM_GET_COUNTER(&htim2); // 鑾峰彇缁撴潫鏃堕棿
          uint32_t duration;
          if (end_time_freq1 > start_time_freq1)
          {
            duration = end_time_freq1 - start_time_freq1; // 璁＄畻鍛ㄦ湡
          }
          else
          {
            duration = 99999 - start_time_freq1 + end_time_freq1; // 璁＄畻鍛ㄦ湡
          }
          Freq1 = 1000000 / duration; // 璁＄畻棰戠巼
          state_freq1 = 0;
        }
      }
      else if (GPIO_Pin == GPIO_PIN_7)
      {
        if (state_freq2 == 0)
        {
          start_time_freq2 = __HAL_TIM_GET_COUNTER(&htim2); // 鑾峰彇璧凤拷?锟芥椂锟???
          state_freq2 = 1;
        }
        else
        {
          uint32_t end_time_freq2 = __HAL_TIM_GET_COUNTER(&htim2); // 鑾峰彇缁撴潫鏃堕棿
          uint32_t duration;
          if (end_time_freq2 > start_time_freq2)
          {
            duration = end_time_freq2 - start_time_freq2; // 璁＄畻鍛ㄦ湡
          }
          else
          {
            duration = 99999 - start_time_freq2 + end_time_freq2; // 璁＄畻鍛ㄦ湡
          }
          Freq2 = 1000000 / duration; // 璁＄畻棰戠巼
          state_freq2 = 0;
        }
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartFINTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
void StartADCTask(void *argument)
{
  /* USER CODE BEGIN StartADCTask */
//  ADCMessage adcMessage;

//	uint32_t adc1_sum[ADC1_CHANNEL_COUNT] = {0};
  /* Infinite loop */
  for(;;)
  {

			CalculateAverage(adc1_value, adc1Average, ADC1_CHANNEL_COUNT, SAMPLES_PER_CHANNEL);
			CalculateAverage(adc3_value, adc3Average, ADC3_CHANNEL_COUNT, SAMPLES_PER_CHANNEL);

    osDelay(1);
  }
  /* USER CODE END StartADCTask */
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
#if 0

		Uart_XFS_broadcast("[f0][s0][i1]嘀![s5][f1]前部运输机准备启动,请工作人员注意");
//		xQueueSend(AudioQueueHandle, &aRxBuffer[0],200);
		HAL_Delay(50);
	if(xQueueReceive(AudioQueueHandle,&aRxBuffer, portMAX_DELAY)==pdPASS){
		if(aRxBuffer[0]==0x4F){
				Uart_XFS_broadcast("查询检测");
			if(aRxBuffer[0]==0x4F){
				break;
			}
		}

//		Uart_XFS_broadcast("查询检测");
//		HAL_Delay(3500);
	}
#endif
	
#if 1
		Uart_XFS_broadcast("[f0][s0][i1]嘀![s5][f1]前部运输机准备启动,请工作人员注意");
		
		HAL_Delay(6000);
		Uart_XFS_broadcast("查询检测");
		HAL_Delay(3500);
	
#endif
    osDelay(1);
  }
  /* USER CODE END StartAudioTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 灏嗕腑鏂俊鍙峰彂閫佸埌闃熷垪 */
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){
    xQueueSendFromISR(FINQueueHandle, &GPIO_Pin, &xHigherPriorityTaskWoken);
	
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}




/* USER CODE END Application */

