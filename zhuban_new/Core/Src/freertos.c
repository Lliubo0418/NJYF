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
//前四路为温度采集，后四路为电流采集
extern  uint32_t adc1_value[ADC1_BUFFER_SIZE];
//电流采集
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
  .priority = (osPriority_t) osPriorityLow,
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
//  ADCQueueHandle = osMessageQueueNew (8, sizeof(uint32_t), &ADCQueue_attributes);

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
          start_time_freq1 = __HAL_TIM_GET_COUNTER(&htim2); // 获取起始时间
          state_freq1 = 1;
        }
        else
        {
          uint32_t end_time_freq1 = __HAL_TIM_GET_COUNTER(&htim2); // 获取结束时间
          uint32_t duration;
          if (end_time_freq1 > start_time_freq1)
          {
            duration = end_time_freq1 - start_time_freq1; // 计算周期
          }
          else
          {
            duration = 99999 - start_time_freq1 + end_time_freq1; // 计算周期
          }
          Freq1 = 1000000 / duration; // 计算频率
          state_freq1 = 0;
        }
      }
      else if (GPIO_Pin == GPIO_PIN_7)
      {
        if (state_freq2 == 0)
        {
          start_time_freq2 = __HAL_TIM_GET_COUNTER(&htim2); // 获取起始时间
          state_freq2 = 1;
        }
        else
        {
          uint32_t end_time_freq2 = __HAL_TIM_GET_COUNTER(&htim2); // 获取结束时间
          uint32_t duration;
          if (end_time_freq2 > start_time_freq2)
          {
            duration = end_time_freq2 - start_time_freq2; // 计算周期
          }
          else
          {
            duration = 99999 - start_time_freq2 + end_time_freq2; // 计算周期
          }
          Freq2 = 1000000 / duration; // 计算频率
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
			// 循环遍历每个通道
//    for (int channel = 0; channel < ADC1_CHANNEL_COUNT; channel++)
//    {
//        // 计算当前通道所有样本的总和
//        for (int sample = 0; sample < SAMPLES_PER_CHANNEL; sample++)
//        {
//            adc1_sum[channel] += adc1_value[(ADC1_CHANNEL_COUNT*sample-1)+channel];
//        }

//        // 计算当前通道的平均值
//        adc1Average[channel] = (float)adc1_sum[channel] / SAMPLES_PER_CHANNEL;
//    }
//    if (xQueueReceive(ADCQueueHandle, &adcMessage, portMAX_DELAY) == pdPASS)
//    {
//      if (adcMessage.channelCount == ADC1_CHANNEL_COUNT)
//      {
//        CalculateAverage(adcMessage.buffer, adc1Average, ADC1_CHANNEL_COUNT, SAMPLES_PER_CHANNEL);
//        // 处理adc1Average数据
//      }
//      else if (adcMessage.channelCount == ADC3_CHANNEL_COUNT)
//      {
//        CalculateAverage(adcMessage.buffer, adc3Average, ADC3_CHANNEL_COUNT, SAMPLES_PER_CHANNEL);
//        // 处理adc3Average数据
//      }
//    }
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
    osDelay(1);
  }
  /* USER CODE END StartAudioTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 将中断信号发送到队列 */
    xQueueSendFromISR(FINQueueHandle, &GPIO_Pin, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ADC转换完成回调函数 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    ADCMessage adcMessage;
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		memset(&adcMessage, 0, sizeof(ADCMessage));
//	
//    if (hadc->Instance == ADC1)
//    {
//        adcMessage.buffer = adc1_value;
//        adcMessage.channelCount = ADC1_CHANNEL_COUNT;
//        xQueueSendFromISR(ADCQueueHandle, &adcMessage, NULL);
//    }
//    else if (hadc->Instance == ADC3)
//    {
//        adcMessage.buffer = adc3_value;
//        adcMessage.channelCount = ADC3_CHANNEL_COUNT;
//        xQueueSendFromISR(ADCQueueHandle, &adcMessage, NULL);
//    }
//}
/* USER CODE END Application */

