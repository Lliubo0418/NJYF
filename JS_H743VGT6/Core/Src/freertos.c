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
#include "gpio.h"
#include "limits.h"
#include "usart.h"
#include "stdio.h"
#include "semphr.h"
#include "portmacro.h"

extern uint32_t Sync_capture_Buf[2]; // 存放计数值
extern uint8_t Sync_Cnt;             // 状态标志位
extern uint32_t Sync_high_time;      // 高电平时间

extern uint16_t position_xor;

uint16_t semavalue = 0;
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
/* Definitions for SyncTask */
osThreadId_t SyncTaskHandle;
const osThreadAttr_t SyncTask_attributes = {
  .name = "SyncTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ICTask */
osThreadId_t ICTaskHandle;
const osThreadAttr_t ICTask_attributes = {
  .name = "ICTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Rec_SwitchTask */
osThreadId_t Rec_SwitchTaskHandle;
const osThreadAttr_t Rec_SwitchTask_attributes = {
  .name = "Rec_SwitchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for AlarmTask */
osThreadId_t AlarmTaskHandle;
const osThreadAttr_t AlarmTask_attributes = {
  .name = "AlarmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for HolesCountingSem */
osSemaphoreId_t HolesCountingSemHandle;
const osSemaphoreAttr_t HolesCountingSem_attributes = {
  .name = "HolesCountingSem"
};
/* Definitions for PositionEvent1 */
osEventFlagsId_t PositionEvent1Handle;
const osEventFlagsAttr_t PositionEvent1_attributes = {
  .name = "PositionEvent1"
};
/* Definitions for PositionEvent2 */
osEventFlagsId_t PositionEvent2Handle;
const osEventFlagsAttr_t PositionEvent2_attributes = {
  .name = "PositionEvent2"
};
/* Definitions for PositionEvent3 */
osEventFlagsId_t PositionEvent3Handle;
const osEventFlagsAttr_t PositionEvent3_attributes = {
  .name = "PositionEvent3"
};
/* Definitions for PositionEvent4 */
osEventFlagsId_t PositionEvent4Handle;
const osEventFlagsAttr_t PositionEvent4_attributes = {
  .name = "PositionEvent4"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSyncTask(void *argument);
void StartICTask(void *argument);
void StartRec_SwitchTask(void *argument);
void StartAlarmTask(void *argument);
void StartledTask(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of HolesCountingSem */
  HolesCountingSemHandle = osSemaphoreNew(16, 0, &HolesCountingSem_attributes);

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
  /* creation of SyncTask */
  SyncTaskHandle = osThreadNew(StartSyncTask, NULL, &SyncTask_attributes);

  /* creation of ICTask */
  ICTaskHandle = osThreadNew(StartICTask, NULL, &ICTask_attributes);

  /* creation of Rec_SwitchTask */
  Rec_SwitchTaskHandle = osThreadNew(StartRec_SwitchTask, NULL, &Rec_SwitchTask_attributes);

  /* creation of AlarmTask */
  AlarmTaskHandle = osThreadNew(StartAlarmTask, NULL, &AlarmTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartledTask, NULL, &ledTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of PositionEvent1 */
  PositionEvent1Handle = osEventFlagsNew(&PositionEvent1_attributes);

  /* creation of PositionEvent2 */
  PositionEvent2Handle = osEventFlagsNew(&PositionEvent2_attributes);

  /* creation of PositionEvent3 */
  PositionEvent3Handle = osEventFlagsNew(&PositionEvent3_attributes);

  /* creation of PositionEvent4 */
  PositionEvent4Handle = osEventFlagsNew(&PositionEvent4_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSyncTask */
/**
 * @brief  Function implementing the SyncTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSyncTask */
void StartSyncTask(void *argument)
{
  /* USER CODE BEGIN StartSyncTask */
  /* Infinite loop */
  for (;;)
  {
    switch (Sync_Cnt)
    {
    case 0:
      Sync_Cnt++;
      TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4);
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
      HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4); // 启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
      break;
    case 3:
      if (Sync_capture_Buf[1] < Sync_capture_Buf[0])
      {                                                                       // 不存在先捕获高电平后捕获低电平capture_Buf[1]<capture_Buf[0],肯定是有了溢出
        Sync_high_time = 0x186A0 + Sync_capture_Buf[1] - Sync_capture_Buf[0]; // 100000    0x186A0
      }
      else
      {
        Sync_high_time = Sync_capture_Buf[1] - Sync_capture_Buf[0]; // 高电平时间
      }

      Sync_Cnt = 0;
      if (Sync_high_time == 0x1B)
      {
        Sync_Cnt = 5;
#if 0        
        xTaskNotify((TaskHandle_t)StartICTask, 0x01, eSetValueWithOverwrite);
        xTaskNotify((TaskHandle_t)StartRec_switchTask, 0x01, eSetValueWithOverwrite);
#endif
#if 1
        xTaskNotifyGive((TaskHandle_t)ICTaskHandle);
        xTaskNotifyGive((TaskHandle_t)Rec_SwitchTaskHandle);
#endif
      }
      else
      {
        Sync_Cnt = 0;
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartSyncTask */
}

/* USER CODE BEGIN Header_StartICTask */
/**
 * @brief Function implementing the ICTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartICTask */
void StartICTask(void *argument)
{
  /* USER CODE BEGIN StartICTask */
  BaseType_t ret;
  uint32_t NotifyValue;
  /* Infinite loop */
  for (;;)
  {
#if 0

    ret = xTaskNotifyWait((uint32_t)0x00,
                          (uint32_t)ULONG_MAX,
                          (uint32_t *)&NotifyValue,
                          (TickType_t)portMAX_DELAY);
#endif

#if 1
    ret = ulTaskNotifyTake(pdFALSE, 0);
#endif
    if (ret == pdPASS)
    {
      // LXXX:按照所查资料提示，如果不行那就还是将同步引脚更改为其他引脚，TIM2作为主定时器通过事件触发从定时器同步捕获，硬件层面的同步捕获
      HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);
      HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
    }
    else
    {
      printf("StartICTask任务通知获取失败\r\n");
    }
    osDelay(1);
  }
  /* USER CODE END StartICTask */
}

/* USER CODE BEGIN Header_StartRec_SwitchTask */
/**
 * @brief Function implementing the Rec_SwitchTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRec_SwitchTask */
void StartRec_SwitchTask(void *argument)
{
  /* USER CODE BEGIN StartRec_SwitchTask */
  BaseType_t ret;
  uint32_t NotifyValue;
  /* Infinite loop */
  for (;;)
  {
#if 0   

    ret = xTaskNotifyWait((uint32_t)0x00,
                          (uint32_t)ULONG_MAX,
                          (uint32_t *)&NotifyValue,
                          (TickType_t)portMAX_DELAY);
#endif
#if 1
    ret = ulTaskNotifyTake(pdFALSE, 0);
#endif
    if (ret == pdPASS)
    {
      EN_R1_R2_open();
      EN_R3_R4_open();
      HAL_TIM_Base_Start_IT(&htim6); // start timer for R1_channel switch
    }
    else
    {
      printf("StartRec_switchTask任务通知获取失败\r\n");
    }
    osDelay(1);
  }
  /* USER CODE END StartRec_SwitchTask */
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
 * @brief Function implementing the AlarmTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void *argument)
{
  /* USER CODE BEGIN StartAlarmTask */
  
  /* Infinite loop */
  for (;;)
  {
    #if 0
    BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreTakeFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
    semavalue = uxSemaphoreGetCount(HolesCountingSemHandle);
    #endif
    for(int i = 0; i <sizeof(position_xor);i++){
      if(((position_xor>>i)&1)==1){
      semavalue++;
      }
    }
    if (semavalue > 0)
    {
      Holes_output_alarm_Open();
      HAL_Delay(500);
      Holes_output_alarm_Close();
    }
    osDelay(1);
  }
  /* USER CODE END StartAlarmTask */
}

/* USER CODE BEGIN Header_StartledTask */
/**
 * @brief Function implementing the ledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartledTask */
void StartledTask(void *argument)
{
  /* USER CODE BEGIN StartledTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartledTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

