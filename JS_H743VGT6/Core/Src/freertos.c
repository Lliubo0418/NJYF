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

uint8_t Isfirstcirculation = 1;

// 事件组
int32_t position_old = 0;
int32_t position_new = 0;
int32_t position_xor = 0;
uint32_t position_new_plus_old = 0;
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
/* Definitions for Hole_ldentifica */
osThreadId_t Hole_ldentificaHandle;
const osThreadAttr_t Hole_ldentifica_attributes = {
  .name = "Hole_ldentifica",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for HolesCountingSem */
osSemaphoreId_t HolesCountingSemHandle;
const osSemaphoreAttr_t HolesCountingSem_attributes = {
  .name = "HolesCountingSem"
};
/* Definitions for PositionEvent1_MPC1 */
osEventFlagsId_t PositionEvent1_MPC1Handle;
const osEventFlagsAttr_t PositionEvent1_MPC1_attributes = {
  .name = "PositionEvent1_MPC1"
};
/* Definitions for PositionEvent2_MPC1 */
osEventFlagsId_t PositionEvent2_MPC1Handle;
const osEventFlagsAttr_t PositionEvent2_MPC1_attributes = {
  .name = "PositionEvent2_MPC1"
};
/* Definitions for PositionEvent3_MPC1 */
osEventFlagsId_t PositionEvent3_MPC1Handle;
const osEventFlagsAttr_t PositionEvent3_MPC1_attributes = {
  .name = "PositionEvent3_MPC1"
};
/* Definitions for PositionEvent4_MPC1 */
osEventFlagsId_t PositionEvent4_MPC1Handle;
const osEventFlagsAttr_t PositionEvent4_MPC1_attributes = {
  .name = "PositionEvent4_MPC1"
};
/* Definitions for PositionEvent1_MPC2 */
osEventFlagsId_t PositionEvent1_MPC2Handle;
const osEventFlagsAttr_t PositionEvent1_MPC2_attributes = {
  .name = "PositionEvent1_MPC2"
};
/* Definitions for PositionEvent2_MPC2 */
osEventFlagsId_t PositionEvent2_MPC2Handle;
const osEventFlagsAttr_t PositionEvent2_MPC2_attributes = {
  .name = "PositionEvent2_MPC2"
};
/* Definitions for PositionEvent3_MPC2 */
osEventFlagsId_t PositionEvent3_MPC2Handle;
const osEventFlagsAttr_t PositionEvent3_MPC2_attributes = {
  .name = "PositionEvent3_MPC2"
};
/* Definitions for PositionEvent4_MPC2 */
osEventFlagsId_t PositionEvent4_MPC2Handle;
const osEventFlagsAttr_t PositionEvent4_MPC2_attributes = {
  .name = "PositionEvent4_MPC2"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSyncTask(void *argument);
void StartICTask(void *argument);
void StartRec_SwitchTask(void *argument);
void StartAlarmTask(void *argument);
void StartHole_ldentificationTask(void *argument);

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

  /* creation of Hole_ldentifica */
  Hole_ldentificaHandle = osThreadNew(StartHole_ldentificationTask, NULL, &Hole_ldentifica_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of PositionEvent1_MPC1 */
  PositionEvent1_MPC1Handle = osEventFlagsNew(&PositionEvent1_MPC1_attributes);

  /* creation of PositionEvent2_MPC1 */
  PositionEvent2_MPC1Handle = osEventFlagsNew(&PositionEvent2_MPC1_attributes);

  /* creation of PositionEvent3_MPC1 */
  PositionEvent3_MPC1Handle = osEventFlagsNew(&PositionEvent3_MPC1_attributes);

  /* creation of PositionEvent4_MPC1 */
  PositionEvent4_MPC1Handle = osEventFlagsNew(&PositionEvent4_MPC1_attributes);

  /* creation of PositionEvent1_MPC2 */
  PositionEvent1_MPC2Handle = osEventFlagsNew(&PositionEvent1_MPC2_attributes);

  /* creation of PositionEvent2_MPC2 */
  PositionEvent2_MPC2Handle = osEventFlagsNew(&PositionEvent2_MPC2_attributes);

  /* creation of PositionEvent3_MPC2 */
  PositionEvent3_MPC2Handle = osEventFlagsNew(&PositionEvent3_MPC2_attributes);

  /* creation of PositionEvent4_MPC2 */
  PositionEvent4_MPC2Handle = osEventFlagsNew(&PositionEvent4_MPC2_attributes);

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
  BaseType_t ret;
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

/* USER CODE BEGIN Header_StartHole_ldentificationTask */
/**
* @brief Function implementing the Hole_ldentifica thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHole_ldentificationTask */
void StartHole_ldentificationTask(void *argument)
{
  /* USER CODE BEGIN StartHole_ldentificationTask */
  BaseType_t ret;
  /* Infinite loop */
  for(;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if(ret == pdPASS){
          if (Isfirstcirculation)
    {
      position_old = (osEventFlagsGet(PositionEvent1_MPC1Handle) & 0xFFFF); // 上电第一次循环
      Isfirstcirculation = 0;
      xEventGroupClearBitsFromISR(PositionEvent1_MPC1Handle, 0xFFFF); // 必要的清0
    }
    else
    {
      position_new = (osEventFlagsGet(PositionEvent1_MPC1Handle) & 0xFFFF); // 除第一次外的每次循环
      position_xor = position_old ^ position_new;
      position_new_plus_old = position_new - position_old;
      if ((position_xor != 0) && (position_new_plus_old < 65536) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) // 有差异且位置检测从无到有算孔，从有到无不考虑
      {
        vTaskNotifyGiveFromISR(AlarmTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
      position_old = position_new;
      xEventGroupClearBitsFromISR(PositionEvent1_MPC1Handle, 0xFFFF); // 循环结束，清除所有标志位，以便新的一轮置位
    }
    }
    osDelay(1);
  }
  /* USER CODE END StartHole_ldentificationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

