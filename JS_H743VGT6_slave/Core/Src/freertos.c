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
#include "event_groups.h"

extern uint32_t Sync_capture_Buf[2]; // ��ż���ֵ
extern uint8_t Sync_Cnt;             // ״̬��־λ
extern uint32_t Sync_high_time;      // �ߵ�ƽʱ��

uint16_t semavalue = 0;

uint8_t Isfirstcirculation = 1;

int32_t position_old[4] = {0};
int32_t position_xor[4] = {0};

uint8_t position_num=0;
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
osThreadId_t Hole_ldentificationTaskHandle;
const osThreadAttr_t Hole_ldentifica_attributes = {
  .name = "Hole_ldentificationTask",
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
  Hole_ldentificationTaskHandle = osThreadNew(StartHole_ldentificationTask, NULL, &Hole_ldentifica_attributes);

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
      HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4); // �������벶��       ����: __HAL_TIM_ENABLE(&htim5);
      break;
    case 3:
      if (Sync_capture_Buf[1] < Sync_capture_Buf[0])
      {                                                                       // �������Ȳ���ߵ�ƽ�󲶻�͵�ƽcapture_Buf[1]<capture_Buf[0],�϶����������
        Sync_high_time = 0x186A0 + Sync_capture_Buf[1] - Sync_capture_Buf[0]; // 100000    0x186A0
      }
      else
      {
        Sync_high_time = Sync_capture_Buf[1] - Sync_capture_Buf[0]; // �ߵ�ƽʱ��
      }

     // Sync_Cnt = 0;
      if (Sync_high_time == 0x1B)
      {
        Sync_Cnt = 5;
        xTaskNotifyGive((TaskHandle_t)ICTaskHandle);
        xTaskNotifyGive((TaskHandle_t)Rec_SwitchTaskHandle);
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

  /* Infinite loop */
  for (;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)
    {
      // LXXX:��������������ʾ����������Ǿͻ��ǽ�ͬ�����Ÿ���Ϊ�������ţ�TIM2��Ϊ����ʱ��ͨ���¼������Ӷ�ʱ��ͬ������Ӳ�������ͬ������
      HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);
      HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
    }
    else
    {
      printf("StartICTask����֪ͨ��ȡʧ��\r\n");
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
  /* Infinite loop */
  for (;;)
  {

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
      printf("StartRec_switchTask����֪ͨ��ȡʧ��\r\n");
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
  TickType_t xLastWakeTime;



  // ��ʼ��xLastWakeTimeΪ��ǰ��tick����
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for (;;)
  {

    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)
    {
      for (int j = 0; j < 32; j++)
      {
        while (((position_xor[position_num] >> j) & 1) == 1)
        {
          semavalue++;
        }
      }
    }

    if (semavalue > 0)
    {
      semavalue--;
      Holes_output_alarm_Reported();
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
  int32_t position_new[4] = {0};

  int32_t position_new_plus_old[4] = {0};
  EventGroupHandle_t eventHandles[] = {
      PositionEvent1_MPC1Handle, PositionEvent1_MPC2Handle,
      PositionEvent2_MPC1Handle, PositionEvent2_MPC2Handle,
      PositionEvent3_MPC1Handle, PositionEvent3_MPC2Handle,
      PositionEvent4_MPC1Handle, PositionEvent4_MPC2Handle};
  /* Infinite loop */
  for (;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)

    {
      if (Isfirstcirculation) // �ϵ��һ��ѭ��
      {
        for (position_num = 0; position_num < 4; position_num++)
        {
          position_old[position_num] = (osEventFlagsGet(eventHandles[position_num * 2 + 1]) & 0xFFFF);
          position_old[position_num] |= (osEventFlagsGet(eventHandles[position_num * 2]) & 0xFFFF) << 16;
        }
        Isfirstcirculation = 0;
      }
      else
      {
        // ����position_new������
        for (position_num = 0; position_num < 4; position_num++)
        {
          // ��ȡ��ֵ
          position_new[position_num] = (osEventFlagsGet(eventHandles[position_num * 2 + 1]) & 0xFFFF);
          position_new[position_num] |= (osEventFlagsGet(eventHandles[position_num * 2]) & 0xFFFF) << 16;

          // �������Ͳ�ֵ
          position_xor[position_num] = position_old[position_num] ^ position_new[position_num];
          position_new_plus_old[position_num] = position_new[position_num] - position_old[position_num];

          // ��������֪ͨ
          if (position_xor[position_num] != 0 && position_new_plus_old[position_num] > 0 && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
          {
            xTaskNotifyGive(AlarmTaskHandle);
          }

          // ����oldֵ
          position_old[position_num] = position_new[position_num];
        }

        // ������б�־λ���Ա��µ�һ����λ
        for (position_num = 0; position_num < 8; position_num++)
        {
          xEventGroupClearBits(eventHandles[position_num], 0xFFFF);
        }
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartHole_ldentificationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
