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
/*同步信号*/
extern uint32_t Sync_capture_Buf[3]; // 存放计数值
extern uint8_t Sync_Cnt;             // 状态标志位
extern uint32_t Sync_high_time;      // 高电平时间


/* USER CODE END Variables */
/* Definitions for SYNCTask */
osThreadId_t SYNCTaskHandle;
const osThreadAttr_t SYNCTask_attributes = {
  .name = "SYNCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ICTask */
osThreadId_t ICTaskHandle;
const osThreadAttr_t ICTask_attributes = {
  .name = "ICTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Rec_SwitchTask */
osThreadId_t Rec_SwitchTaskHandle;
const osThreadAttr_t Rec_SwitchTask_attributes = {
  .name = "Rec_SwitchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Holes_OutputTas */
osThreadId_t Holes_OutputTasHandle;
const osThreadAttr_t Holes_OutputTas_attributes = {
  .name = "Holes_OutputTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSYNCTask(void *argument);
void StartICTask(void *argument);
void StartRec_switchTask(void *argument);
void StartHoles_OutputTask(void *argument);

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
  /* creation of SYNCTask */
  SYNCTaskHandle = osThreadNew(StartSYNCTask, NULL, &SYNCTask_attributes);

  /* creation of ICTask */
  ICTaskHandle = osThreadNew(StartICTask, NULL, &ICTask_attributes);

  /* creation of Rec_SwitchTask */
  Rec_SwitchTaskHandle = osThreadNew(StartRec_switchTask, NULL, &Rec_SwitchTask_attributes);

  /* creation of Holes_OutputTas */
  Holes_OutputTasHandle = osThreadNew(StartHoles_OutputTask, NULL, &Holes_OutputTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSYNCTask */
/**
  * @brief  Function implementing the SYNCTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSYNCTask */
void StartSYNCTask(void *argument)
{
  /* USER CODE BEGIN StartSYNCTask */
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
  /* Infinite loop */
  for(;;)
  {
		switch (Sync_Cnt)
    {
    case 0:
      Sync_Cnt++;
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
      HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); // 启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
      break;
    case 3:
      if (Sync_capture_Buf[1] < Sync_capture_Buf[0])
      { // 不存在先捕获高电平后捕获低电平capture_Buf[1]<capture_Buf[0],肯定是有了溢出
        Sync_high_time = 0x186A0 + Sync_capture_Buf[1] - Sync_capture_Buf[0];    //100000
      }
      else
      {
        Sync_high_time = Sync_capture_Buf[1] - Sync_capture_Buf[0]; // 高电平时间
      }

      Sync_Cnt = 0;
			 if (Sync_high_time == 0x1B)
      {

        Sync_Cnt = 5;
				xTaskNotify((TaskHandle_t)StartICTask,NULL,eSetValueWithOverwrite);
				xTaskNotify((TaskHandle_t)StartRec_switchTask,NULL,eSetValueWithOverwrite);
				
			}
		}
    osDelay(10);
  }
  /* USER CODE END StartSYNCTask */
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
  for(;;)
  {
    ret = xTaskNotifyWait((uint32_t)0x00,
                          (uint32_t)ULONG_MAX,
                          (uint32_t *)&NotifyValue,
                          (TickType_t)portMAX_DELAY);
          if(ret==pdPASS){
            //能否正确开启捕获？待测试
            //LTODO:待测试
            //LHACK:如果不行更换为HAL_TIM_IC_Start_IT函数
            //LXXX:按照所查资料提示，如果不行那就还是将同步引脚更改为其他引脚，TIM2作为主定时器通过事件触发从定时器同步捕获，硬件层面的同步捕获
            __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_CC4);    
            __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);    
            __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_CC1);
            __HAL_TIM_ENABLE_IT(&htim9,TIM_IT_CC1);

          }
					else{
            printf("StartICTask任务通知获取失败\r\n");
          }
    osDelay(1);
  }
  /* USER CODE END StartICTask */
}

/* USER CODE BEGIN Header_StartRec_switchTask */
/**
* @brief Function implementing the Rec_SwitchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRec_switchTask */
void StartRec_switchTask(void *argument)
{
  /* USER CODE BEGIN StartRec_switchTask */
  BaseType_t ret;
  uint32_t NotifyValue;
  /* Infinite loop */
  for (;;)
  {
    ret = xTaskNotifyWait((uint32_t)0x00,
                          (uint32_t)ULONG_MAX,
                          (uint32_t *)&NotifyValue,
                          (TickType_t)portMAX_DELAY);
    if (ret == pdPASS)
    {
      EN_R1_R2_open();
      HAL_TIM_Base_Start_IT(&htim6);    //start timer for R1_channel switch
    }
    else
    {
      printf("StartRec_switchTask任务通知获取失败\r\n");
    }
    osDelay(1);
  }
  /* USER CODE END StartRec_switchTask */
}

/* USER CODE BEGIN Header_StartHoles_OutputTask */
/**
* @brief Function implementing the Holes_OutputTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHoles_OutputTask */
void StartHoles_OutputTask(void *argument)
{
  /* USER CODE BEGIN StartHoles_OutputTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHoles_OutputTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

