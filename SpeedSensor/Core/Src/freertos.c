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
#include "spi.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint16_t Frequency;
const float DAC_COEFFICIENT = 65536.0 / 2.5;
const uint8_t DAC_DEFAULT_MODE = 0;
uint16_t Frequency_old = 0;
uint32_t period = 0;                //LTODO:测试后改回PWM_SetFrequencyDutyCycle内
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void PWM_SetFrequencyDutyCycle(uint16_t frequency);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Freq_outTaskHandle;
osThreadId Current_outTaskHandle;
osThreadId Freq_ICTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartFreq_outTask(void const *argument);
void StartCurrent_outTask(void const *argument);
void StartFreq_ICTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* definition and creation of Freq_outTask */
  osThreadDef(Freq_outTask, StartFreq_outTask, osPriorityNormal, 0, 128);
  Freq_outTaskHandle = osThreadCreate(osThread(Freq_outTask), NULL);

  /* definition and creation of Current_outTask */
  osThreadDef(Current_outTask, StartCurrent_outTask, osPriorityNormal, 0, 128);
  Current_outTaskHandle = osThreadCreate(osThread(Current_outTask), NULL);

  /* definition and creation of Freq_ICTask */
  osThreadDef(Freq_ICTask, StartFreq_ICTask, osPriorityAboveNormal, 0, 128);
  Freq_ICTaskHandle = osThreadCreate(osThread(Freq_ICTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartFreq_outTask */
/**
 * @brief  Function implementing the Freq_outTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFreq_outTask */
void StartFreq_outTask(void const *argument)
{
  /* USER CODE BEGIN StartFreq_outTask */
  BaseType_t ret;
  /* Infinite loop */
  for (;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)
    {
      // LTODO: 添加对应按照线性关系的频率输出
      PWM_SetFrequencyDutyCycle(Frequency_old);
    }

    osDelay(10);
  }
  /* USER CODE END StartFreq_outTask */
}

/* USER CODE BEGIN Header_StartCurrent_outTask */
/**
 * @brief Function implementing the Current_outTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCurrent_outTask */
void StartCurrent_outTask(void const *argument)
{
  /* USER CODE BEGIN StartCurrent_outTask */
  BaseType_t ret;
  uint16_t DATA = 0;
  float Vout = 0;
  /* Infinite loop */
  for (;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)
    {
      Vout = 0.02 * Frequency_old + 0.4;
      DATA = (uint16_t)(Vout * DAC_COEFFICIENT);
      SPI_DAC8560_Write(DAC_DEFAULT_MODE, DATA);
    }
    osDelay(10);
  }
  /* USER CODE END StartCurrent_outTask */
}

/* USER CODE BEGIN Header_StartFreq_ICTask */
/**
 * @brief Function implementing the Freq_ICTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFreq_ICTask */
void StartFreq_ICTask(void const *argument)
{
  /* USER CODE BEGIN StartFreq_ICTask */
  uint16_t DATA = 0;
  BaseType_t ret;
  /* Infinite loop */
  for (;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)
    {
      if (Frequency_old != Frequency)
      {
        if (Frequency > 0 && Frequency <= 80)
        {
          xTaskNotifyGive(Freq_outTaskHandle);
          xTaskNotifyGive(Current_outTaskHandle);
          Frequency_old = Frequency;
        }
        else if (Frequency == 0)
        {
          // LTODO:频率输出200Hz，电流输出4mA，即默认值
          PWM_SetFrequencyDutyCycle(0);             // 200HZ
          DATA = (uint16_t)(0.4 * DAC_COEFFICIENT); // 4mA
          SPI_DAC8560_Write(DAC_DEFAULT_MODE, DATA);
          Frequency_old = Frequency;
        }
        else if (Frequency > 80)
        {
          // LTODO:超速报警，关闭一切输出
          HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); // 关闭TIM4通道4的PWM
          SPI_DAC8560_Write(DAC_DEFAULT_MODE, 0);
        }
      }
    }

    osDelay(10);
  }
  /* USER CODE END StartFreq_ICTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void PWM_SetFrequencyDutyCycle(uint16_t frequency)
{
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); // 关闭TIM4通道4的PWM
  // 计算定时器的时钟频率

  

  // 计算周期值

  period = (uint32_t)round((1000000.0 / (10 * frequency + 200))) - 1;

  // 更新定时器的预分频器和周期

  htim4.Instance->ARR = period; // 设置自动重装载寄存器（周期）

  // 设置PWM占空比
  htim4.Instance->CCR4 = (uint32_t)round(((float)(period + 1) / 2)); // 设置比较寄存器（占空比）

  // 启动PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 启动TIM4通道4的PWM
}
/* USER CODE END Application */
