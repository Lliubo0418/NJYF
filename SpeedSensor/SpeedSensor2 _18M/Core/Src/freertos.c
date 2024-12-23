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
uint16_t power_on_init = 1;
extern float Frequency;
const float DAC_COEFFICIENT = 65536.0 / 2.5;
const uint8_t DAC_DEFAULT_MODE = 0;
float Frequency_old = 0;
const uint16_t Frequency_Input_Max = 80;
uint32_t period = 0; // LTODO:测试后改回PWM_SetFrequencyDutyCycle内
uint16_t TimerFlag = 1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void PWM_SetFrequencyDutyCycle(float frequency);
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
osThreadId ADCTaskHandle;
osTimerId SlipTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartFreq_outTask(void const * argument);
void StartCurrent_outTask(void const * argument);
void StartFreq_ICTask(void const * argument);
void StartADCTask(void const * argument);
void SlipTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the timer(s) */
  /* definition and creation of SlipTimer */
  osTimerDef(SlipTimer, SlipTimerCallback);
  SlipTimerHandle = osTimerCreate(osTimer(SlipTimer), osTimerOnce, NULL);

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

  /* definition and creation of ADCTask */
  osThreadDef(ADCTask, StartADCTask, osPriorityNormal, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);

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
void StartFreq_outTask(void const * argument)
{
  /* USER CODE BEGIN StartFreq_outTask */
  BaseType_t ret;
  /* Infinite loop */
  for (;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)
    {
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
void StartCurrent_outTask(void const * argument)
{
  /* USER CODE BEGIN StartCurrent_outTask */
  BaseType_t ret;
  float DATA = 0;
  float Vout = 0;
  /* Infinite loop */
  for (;;)
  {
    ret = ulTaskNotifyTake(pdFALSE, 0);
    if (ret == pdPASS)
    {
      Vout = 0.02 * Frequency_old + 0.4;
			
      DATA = (float)(Vout * DAC_COEFFICIENT);
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
void StartFreq_ICTask(void const * argument)
{
  /* USER CODE BEGIN StartFreq_ICTask */
  float DATA = 0;
  BaseType_t ret;
  /* Infinite loop */
  for (;;)
  {
    if (power_on_init)
    {
      DATA = (float)(0.4 * DAC_COEFFICIENT); // 4mA
//		DATA = (float)(0.397 * DAC_COEFFICIENT); // 4mA   如需下降0.03的电压替换上一行
      SPI_DAC8560_Write(DAC_DEFAULT_MODE, DATA);
			SPI_DAC8560_Write(DAC_DEFAULT_MODE, DATA);
      power_on_init = 0;
    }
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
          if (Frequency >= 20 && Frequency <= 28)
          {
            if (TimerFlag)
            {
              xTimerReset(SlipTimerHandle, 0);
              xTimerStart(SlipTimerHandle, pdMS_TO_TICKS(10000));
              TimerFlag = 0;
            }
          }
          else if (TimerFlag == 0)
          {
            xTimerStop(SlipTimerHandle, 0);
            TimerFlag = 1;
          }
        }
        else if (Frequency == 0)
        {
          
          PWM_SetFrequencyDutyCycle(0);             // 200HZ
          DATA = (float)(0.4 * DAC_COEFFICIENT); // 4mA
//				DATA = (float)(0.397 * DAC_COEFFICIENT); // 4mA   如需下降0.03的电压替换上一行
          SPI_DAC8560_Write(DAC_DEFAULT_MODE, DATA);
          Frequency_old = Frequency;
        }
        else if (Frequency > Frequency_Input_Max)
        {

          PWM_SetFrequencyDutyCycle(Frequency_Input_Max);
          DATA = (float)(2 * DAC_COEFFICIENT); // 20mA
//				DATA = (float)(0.197 * DAC_COEFFICIENT); // 4mA   如需下降0.03的电压替换上一行
          SPI_DAC8560_Write(DAC_DEFAULT_MODE, DATA);
        }
      }
    }

    osDelay(10);
  }
  /* USER CODE END StartFreq_ICTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
 * @brief Function implementing the ADCTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADCTask */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */

  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
   
    vTaskDelay(500);
    osDelay(1);
  }
  /* USER CODE END StartADCTask */
}

/* SlipTimerCallback function */
void SlipTimerCallback(void const * argument)
{
  /* USER CODE BEGIN SlipTimerCallback */

  HAL_GPIO_WritePin(K1_4_GPIO_Port, K1_4_Pin, GPIO_PIN_RESET);
  /* USER CODE END SlipTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void PWM_SetFrequencyDutyCycle(float frequency)
{


  period = (uint32_t)round((1000000.0 / (10 * frequency + 200))) - 1; // float->uint32_t

  // 更新定时器的预分频器和周期

  htim3.Instance->ARR = period; // 设置周期

  // 设置PWM占空比
  htim3.Instance->CCR4 = (uint32_t)round(((float)(period + 1) / 2)); // 设置比较寄存器（占空比）

  // 启动PWM

}
/* USER CODE END Application */

