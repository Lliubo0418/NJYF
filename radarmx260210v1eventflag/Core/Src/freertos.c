/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "spi.h"
#include "data_process.h"
#include "com_process.h"
#include <math.h>
#include "usbd_cdc_if.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "PhaserangTask.h"
#include "UsbtoUsartTask.h"
#include "CAN_procTask.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SRAM_READ_SIZE 40000
#define buf_len 1000
#define data_num (buf_len / sizeof(uint16_t))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */



/* USER CODE END Variables */
/* Definitions for PhaserangTask */
osThreadId_t PhaserangTaskHandle;
const osThreadAttr_t PhaserangTask_attributes = {
  .name = "PhaserangTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UsbtoUsartTask */
osThreadId_t UsbtoUsartTaskHandle;
const osThreadAttr_t UsbtoUsartTask_attributes = {
  .name = "UsbtoUsartTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CAN_procTask */
osThreadId_t CAN_procTaskHandle;
const osThreadAttr_t CAN_procTask_attributes = {
  .name = "CAN_procTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};
/* Definitions for usbRxQueue */
osMessageQueueId_t usbRxQueueHandle;
const osMessageQueueAttr_t usbRxQueue_attributes = {
  .name = "usbRxQueue"
};
/* Definitions for heartbeatTimer */
osTimerId_t heartbeatTimerHandle;
const osTimerAttr_t heartbeatTimer_attributes = {
  .name = "heartbeatTimer"
};
/* Definitions for usbBufferMutex */
osMutexId_t usbBufferMutexHandle;
const osMutexAttr_t usbBufferMutex_attributes = {
  .name = "usbBufferMutex"
};
/* Definitions for paramMutex */
osMutexId_t paramMutexHandle;
const osMutexAttr_t paramMutex_attributes = {
  .name = "paramMutex"
};
/* Definitions for flagsEvent */
EventGroupHandle_t flagsEventHandle;
const osEventFlagsAttr_t flagsEvent_attributes = {
  .name = "flagsEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartPhaserangTask(void *argument);
void StartUsbtoUsartTask(void *argument);
void StartCAN_procTask(void *argument);
void heartCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of usbBufferMutex */
  usbBufferMutexHandle = osMutexNew(&usbBufferMutex_attributes);

  /* creation of paramMutex */
  paramMutexHandle = osMutexNew(&paramMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of heartbeatTimer */
  heartbeatTimerHandle = osTimerNew(heartCallback, osTimerPeriodic, NULL, &heartbeatTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  osTimerStart(heartbeatTimerHandle, 105U);
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew(16, sizeof(CanMessage_t), &canRxQueue_attributes);

  /* creation of usbRxQueue */
  usbRxQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &usbRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PhaserangTask */
  PhaserangTaskHandle = osThreadNew(StartPhaserangTask, NULL, &PhaserangTask_attributes);

  /* creation of UsbtoUsartTask */
  UsbtoUsartTaskHandle = osThreadNew(StartUsbtoUsartTask, NULL, &UsbtoUsartTask_attributes);

  /* creation of CAN_procTask */
  CAN_procTaskHandle = osThreadNew(StartCAN_procTask, NULL, &CAN_procTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of flagsEvent */
  flagsEventHandle = xEventGroupCreate();

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  if (CAN_Enable_RxInterrupt() != HAL_OK)
  {

  }
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartPhaserangTask */
/**
 * @brief  Function implementing the PhaserangTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPhaserangTask */
__weak void StartPhaserangTask(void *argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN StartPhaserangTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPhaserangTask */
}

/* USER CODE BEGIN Header_StartUsbtoUsartTask */
/**
 * @brief Function implementing the UsbtoUsartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUsbtoUsartTask */
__weak void StartUsbtoUsartTask(void *argument)
{
  /* USER CODE BEGIN StartUsbtoUsartTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsbtoUsartTask */
}

/* USER CODE BEGIN Header_StartCAN_procTask */
/**
 * @brief Function implementing the CAN_procTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCAN_procTask */
__weak void StartCAN_procTask(void *argument)
{
  /* USER CODE BEGIN StartCAN_procTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCAN_procTask */
}

/* heartCallback function */
__weak void heartCallback(void *argument)
{
  /* USER CODE BEGIN heartCallback */

  /* USER CODE END heartCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

