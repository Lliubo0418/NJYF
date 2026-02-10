#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "semphr.h"
#include "queue.h"
#include "com_process.h"
#include "CAN_procTask.h"
#include "timers.h"
#include "tim.h"


extern osMessageQueueId_t canRxQueueHandle;
float task_interval_ms2 = 0;
void StartCAN_procTask(void *argument)
{
  CanMessage_t msg;
  uint8_t TxData[8];
	static uint32_t last_execution_time2 = 0;
  for (;;)
  {
    uint32_t current_time2 = __HAL_TIM_GET_COUNTER(&htim5);
    if (last_execution_time2 > 0)
    {
      uint32_t task_interval_ticks = current_time2 - last_execution_time2;
      task_interval_ms2 = (float)task_interval_ticks / 84000.0f; // Convert ticks to ms (84 MHz clock)
    }
    last_execution_time2 = current_time2;
    
    if (xQueueReceive(canRxQueueHandle, &msg, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      switch (msg.id)
      {
      case 0x80:
      {
        TxData[0] = 0x11;
        TxData[1] = 0x49;
        TxData[2] = 0x7E;
        TxData[3] = 0x06;
        TxData[4] = 0x01;

        int32_t pos_0p1mm;

        if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          if (ws_ds_flag == 0)
            pos_0p1mm = par_params.ds_strip_edge_position;
          else
            pos_0p1mm = par_params.ws_strip_edge_position;
          xSemaphoreGive(paramMutexHandle);
        }

        int32_t distance_um = pos_0p1mm * 100;

        const int32_t RANGE_UM = 65535;

        int32_t coeff = distance_um / RANGE_UM;
        int32_t remainder = distance_um % RANGE_UM;

        if (remainder < 0)
        {
          remainder += RANGE_UM;
          coeff -= 1;
        }

        TxData[5] = (uint8_t)(remainder & 0xFF);
        TxData[6] = (uint8_t)((remainder >> 8) & 0xFF);
        TxData[7] = (uint8_t)(coeff & 0xFF);

        CAN_send_msg(0x288, TxData, 8);
        osDelay(1);
        CAN_send_msg(0x188, TxData, 8);
        break;
      }

      case 0x208:
        break;

      case 0x608:
        break;

      case 0x00:
        break;

      case 0x701:
        break;

      default:
        break;
      }
    }

    osDelay(1);
  }
}

void heartCallback(void *argument)
{
  (void)argument;
  uint8_t data[1] = {0x05};
  CAN_send_msg(0x708, data, 1);
}
