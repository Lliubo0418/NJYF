#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "tim.h"
#include "event_groups.h"
#include "semphr.h"
#include "com_process.h"
#include "UsbtoUsartTask.h"

extern EventGroupHandle_t flagsEventHandle;
uint8_t adc_ws_ds_flag = 0;
float task_interval_ms1 = 0;

void StartUsbtoUsartTask(void *argument)
{
  uint8_t ack = 0x06;
	static uint32_t last_execution_time1 = 0;
  for (;;)
  {
		uint32_t current_time1 = __HAL_TIM_GET_COUNTER(&htim5);
    if (last_execution_time1 > 0)
    {
      uint32_t task_interval_ticks = current_time1 - last_execution_time1;
      task_interval_ms1 = (float)task_interval_ticks / 84000.0f; // Convert ticks to ms (84 MHz clock)
    }
    last_execution_time1 = current_time1;
		

		
    EventBits_t usb_flags = xEventGroupWaitBits(flagsEventHandle, USB_EVT_RX_DONE, pdTRUE, pdFALSE, portMAX_DELAY);
		

    if (usb_flags & USB_EVT_RX_DONE)
    {
      if (g_usb_usart_rx_sta > 0)
      {
        uint8_t head = g_usb_usart_rx_buffer[0];

        if (head == 0x06 && g_usb_usart_rx_sta >= 1)
        {
          HAL_TIM_Base_Stop_IT(&htim3);
          waiting_for_ack = 0;

          g_usb_usart_rx_sta = 0;
          memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
          buffer_overflowFlag = 0;
        }
        else if (head == 0x15 && g_usb_usart_rx_sta >= 5)
        {
          if (g_usb_usart_rx_buffer[1] == 0x80)
          {
            CDC_Transmit_FS(&ack, 1);
          }
          else if (g_usb_usart_rx_buffer[1] == 0x00 && g_usb_usart_rx_buffer[2] == 0x65)
          {
            if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
            {
              par_params.receive_command = (g_usb_usart_rx_buffer[1] << 8) | g_usb_usart_rx_buffer[2];
              xSemaphoreGive(paramMutexHandle);
            }
            Handle_Parameter_Read(0x65);
          }
          else if (g_usb_usart_rx_buffer[1] == 0x00 && g_usb_usart_rx_buffer[2] == 0x01)
          {
            if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
            {
              par_params.receive_command = (g_usb_usart_rx_buffer[1] << 8) | g_usb_usart_rx_buffer[2];
              xSemaphoreGive(paramMutexHandle);
            }
            Handle_Parameter_Read(0x01);
            waiting_for_ack = 1;
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            HAL_TIM_Base_Start_IT(&htim3);
          }
          else if (g_usb_usart_rx_buffer[1] == 0x4B)
          {
t0 = TIM5->CNT;
            if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
            {
              par_params.receive_command = (g_usb_usart_rx_buffer[1] << 8) | g_usb_usart_rx_buffer[2];
              xSemaphoreGive(paramMutexHandle);
            }
            adc_ws_ds_flag = g_usb_usart_rx_buffer[2];
            uint16_t start_pos = (g_usb_usart_rx_buffer[3] << 8) | g_usb_usart_rx_buffer[4];
            Handle_ADC_Collect(adc_ws_ds_flag, start_pos);
t1 = TIM5->CNT;
delta_times = (t1 - t0) / 84.0f / 1000.0f;
          }
          else if (g_usb_usart_rx_buffer[1] == 0xAA)
          {
            if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
            {
              if (g_usb_usart_rx_buffer[2] == 0)
              {
                par_params.ds_zero_position = peak_pos - 833;
                gst_rsp_parameter.ds_phase_offset_sum = 0;
                par_params.ds_base_position = par_params.ds_iq_data;
                par_params.ds_strip_edge_position = par_params.ds_iq_data;
              }
              else
              {
                par_params.ws_zero_position = peak_pos - 833;
                gst_rsp_parameter.ws_phase_offset_sum = 0;
                par_params.ws_base_position = par_params.ws_iq_data;
                par_params.ws_strip_edge_position = par_params.ws_iq_data;
              }
              xSemaphoreGive(paramMutexHandle);
            }
          }
          else if (g_usb_usart_rx_buffer[1] == 0xBB)
          {
            if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
            {
              par_params.error = (g_usb_usart_rx_buffer[2]) << 8 | g_usb_usart_rx_buffer[3];
              xSemaphoreGive(paramMutexHandle);
            }
          }

          g_usb_usart_rx_sta = 0;
          memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
          buffer_overflowFlag = 0;
        }
        else if (head == 0x01 && g_usb_usart_rx_sta >= 128)
        {
          if (g_usb_usart_rx_buffer[1] == 0x01 && g_usb_usart_rx_buffer[2] == 0xFE)
          {
            CDC_Transmit_FS(&ack, 1);

            uint8_t cmd_code = g_usb_usart_rx_buffer[8];
            Handle_Setting_Command(g_usb_usart_rx_buffer, cmd_code);
          }

          g_usb_usart_rx_sta = 0;
          memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
          buffer_overflowFlag = 0;
        }
        else if (g_usb_usart_rx_sta >= USB_USART_REC_LEN)
        {
          g_usb_usart_rx_sta = 0;
          memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
          buffer_overflowFlag = 1;
        }
      }
    }
  }
}
