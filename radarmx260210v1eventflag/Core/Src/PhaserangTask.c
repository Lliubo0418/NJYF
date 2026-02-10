#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "spi.h"
#include "data_process.h"
#include "com_process.h"
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
#include <math.h>


extern EventGroupHandle_t flagsEventHandle;

// For measuring task execution interval in milliseconds
float task_interval_ms = 0;

void StartPhaserangTask(void *argument)
{
  uint16_t spi_ctrl = 0;
  uint16_t spi_buf[8];
  MX_USB_DEVICE_Init();

  static uint32_t last_disposal_cnt = 0xFFFFFFFF;
  static uint32_t last_read_cnt = 0xFFFFFFFF;
  static uint32_t last_gate_left = 0xFFFFFFFF;
  static uint32_t last_gate_right = 0xFFFFFFFF;
  
  // For measuring task execution interval using TIM5
  static uint32_t last_execution_time = 0;

  for (;;)
  {
    // Calculate task execution interval using TIM5 and convert to milliseconds
    uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim5);
    if (last_execution_time > 0)
    {
      uint32_t task_interval_ticks = current_time - last_execution_time;
      task_interval_ms = (float)task_interval_ticks / 84000.0f; // Convert ticks to ms (84 MHz clock)
    }
    last_execution_time = current_time;

    fpga_read_flag = 0;

    if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      if (last_disposal_cnt != par_params.data_disposal_count)
      {
        spi_ctrl = 0;
        spi_ctrl |= 1 << 14;
        spi_buf[0] = spi1_write_bit16(spi_ctrl & 0xFFFF);
        spi_buf[1] = spi1_write_bit16(par_params.data_disposal_count & 0xFFFF);
        last_disposal_cnt = par_params.data_disposal_count;
      }

      if (last_read_cnt != par_params.data_read_count)
      {
        spi_ctrl = 0;
        spi_ctrl |= 1 << 13;
        spi_buf[2] = spi1_write_bit16(spi_ctrl & 0xFFFF);
        spi_buf[3] = spi1_write_bit16(par_params.data_read_count & 0xFFFF);
        last_read_cnt = par_params.data_read_count;
      }

      if (last_gate_left != par_params.strip_gate_line_left_1)
      {
        spi_ctrl = 0;
        spi_ctrl |= 1 << 12;
        spi_buf[2] = spi1_write_bit16(spi_ctrl & 0xFFFF);
        spi_buf[3] = spi1_write_bit16(par_params.strip_gate_line_left_1 & 0xFFFF);
        last_gate_left = par_params.strip_gate_line_left_1;
      }

      if (last_gate_right != par_params.strip_gate_line_right_1)
      {
        spi_ctrl = 0;
        spi_ctrl |= 1 << 11;
        spi_buf[2] = spi1_write_bit16(spi_ctrl & 0xFFFF);
        spi_buf[3] = spi1_write_bit16(par_params.strip_gate_line_right_1 & 0xFFFF);
        last_gate_right = par_params.strip_gate_line_right_1;
      }
      xSemaphoreGive(paramMutexHandle);
    }
    
    EventBits_t flags = xEventGroupWaitBits(flagsEventHandle, FPGA_EVT_READY, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));

    if (flags & FPGA_EVT_READY)
    {
      if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        par_params.error &= ~(1 << 8);
        xSemaphoreGive(paramMutexHandle);
      }

      uint16_t spi_ctrl = 0x8000;
      
      peak_pos_h = spi1_write_bit16(spi_ctrl & 0xFFFF);
      spi_buf[4] = peak_pos_h;
      peak_pos_l = spi1_write_bit16(0);
      spi_buf[5] = peak_pos_l;

      uint32_t raw_peak = ((peak_pos_h & 0x03) << 16) | peak_pos_l;     //18位数据？24位数据？但无论哪个都不能使用uint16_t

      peak_pos = raw_peak;

      if (peak_pos != 0)
      {
        data_process();
      }

    }
    else
    {
      if (tigger_error_flag != 0)
      {
        par_params.error |= (1 << 8);
        tigger_error_flag = 0;
      }

      osDelay(1);
    }
    osDelay(1);

#if DBG_FPGA_SRAM_USB
    if (!sram_read_flag)
    {
      FPGA_RAM_FLAG(0);
      osDelay(1);

      uint32_t addr = 0;
      for (addr = 0; addr < SRAM_READ_SIZE; addr += buf_len)
      {
        if (xSemaphoreTake(usbBufferMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          memset(g_usb_usart_tx_buffer, 0, sizeof(g_usb_usart_tx_buffer));
          sram_read_u16((uint16_t *)g_usb_usart_tx_buffer, addr, data_num);

          CDC_Transmit_FS(g_usb_usart_tx_buffer, buf_len);
          xSemaphoreGive(usbBufferMutexHandle);
        }

        osDelay(3);
      }

      osDelay(30000);
      if (xSemaphoreTake(usbBufferMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        for (int i = 0; i < buf_len; i++)
          g_usb_usart_tx_buffer[i] = 0x01;
        CDC_Transmit_FS(g_usb_usart_tx_buffer, 8);
        xSemaphoreGive(usbBufferMutexHandle);
      }
      osDelay(500);

      FPGA_RAM_FLAG(1);
      osDelay(1);

      for (addr = 0; addr < SRAM_READ_SIZE; addr += buf_len)
      {
        if (xSemaphoreTake(usbBufferMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          memset(g_usb_usart_tx_buffer, 0, sizeof(g_usb_usart_tx_buffer));
          sram_read_u16((uint16_t *)g_usb_usart_tx_buffer, addr, data_num);
          CDC_Transmit_FS(g_usb_usart_tx_buffer, buf_len);
          xSemaphoreGive(usbBufferMutexHandle);
        }

        osDelay(3);
      }
      osDelay(30000);
      if (xSemaphoreTake(usbBufferMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        for (int i = 0; i < buf_len; i++)
          g_usb_usart_tx_buffer[i] = 0x02;
        CDC_Transmit_FS(g_usb_usart_tx_buffer, 8);
        xSemaphoreGive(usbBufferMutexHandle);
      }
      osDelay(500);

      if (busy_cnt != 0)
      {
        printf("%d packets are lost in transmission!!!!!\n", busy_cnt);
      }
      busy_cnt = 0;

      sram_read_flag = 0;
    }
#endif
  }
}
