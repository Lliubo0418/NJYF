
/**
 ****************************************************************************************************
 * @file        com_process.c
 * @author
 * @version     V1.1
 * @date        2025-03-20
 * @brief       通信处理模块
 *              接收模块：报头解析、长度解析、CRC码解析 (03.28, ok)
 *              发送模块：报头组装、长度组装、CRC组装
 * @license     Copyright (c) 2020-2032, 常州潞城
 ****************************************************************************************************
 */
#include "com_process.h"
#include <string.h>
#include "stmflash.h"
#include "fmc.h"
#include "usbd_cdc_if.h"
#include "spi.h"
#include "cmsis_os.h"
#include "semphr.h"

#define MYDEBUG 0


uint32_t t0 = 0, t1 = 0;
float delta_times = 0;

uint32_t g_tx_flag = 0x00;
uint32_t g_rx_flag = 0x00;
uint8_t sin_data[400], cos_data[400];

uint32_t device_side_flag = DEVICE_SIDE_WS;
/* 填充128字节响应 MSB*/
void Prepare_128Byte_Response(uint8_t *buffer, uint8_t cmd_code)
{
  memset(buffer, 0, 128);
  if (cmd_code == 0x65)
  {
    buffer[0] = 0x01;
    buffer[1] = 0x01;
    buffer[2] = 0xFE;
    /* 填充 ParameterFrame 字段（高位在前，所有字段 2 字节，除 sin_gain 和 cos_gain） */
    buffer[3] = (par_params.ds_strip_edge_position >> 8) & 0xFF; // 高位
    buffer[4] = par_params.ds_strip_edge_position & 0xFF;        // 低位
    buffer[5] = (par_params.ws_strip_edge_position >> 8) & 0xFF;
    buffer[6] = par_params.ws_strip_edge_position & 0xFF;
    buffer[7] = (par_params.ds_base_position >> 8) & 0xFF;
    buffer[8] = par_params.ds_base_position & 0xFF;
    buffer[9] = (par_params.ws_base_position >> 8) & 0xFF;
    buffer[10] = par_params.ws_base_position & 0xFF;
    buffer[11] = (par_params.ds_theta_base >> 8) & 0xFF;
    buffer[12] = par_params.ds_theta_base & 0xFF;
    buffer[13] = (par_params.ws_theta_base >> 8) & 0xFF;
    buffer[14] = par_params.ws_theta_base & 0xFF;
    buffer[15] = (par_params.ds_theta_current >> 8) & 0xFF;
    buffer[16] = par_params.ds_theta_current & 0xFF;
    buffer[17] = (par_params.ws_theta_current >> 8) & 0xFF;
    buffer[18] = par_params.ws_theta_current & 0xFF;
    buffer[19] = (par_params.ds_rotate_n_data >> 8) & 0xFF;
    buffer[20] = par_params.ds_rotate_n_data & 0xFF;
    buffer[21] = (par_params.ws_rotate_n_data >> 8) & 0xFF;
    buffer[22] = par_params.ws_rotate_n_data & 0xFF;
    buffer[23] = (par_params.ds_iq_data >> 8) & 0xFF;
    buffer[24] = par_params.ds_iq_data & 0xFF;
    buffer[25] = (par_params.ws_iq_data >> 8) & 0xFF;
    buffer[26] = par_params.ws_iq_data & 0xFF;
    buffer[27] = (par_params.board_temperature >> 8) & 0xFF;
    buffer[28] = par_params.board_temperature & 0xFF;
    buffer[29] = (par_params.ds_antenna_temperature >> 8) & 0xFF;
    buffer[30] = par_params.ds_antenna_temperature & 0xFF;
    buffer[31] = (par_params.ws_antenna_temperature >> 8) & 0xFF;
    buffer[32] = par_params.ws_antenna_temperature & 0xFF;
    buffer[33] = (par_params.panel_temperature >> 8) & 0xFF;
    buffer[34] = par_params.panel_temperature & 0xFF;
    buffer[35] = (par_params.strip_gate_line_left_1 >> 8) & 0xFF;
    buffer[36] = par_params.strip_gate_line_left_1 & 0xFF;
    buffer[37] = (par_params.strip_gate_line_right_1 >> 8) & 0xFF;
    buffer[38] = par_params.strip_gate_line_right_1 & 0xFF;
    buffer[39] = (par_params.ds_strip_edge_line >> 8) & 0xFF;
    buffer[40] = par_params.ds_strip_edge_line & 0xFF;
    buffer[41] = (par_params.ws_strip_edge_line >> 8) & 0xFF;
    buffer[42] = par_params.ws_strip_edge_line & 0xFF;
    buffer[43] = (par_params.iq_theta_long_time_count >> 8) & 0xFF;
    buffer[44] = par_params.iq_theta_long_time_count & 0xFF;
    buffer[45] = (par_params.error >> 8) & 0xFF;
    buffer[46] = par_params.error & 0xFF;
    buffer[47] = (par_params.receive_command >> 8) & 0xFF;
    buffer[48] = par_params.receive_command & 0xFF;
    buffer[49] = 0x08;
    buffer[50] = 0x37;
  }
  else
  {
    buffer[0] = 0x01;
    buffer[1] = 0x01;
    buffer[2] = 0xFE;
   
    buffer[3] = (par_params.ds_strip_edge_position >> 8) & 0xFF; // 高位
    buffer[4] = par_params.ds_strip_edge_position & 0xFF;        // 低位
    buffer[5] = (par_params.ws_strip_edge_position >> 8) & 0xFF;
    buffer[6] = par_params.ws_strip_edge_position & 0xFF;
    buffer[7] = (par_params.ds_base_position >> 8) & 0xFF;
    buffer[8] = par_params.ds_base_position & 0xFF;
    buffer[9] = (par_params.ws_base_position >> 8) & 0xFF;
    buffer[10] = par_params.ws_base_position & 0xFF;
    buffer[11] = (par_params.ds_theta_base >> 8) & 0xFF;
    buffer[12] = par_params.ds_theta_base & 0xFF;
    buffer[13] = (par_params.ws_theta_base >> 8) & 0xFF;
    buffer[14] = par_params.ws_theta_base & 0xFF;
    buffer[15] = (par_params.ds_theta_current >> 8) & 0xFF;
    buffer[16] = par_params.ds_theta_current & 0xFF;
    buffer[17] = (par_params.ws_theta_current >> 8) & 0xFF;
    buffer[18] = par_params.ws_theta_current & 0xFF;
    buffer[19] = (par_params.ds_rotate_n_data >> 8) & 0xFF;
    buffer[20] = par_params.ds_rotate_n_data & 0xFF;
    buffer[21] = (par_params.ws_rotate_n_data >> 8) & 0xFF;
    buffer[22] = par_params.ws_rotate_n_data & 0xFF;
    buffer[23] = (par_params.ds_iq_data >> 8) & 0xFF;
    buffer[24] = par_params.ds_iq_data & 0xFF;
    buffer[25] = (par_params.ws_iq_data >> 8) & 0xFF;
    buffer[26] = par_params.ws_iq_data & 0xFF;
    buffer[27] = (par_params.board_temperature >> 8) & 0xFF;
    buffer[28] = par_params.board_temperature & 0xFF;
    buffer[29] = (par_params.ds_antenna_temperature >> 8) & 0xFF;
    buffer[30] = par_params.ds_antenna_temperature & 0xFF;
    buffer[31] = (par_params.ws_antenna_temperature >> 8) & 0xFF;
    buffer[32] = par_params.ws_antenna_temperature & 0xFF;
    buffer[33] = (par_params.panel_temperature >> 8) & 0xFF;
    buffer[34] = par_params.panel_temperature & 0xFF;
    buffer[35] = (par_params.strip_gate_line_left_1 >> 8) & 0xFF;
    buffer[36] = par_params.strip_gate_line_left_1 & 0xFF;
    buffer[37] = (par_params.strip_gate_line_right_1 >> 8) & 0xFF;
    buffer[38] = par_params.strip_gate_line_right_1 & 0xFF;
    buffer[39] = (par_params.ds_strip_edge_line >> 8) & 0xFF;
    buffer[40] = par_params.ds_strip_edge_line & 0xFF;
    buffer[41] = (par_params.ws_strip_edge_line >> 8) & 0xFF;
    buffer[42] = par_params.ws_strip_edge_line & 0xFF;
    buffer[43] = (par_params.iq_theta_long_time_count >> 8) & 0xFF;
    buffer[44] = par_params.iq_theta_long_time_count & 0xFF;
    buffer[45] = (par_params.error >> 8) & 0xFF;
    buffer[46] = par_params.error & 0xFF;
    buffer[47] = (par_params.receive_command >> 8) & 0xFF;
    buffer[48] = par_params.receive_command & 0xFF;
    buffer[49] = (par_params.strip_gate_line_left_2 >> 8) & 0xFF;
    buffer[50] = par_params.strip_gate_line_left_2 & 0xFF;
    buffer[51] = (par_params.strip_gate_line_right_2 >> 8) & 0xFF;
    buffer[52] = par_params.strip_gate_line_right_2 & 0xFF;
    buffer[53] = (par_params.ws_distance_from_zero_position >> 8) & 0xFF;
    buffer[54] = par_params.ws_distance_from_zero_position & 0xFF;
    buffer[55] = (par_params.ds_distance_from_zero_position >> 8) & 0xFF;
    buffer[56] = par_params.ds_distance_from_zero_position & 0xFF;
    buffer[57] = (par_params.ws_zero_position >> 8) & 0xFF;
    buffer[58] = par_params.ws_zero_position & 0xFF;
    buffer[59] = (par_params.ds_zero_position >> 8) & 0xFF;
    buffer[60] = par_params.ds_zero_position & 0xFF;
    buffer[61] = (par_params.ws_roll_out >> 8) & 0xFF;
    buffer[62] = par_params.ws_roll_out & 0xFF;
    buffer[63] = (par_params.ds_roll_out >> 8) & 0xFF;
    buffer[64] = par_params.ds_roll_out & 0xFF;
    buffer[65] = (par_params.theta_offset >> 8) & 0xFF;
    buffer[66] = par_params.theta_offset & 0xFF;
    buffer[67] = (par_params.filter_a >> 8) & 0xFF;
    buffer[68] = par_params.filter_a & 0xFF;
    buffer[69] = (par_params.filter_c >> 8) & 0xFF;
    buffer[70] = par_params.filter_c & 0xFF;
    buffer[71] = (par_params.wait_counter >> 8) & 0xFF;
    buffer[72] = par_params.wait_counter & 0xFF;
    buffer[73] = (par_params.iq0_iq1_threshold >> 8) & 0xFF;
    buffer[74] = par_params.iq0_iq1_threshold & 0xFF;
    buffer[75] = (par_params.theta0_theta1_threshold >> 8) & 0xFF;
    buffer[76] = par_params.theta0_theta1_threshold & 0xFF;
    buffer[77] = (par_params.iq_theta_threshold >> 8) & 0xFF;
    buffer[78] = par_params.iq_theta_threshold & 0xFF;
    buffer[79] = (par_params.iq_theta_long_time_count2 >> 8) & 0xFF;
    buffer[80] = par_params.iq_theta_long_time_count2 & 0xFF;
    buffer[81] = (par_params.max_id_number >> 8) & 0xFF;
    buffer[82] = par_params.max_id_number & 0xFF;
    buffer[83] = (par_params.emw_number >> 8) & 0xFF;
    buffer[84] = par_params.emw_number & 0xFF;
    buffer[85] = par_params.sin_gain;
    buffer[86] = par_params.cos_gain;
    buffer[87] = (par_params.data_disposal_count >> 8) & 0xFF;
    buffer[88] = par_params.data_disposal_count & 0xFF;
    buffer[89] = (par_params.data_read_count >> 8) & 0xFF;
    buffer[90] = par_params.data_read_count & 0xFF;
    buffer[91] = (par_params.dirt_warning_level >> 8) & 0xFF;
    buffer[92] = par_params.dirt_warning_level & 0xFF;
    buffer[93] = (par_params.dirt_detect_average >> 8) & 0xFF;
    buffer[94] = par_params.dirt_detect_average & 0xFF;
    buffer[95] = (par_params.dirt_detect_sampling >> 8) & 0xFF;
    buffer[96] = par_params.dirt_detect_sampling & 0xFF;
    /* 字节97–126：reserved 已由 memset 填充0 */
  }

  /* 计算校验和 */
  uint8_t checksum = 0;
  for (int i = 0; i < 127; i++)
  { /* 字节0–126 */
    checksum += buffer[i];
  }
  buffer[127] = checksum;
}
// 准备 1024 字节应答帧（大端格式）
void Prepare_1024Byte_Response(uint8_t *buffer,  uint16_t start_pos)
{

  
  memset(buffer, 0, 1024);
  AdcFrame *adc_params = (AdcFrame *)buffer;

  
  if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    memcpy(adc_params, &par_params, offsetof(AdcFrame, adc_data));
    xSemaphoreGive(paramMutexHandle);
  }

  
  Get_ADC_Data(start_pos, sin_data, cos_data, SAMPLE_COUNT); // sin和cos各SAMPLE_COUNT个数据

  for (uint16_t i = 0; i < SAMPLE_COUNT; i++)
  {

    adc_params->adc_data[i * 2] = sin_data[i];     // sin
    adc_params->adc_data[i * 2 + 1] = cos_data[i]; // cos
  }

 
  adc_params->checksum = 0;

  
  for (uint16_t offset = 3; offset < 49; offset += 2)
  {
    SwapUint16BE(&buffer[offset]);
  }

  uint8_t checksum = 0;
  for (int i = 0; i < 1023; i++)
  {
    checksum += buffer[i];
  }


  adc_params->checksum = checksum;
//  printf("sizeof(AdcFrame) = %zu\n", sizeof(AdcFrame));
}

/* 处理参数设置命令 */
void Handle_Setting_Command(uint8_t *buf, uint8_t cmd_code)
{
  if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    if (cmd_code == 0xE9)
    { /* 参数设置 */
      par_params.header[0] = buf[0];
      par_params.header[1] = buf[1];
      par_params.header[2] = buf[2];
      par_params.strip_gate_line_left_1 = buf[9] << 8 | buf[10];
      par_params.strip_gate_line_right_1 = buf[11] << 8 | buf[12];
      par_params.strip_gate_line_left_2 = buf[9] << 8 | buf[10];
      par_params.strip_gate_line_right_2 = buf[11] << 8 | buf[12]; // 是否重复
      par_params.ws_distance_from_zero_position = buf[13] << 8 | buf[14];
      par_params.ds_distance_from_zero_position = buf[15] << 8 | buf[16];
      par_params.ws_zero_position = buf[17] << 8 | buf[18];
      par_params.ds_zero_position = buf[19] << 8 | buf[20];
      par_params.ws_roll_out = buf[21] << 8 | buf[22];
      par_params.ds_roll_out = buf[23] << 8 | buf[24];
      par_params.theta_offset = buf[25] << 8 | buf[26];
      par_params.filter_a = buf[27] << 8 | buf[28];
      par_params.filter_c = buf[29] << 8 | buf[30];
      par_params.wait_counter = buf[31] << 8 | buf[32];
      par_params.iq0_iq1_threshold = buf[33] << 8 | buf[34];
      par_params.theta0_theta1_threshold = buf[35] << 8 | buf[36];
      par_params.iq_theta_threshold = buf[37] << 8 | buf[38];
      par_params.iq_theta_long_time_count = buf[39] << 8 | buf[40];
      par_params.iq_theta_long_time_count2 = buf[39] << 8 | buf[40]; // count和count2是否重复
      par_params.max_id_number = buf[41] << 8 | buf[42];
      par_params.emw_number = buf[43] << 8 | buf[44];
      par_params.sin_gain = buf[45];
      par_params.cos_gain = buf[46];
      par_params.data_disposal_count = buf[47] << 8 | buf[48];
      par_params.data_read_count = buf[49] << 8 | buf[50];
      par_params.dirt_warning_level = buf[51] << 8 | buf[52];
      par_params.dirt_detect_average = buf[53] << 8 | buf[54];
      par_params.dirt_detect_sampling = buf[55] << 8 | buf[56];

      Save_Parameter(); /* 保存所有参数 */
    }
    else if (cmd_code == 0xEA)
    { /* STRIP GATE LINE LEFT */
      par_params.strip_gate_line_left_1 = buf[9] << 8 | buf[10];
      par_params.strip_gate_line_left_2 = buf[9] << 8 | buf[10];
			
      Save_Parameter();

    }
    else if (cmd_code == 0xEB)
    { /* STRIP GATE LINE RIGHT */
      par_params.strip_gate_line_right_1 = buf[9] << 8 | buf[10];
      par_params.strip_gate_line_right_2 = buf[9] << 8 | buf[10];
      Save_Parameter();
    }
    else if (cmd_code == 0xEC)
    {                    
      uint16_t peak_offset=0;
      peak_offset = (buf[9] << 8 | buf[10])/6 ;                     
      par_params.ws_zero_position = peak_pos - peak_offset; 
      par_params.ws_theta_base = par_params.ws_theta_current;
      par_params.ws_distance_from_zero_position = buf[9] << 8 | buf[10];
      par_params.ws_base_position = buf[9] << 8 | buf[10];
      par_params.ws_strip_edge_position = buf[9] << 8 | buf[10];
      reset_phase_data();

      Save_Parameter();
    }
    else if (cmd_code == 0xED)
    {
      uint16_t peak_offset=0;
      peak_offset = (buf[9] << 8 | buf[10])/6 ;                     
      par_params.ds_zero_position = peak_pos - peak_offset;
      par_params.ds_theta_base = par_params.ds_theta_current;
      par_params.ds_distance_from_zero_position = buf[9] << 8 | buf[10];
      par_params.ds_base_position = buf[9] << 8 | buf[10];
      par_params.ds_strip_edge_position = buf[9] << 8 | buf[10];
      reset_phase_data();

      Save_Parameter();
    }
    else if (cmd_code == 0xE8)
    { /* BASE UPDATE */
      if (buf[4] == 0)
      {
        gst_rsp_parameter.ds_phase_offset_sum = 0;
        par_params.ds_base_position = par_params.ds_iq_data;
        par_params.ds_strip_edge_position = par_params.ds_iq_data;
        Save_Parameter();
      }
    }
    else if (cmd_code == 0xFC)
    { /* GAIN SET */
      par_params.sin_gain = buf[9];
      par_params.cos_gain = buf[10];
      spi2_write_gain();
      Save_Parameter();
    }

    xSemaphoreGive(paramMutexHandle);
  }
}

/* 处理参数读取命令 */
void Handle_Parameter_Read(uint8_t cmd_code)
{
  
  if (xSemaphoreTake(paramMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    Prepare_128Byte_Response(g_usb_usart_tx_buffer, cmd_code);

    
    if (xSemaphoreTake(usbBufferMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      uint8_t retries = 1;
      while (retries-- > 0)
      {
        if (CDC_Transmit_FS(g_usb_usart_tx_buffer, 128) == USBD_OK)
        {
          break;
        }
      }
      xSemaphoreGive(usbBufferMutexHandle);
    }

    xSemaphoreGive(paramMutexHandle);
  }
}

/* 处理ADC采集命令 */
void Handle_ADC_Collect(uint8_t adc_ws_ds_flag, uint16_t start_pos)
{

  
  if (xSemaphoreTake(usbBufferMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    // 根据 adc_ws_ds_flag 选择对应的缓冲区
    uint8_t *tx_buffer = (adc_ws_ds_flag == DEVICE_SIDE_WS) ? g_usb_usart_ws_tx_buffer : g_usb_usart_ds_tx_buffer;
    if(adc_ws_ds_flag == ws_ds_flag){   //相同才能写对应的buffer
      Prepare_1024Byte_Response(tx_buffer, start_pos);   
    }

    uint8_t retries = 3;
    while (retries-- > 0)
    {
      if (CDC_Transmit_FS(tx_buffer, 1024) == USBD_OK)
      {
        break;
      }
      osDelay(1);
    }
    xSemaphoreGive(usbBufferMutexHandle);
  }
  osDelay(3);
}

void Save_Parameter(void)
{
  static uint8_t buffer[sizeof(ParameterFrame)];

  
  memset(par_params.reserved, 0, sizeof(par_params.reserved));

  
  memcpy(buffer, &par_params, sizeof(ParameterFrame));

  HAL_FLASH_Unlock();


  
  if (memcmp(buffer, (void *)ADDR_FLASH_SECTOR_5, sizeof(ParameterFrame)) != 0)
  {
		__disable_irq();
    FLASH_Erase_Sector(FLASH_SECTOR_5, VOLTAGE_RANGE_3);
    for (uint32_t i = 0; i < sizeof(ParameterFrame); i += 4)
    {
      uint32_t addr = ADDR_FLASH_SECTOR_5 + i;
      uint32_t data_word = *(uint32_t *)(buffer + i);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data_word);
    }
//    printf("参数已写入 Flash\r\n");
		__enable_irq();
  }
  else
  {
//    printf("参数未变化，不写入\r\n");
  }

  
  HAL_FLASH_Lock();
}

/* 从Flash加载参数 */
void Load_Parameters(void)
{
  memcpy(&par_params, (void *)ADDR_FLASH_SECTOR_5, sizeof(ParameterFrame));
  memset(par_params.reserved, 0, sizeof(par_params.reserved)); // 清空 reserved 区域

  /* 诊断输出 */
  printf("[LOAD_PARAM] par_params地址: 0x%08X, 大小: %d\r\n", (uint32_t)&par_params, (int)sizeof(ParameterFrame));
  printf("[LOAD_PARAM] sin_gain=%d, cos_gain=%d\r\n", par_params.sin_gain, par_params.cos_gain);
}

/* 校验和验证 */
uint8_t Verify_Checksum(uint8_t *buf, uint16_t len)
{
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len - 1; i++)
  {
    sum += buf[i];
  }
  return sum == buf[len - 1];
}


void Get_ADC_Data(uint16_t start_pos, uint8_t *sin_data, uint8_t *cos_data, uint16_t count)
{
#if MYDEBUG
  for (uint16_t i = 0; i < count; i++)
  {
    sin_data[i] = 0xAAFF; /* 示例数据，替换为实际ADC sin值 */
    cos_data[i] = 0xBBCC; /* 示例数据，替换为实际ADC cos值 */
  }

#else
  // 由于DS和WS共用SRAM交替覆盖，直接从基地址读取当前数据
  read_sram_sincos_data(SIN_FLAG, start_pos, count);
  read_sram_sincos_data(COS_FLAG, start_pos, count);
#endif
}

