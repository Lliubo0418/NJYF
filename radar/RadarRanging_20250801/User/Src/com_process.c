
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
#include "usbd_cdc_interface.h"
#include "com_process.h"
#include <string.h>
#include "stmflash.h"
#include "sram.h"

#define MYDEBUG 0



uint8_t g_rx_buf[RX_BUFFER_LEN];
uint8_t g_tx_buf[TX_BUFFER_LEN];
uint8_t g_adc_buf[ADC_BUFFER_LEN];

uint32_t  g_tx_flag = 0x00;
uint32_t  g_rx_flag = 0x00;
uint8_t sin_data[400], cos_data[400];

uint32_t device_side_flag = DEVICE_SIDE_WS;

extern uint8_t ws_ds_flag;                        /* 0: DS, 1: WS */
extern ParameterFrame g_params,g_temp_params ; /* 参数存储实例 */
extern AdcFrame g_adc_params ; /* ADC响应参数实例 */
/* usb_printf发送缓冲区, 用于vsprintf */
extern uint8_t g_usb_usart_printf_buffer[USB_USART_REC_LEN];

/* USB接收的数据缓冲区,最大USART_REC_LEN个字节,用于USBD_CDC_SetRxBuffer函数 */
extern uint8_t g_usb_rx_buffer[USB_USART_REC_LEN];
extern uint8_t g_usb_tx_buffer[USB_USART_SEN_LEN];
extern uint8_t g_usb_tx_adc_buffer[USB_USART_SEN_LEN];

/* 用类似串口1接收数据的方法,来处理USB虚拟串口接收到的数据 */
extern uint8_t g_usb_usart_rx_buffer[USB_USART_REC_LEN];       /* 接收缓冲,最大USART_REC_LEN个字节 */


/* 填充128字节响应 MSB*/
void Prepare_128Byte_Response(uint8_t *buffer, uint8_t cmd_code)
{
    memset(buffer, 0, 128);
    buffer[0] = 0x01;
    buffer[1] = 0x01;
    buffer[2] = 0xFE;
    /* 填充 ParameterFrame 字段（高位在前，所有字段 2 字节，除 sin_gain 和 cos_gain） */
		buffer[3] = (par_params.ds_strip_edge_position >> 8) & 0xFF;// 高位
    buffer[4] = par_params.ds_strip_edge_position & 0xFF;// 低位
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

    /* 计算校验和 */
    uint8_t checksum = 0;
    for (int i = 0; i < 127; i++) { /* 字节0–126 */
        checksum += buffer[i];
    }
    buffer[127] = checksum;
}
// 准备 1024 字节应答帧（大端格式）
void Prepare_1024Byte_Response(uint8_t *buffer, uint8_t flag, uint16_t start_pos)
{

    // === 1. 初始化 buffer 和结构体指针 ===
    memset(buffer, 0, 1024);
    AdcFrame *adc_params = (AdcFrame *)buffer;

    // === 2. 安全复制结构体字段（小端方式写入结构体）===
    memcpy(adc_params, &par_params, MIN(sizeof(ParameterFrame), offsetof(AdcFrame, adc_data)));

    // === 3. 获取 ADC 数据，填入结构体（小端写入） ===
    Get_ADC_Data(flag, start_pos, sin_data, cos_data, SAMPLE_COUNT);

		for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
				uint8_t sin_val = sin_data[i];
				uint8_t cos_val = cos_data[i];

    // 交替写入 sin、cos 两个字节（高位在前，即大端）
			adc_params->adc_data[i * 2]     = sin_val ;// sin high byte
			adc_params->adc_data[i * 2 + 1] = cos_val;        // cos low byte
}

    // === 4. 清空 checksum 字节，避免参与校验计算 ===
    adc_params->checksum = 0;

    // === 5. 将所有 uint16_t 字段转换为大端（字节3~49） ===
    for (uint16_t offset = 3; offset < 49; offset += 2) {
        SwapUint16BE(&buffer[offset]);
    }
		// 准备数据后打印 reserved 区域
		for (int i = 849; i < 1023; i++) {
				if (buffer[i] != 0) {
						printf("reserved[%d] = 0x%02X\n", i, buffer[i]);
				}
		}
    // === 6. 计算校验和（前 1023 字节）===
    uint8_t checksum = 0;
    for (int i = 0; i < 1023; i++) {
        checksum += buffer[i];
    }

    // === 7. 写入 checksum 字段（结构体方式）===
    adc_params->checksum = checksum;
		printf("sizeof(AdcFrame) = %zu\n", sizeof(AdcFrame));


}


/* 处理参数设置命令 */
void Handle_Setting_Command(uint8_t *buf, uint8_t cmd_code)
{
    
    if (cmd_code == 0xE9) { /* 参数设置 */
				par_params.header[0] = buf[0];
				par_params.header[1] = buf[1];
				par_params.header[2] = buf[2];
        par_params.strip_gate_line_left_1 = buf[9]<<8|buf[10];
        par_params.strip_gate_line_right_1 = buf[11]<<8|buf[12];
        par_params.strip_gate_line_left_2 = buf[9]<<8|buf[10];
				par_params.strip_gate_line_right_2 = buf[11]<<8|buf[12];   //是否重复
        par_params.ws_distance_from_zero_position = buf[13]<<8|buf[14];
				par_params.ds_distance_from_zero_position = buf[15]<<8|buf[16];
				par_params.ws_zero_position = buf[17]<<8|buf[18];
				par_params.ds_zero_position = buf[19]<<8|buf[20];
				par_params.ws_roll_out = buf[21]<<8|buf[22];
				par_params.ds_roll_out = buf[23]<<8|buf[24];
				par_params.theta_offset = buf[25]<<8|buf[26];
        par_params.filter_a = buf[27]<<8|buf[28];
        par_params.filter_c = buf[29]<<8|buf[30];
        par_params.wait_counter = buf[31]<<8|buf[32];
        par_params.iq0_iq1_threshold = buf[33]<<8|buf[34];
        par_params.theta0_theta1_threshold = buf[35]<<8|buf[36];
        par_params.iq_theta_threshold = buf[37]<<8|buf[38];
				par_params.iq_theta_long_time_count = buf[39]<<8|buf[40];
				par_params.iq_theta_long_time_count2 = buf[39]<<8|buf[40];      //count和count2是否重复
        par_params.max_id_number = buf[41]<<8|buf[42];
        par_params.emw_number = buf[43]<<8|buf[44];
        par_params.sin_gain = buf[45];
        par_params.cos_gain = buf[46];
        par_params.data_disposal_count = buf[47]<<8|buf[48];
        par_params.data_read_count = buf[49]<<8|buf[50];
        par_params.dirt_warning_level = buf[51]<<8|buf[52];
        par_params.dirt_detect_average = buf[53]<<8|buf[54];
        par_params.dirt_detect_sampling = buf[55]<<8|buf[56];				
				

        
        Save_Parameter(0, 0); /* 保存所有参数 */
				printf("已更新所有参数\r\n");
    } else if (cmd_code == 0xEA) { /* STRIP GATE LINE LEFT */
        par_params.strip_gate_line_left_1 = buf[9]<<8|buf[10];
        Save_Parameter(1, par_params.strip_gate_line_left_1);
			printf("已更新STRIP GATE LINE LEFT\r\n");
    } else if (cmd_code == 0xEB) { /* STRIP GATE LINE RIGHT */
        par_params.strip_gate_line_right_1 = buf[9]<<8|buf[10];
        Save_Parameter(2, par_params.strip_gate_line_right_1);
			printf("已更新STRIP GATE LINE RIGHT\r\n");
    } else if (cmd_code == 0xEC) { /* WS START POSITION */
        par_params.ws_zero_position = buf[9]<<8|buf[10];
        Save_Parameter(3, par_params.ws_zero_position);
			printf("已更新WS START POSITION\r\n");
    } else if (cmd_code == 0xED) { /* DS START POSITION */
        par_params.ds_zero_position = buf[9]<<8|buf[10];
        Save_Parameter(4, par_params.ds_zero_position);
			printf("已更新DS START POSITION\r\n");
    } else if (cmd_code == 0xE8) { /* BASE UPDATE */
        if (!g_base_updated) {
            Save_Parameter(5, 0);
            g_base_updated = 1;
        }
				printf("BASE UPDATE已更新\r\n");
    } else if (cmd_code == 0xFC) { /* GAIN SET */
        par_params.sin_gain = buf[9];
        par_params.cos_gain = buf[10];
        Save_Parameter(6, (par_params.sin_gain << 8) | par_params.cos_gain);
			printf("GAIN SET已更新\r\n");
    }
		printf("处理0x%02X结束\r\n",cmd_code);
}


/* 处理参数读取命令 */
void Handle_Parameter_Read(uint8_t cmd_code)
{
    Prepare_128Byte_Response(g_usb_tx_buffer, cmd_code);
    udbd_cdc_tx_data(g_usb_tx_buffer, 128);
		delay_us(1000);
}

/* 处理ADC采集命令 */
void Handle_ADC_Collect(uint8_t ws_ds_flag, uint16_t start_pos)
{
    Prepare_1024Byte_Response(g_usb_tx_buffer, ws_ds_flag, start_pos);

    udbd_cdc_tx_data(g_usb_tx_buffer, 1024);
		delay_us(2000);
}

uint32_t data_word;
/* 保存参数到Flash */
void Save_Parameter(uint8_t param_id, uint32_t value) {
    static uint8_t buffer[sizeof(ParameterFrame)];
    HAL_FLASH_Unlock();
		__disable_irq();  // 禁止全局中断
    if (param_id == 0) {
        memcpy(buffer, &par_params, sizeof(ParameterFrame));
        // 仅在数据不同时擦除和写入
        if (memcmp(buffer, (void*)ADDR_FLASH_SECTOR_4, sizeof(ParameterFrame)) != 0) {
            FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);					
						for (uint32_t i = 0; i < 128; i += 4) {
							uint32_t addr = ADDR_FLASH_SECTOR_4 + i;
							data_word = *(uint32_t*)(buffer + i);
							HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data_word);
						}
        }
    } else {
        // 单参数写入，更新缓冲区后写入
					*(uint8_t*)(buffer + param_id * 4) = value;
            FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);           
						for (uint32_t i = 0; i < 128; i += 4) {
							uint32_t addr = ADDR_FLASH_SECTOR_4 + i;
							data_word = *(uint32_t*)(buffer + i);
							HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data_word);
						}
    }
		__enable_irq();   // 恢复中断
    HAL_FLASH_Lock();
}

/* 从Flash加载参数 */
void Load_Parameters(void)
{
    memcpy(&par_params, (void*)ADDR_FLASH_SECTOR_4, sizeof(ParameterFrame));
}

/* 校验和验证 */
uint8_t Verify_Checksum(uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len - 1; i++) {
        sum += buf[i];
    }
    return sum == buf[len - 1];
}

/* 模拟ADC数据生成函数（需替换为实际ADC硬件读取） */
void Get_ADC_Data(uint8_t flag, uint16_t start_pos, uint8_t *sin_data, uint8_t *cos_data, uint16_t count)
{
	#if MYDEBUG
    for (uint16_t i = 0; i < count; i++) {
        sin_data[i] = 0xAAFF; /* 示例数据，替换为实际ADC sin值 */
        cos_data[i] = 0xBBCC; /* 示例数据，替换为实际ADC cos值 */
    }
			
	#else
    // 判断是 WS 还是 DS
    if (flag == DEVICE_SIDE_WS) {
        // 读取 SRAM 中 WS 的数据
        read_sram_sincos_data(SIN_FLAG, DEVICE_SIDE_WS, start_pos, count);
        read_sram_sincos_data(COS_FLAG, DEVICE_SIDE_WS, start_pos, count);
    } else {
        // 读取 SRAM 中 DS 的数据
        read_sram_sincos_data(SIN_FLAG, DEVICE_SIDE_DS, start_pos, count);
        read_sram_sincos_data(COS_FLAG, DEVICE_SIDE_DS, start_pos, count);
    }	
	#endif	
}