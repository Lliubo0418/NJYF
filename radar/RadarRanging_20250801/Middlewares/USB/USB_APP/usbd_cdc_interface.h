/**
 ****************************************************************************************************
 * @file        usbd_cdc_interface.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-01-20
 * @brief       USB CDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20220120
 * 第一次发布
 *
 ****************************************************************************************************
 */

 #ifndef __USBD_CDC_IF_H
 #define __USBD_CDC_IF_H
 
 #include "usbd_cdc.h"
 
 
 #define USB_USART_REC_LEN       1024     /* USB串口接收缓冲区最大字节数 */
 #define USB_USART_SEN_LEN       1024     /* USB串口发送缓冲区最大字节数 */





/* Base Update状态 */
static uint8_t g_base_updated = 0; /* 防止重复发送Base Update */
 
 /* 轮询周期，最大65ms，最小1ms */
 #define CDC_POLLING_INTERVAL    1       /* 轮询周期，最大65ms，最小1ms */
 
 extern uint8_t  g_usb_usart_rx_buffer[USB_USART_REC_LEN];   /* 接收缓冲,最大USB_USART_REC_LEN个字节.末字节为换行符 */
 extern uint16_t g_usb_usart_rx_sta;                         /* 接收状态标记 */
 
 extern USBD_CDC_ItfTypeDef  USBD_CDC_fops;
 
 
 void udbd_cdc_tx_data(uint8_t *buf,uint32_t len);
 void udbd_cdc_rx_data(uint8_t* buf, uint32_t len);
 void usb_printf(char* fmt,...); 
 
 #endif 
 
 
 