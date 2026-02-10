/**
 ****************************************************************************************************
 * @file        usbd_cdc_interface.c
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

#include "string.h"
#include "stdarg.h"
// #include "stdio.h"
#include "usbd_cdc_interface.h"
#include "usart.h"
#include "delay.h"
#include "com_process.h"
#include "sram.h"






#define MAX_RETRIES 3  // 最大重试次数

uint16_t buffer_overflowFlag=0;
extern uint8_t ws_ds_flag;                        /* 0: DS, 1: WS */

/* USB虚拟串口相关配置参数 */
USBD_CDC_LineCodingTypeDef LineCoding =
{
    115200,     /* 波特率 */
    0x00,       /* 停止位,默认1位 */
    0x00,       /* 校验位,默认无 */
    0x08        /* 数据位,默认8位 */
};


/* usb_printf发送缓冲区, 用于vsprintf */
uint8_t g_usb_usart_printf_buffer[USB_USART_REC_LEN];

/* USB接收的数据缓冲区,最大USART_REC_LEN个字节,用于USBD_CDC_SetRxBuffer函数 */
uint8_t g_usb_rx_buffer[USB_USART_REC_LEN]={0};
uint8_t g_usb_tx_buffer[USB_USART_SEN_LEN]={0};
uint8_t g_usb_tx_adc_buffer[USB_USART_SEN_LEN]={0};

/* 用类似串口1接收数据的方法,来处理USB虚拟串口接收到的数据 */
uint8_t g_usb_usart_rx_buffer[USB_USART_REC_LEN];       /* 接收缓冲,最大USART_REC_LEN个字节 */

volatile uint8_t sram_read_flag = 0;



/* 接收状态
 * bit15   , 接收完成标志
 * bit14   , 接收到0x0d
 * bit13~0 , 接收到的有效字节数目
 */
uint16_t g_usb_usart_rx_sta=0;  /* 接收状态标记 */


USBD_HandleTypeDef USBD_Device;
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t *pbuf, uint32_t *Len);


/* 虚拟串口配置函数(供USB内核调用) */
USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
    CDC_Itf_Init,
    CDC_Itf_DeInit,
    CDC_Itf_Control,
    CDC_Itf_Receive
};

/**
 * @brief       初始化 CDC
 * @param       无
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_Itf_Init(void)
{
    USBD_CDC_SetRxBuffer(&USBD_Device, g_usb_rx_buffer);
    return USBD_OK;
}

/**
 * @brief       复位 CDC
 * @param       无
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_Itf_DeInit(void)
{
    return USBD_OK;
}

/**
 * @brief       控制 CDC 的设置
 * @param       cmd     : 控制命令
 * @param       buf     : 命令数据缓冲区/参数保存缓冲区
 * @param       length  : 数据长度
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    switch (cmd)
    {
        case CDC_SEND_ENCAPSULATED_COMMAND:
            break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
            break;

        case CDC_SET_COMM_FEATURE:
            break;

        case CDC_GET_COMM_FEATURE:
            break;

        case CDC_CLEAR_COMM_FEATURE:
            break;

        case CDC_SET_LINE_CODING:
            LineCoding.bitrate = (uint32_t) (pbuf[0] | (pbuf[1] << 8) |
                                             (pbuf[2] << 16) | (pbuf[3] << 24));
            LineCoding.format = pbuf[4];
            LineCoding.paritytype = pbuf[5];
            LineCoding.datatype = pbuf[6];
            // /* 打印配置参数 */
            // printf("linecoding.format:%d\r\n", LineCoding.format);
            // printf("linecoding.paritytype:%d\r\n", LineCoding.paritytype);
            // printf("linecoding.datatype:%d\r\n", LineCoding.datatype);
            // printf("linecoding.bitrate:%d\r\n", LineCoding.bitrate);
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t) (LineCoding.bitrate);
            pbuf[1] = (uint8_t) (LineCoding.bitrate >> 8);
            pbuf[2] = (uint8_t) (LineCoding.bitrate >> 16);
            pbuf[3] = (uint8_t) (LineCoding.bitrate >> 24);
            pbuf[4] = LineCoding.format;
            pbuf[5] = LineCoding.paritytype;
            pbuf[6] = LineCoding.datatype;
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            break;

        case CDC_SEND_BREAK:
            break;

        default:
            break;
    }

    return USBD_OK;
}

/**
 * @brief       CDC 数据接收函数
 * @param       buf     : 接收数据缓冲区
 * @param       len     : 接收到的数据长度
 * @retval      USB状态
 *   @arg       USBD_OK(0)   , 正常;
 *   @arg       USBD_BUSY(1) , 忙;
 *   @arg       USBD_FAIL(2) , 失败;
 */
static int8_t CDC_Itf_Receive(uint8_t *buf, uint32_t *len)
{
    USBD_CDC_ReceivePacket(&USBD_Device);
    udbd_cdc_rx_data(buf, *len);
    return USBD_OK;
}

/**
 * @brief       处理从 USB 虚拟串口接收到的数据
 * @param       buf     : 接收数据缓冲区
 * @param       len     : 接收到的数据长度
 * @retval      无
 */
void udbd_cdc_rx_data(uint8_t *buf, uint32_t len)
{
    // 拷贝进接收缓冲区（支持多次调用累积）
    for (uint32_t i = 0; i < len && g_usb_usart_rx_sta < USB_USART_REC_LEN; i++) {
        g_usb_usart_rx_buffer[g_usb_usart_rx_sta++] = buf[i];
    }

    // 缓冲区溢出保护
    if (g_usb_usart_rx_sta >= USB_USART_REC_LEN) {
        buffer_overflowFlag = 1;
        g_usb_usart_rx_sta = 0;
        memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
        return;
    }

    // 判断是否收到了完整帧
    if (g_usb_usart_rx_sta >= 5) {
        // 判断是否是 5 字节指令帧
        if (g_usb_usart_rx_buffer[0] == 0x15) {
            // 解析指令
            if (g_usb_usart_rx_buffer[1] == 0x80) {
                /* 发送 128 字节命令的握手确认 */
                uint8_t ack = 0x06;
                udbd_cdc_tx_data(&ack, 1);
                /* 清空缓冲区，准备接收 128 字节数据 */
                g_usb_usart_rx_sta = 0;
                memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
                return;
            }
            else if (g_usb_usart_rx_buffer[1] == 0x00 && g_usb_usart_rx_buffer[2] == 0x65) {
                /* 启动命令 */
                Handle_Parameter_Read(0x65);
            }
            else if (g_usb_usart_rx_buffer[1] == 0x00 && g_usb_usart_rx_buffer[2] == 0x01) {
                /* 监测数据命令 */
                Handle_Parameter_Read(0x01);
            }
            else if (g_usb_usart_rx_buffer[1] == 0x4B) {
                /* WS/DS ADC 采集命令 */
                ws_ds_flag = g_usb_usart_rx_buffer[2]; /* 0x01 为 WS，0x00 为 DS */
                uint16_t start_pos = (g_usb_usart_rx_buffer[4] << 8) | g_usb_usart_rx_buffer[5];
                Handle_ADC_Collect(ws_ds_flag, start_pos);
            }
            else if (g_usb_usart_rx_buffer[1] == 0xFF) {
                /* 读取 SRAM */
                sram_read_flag = 1; /* 设置标志位 */
            }

            // 清空状态
            g_usb_usart_rx_sta = 0;
            memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
            buffer_overflowFlag = 0;
            return;
        }
    }

    if (g_usb_usart_rx_sta >= 128) {
        // 判断是否是128字节数据帧头
        if (g_usb_usart_rx_buffer[0] == 0x01 &&
            g_usb_usart_rx_buffer[1] == 0x01 &&
            g_usb_usart_rx_buffer[2] == 0xFE) {
            
//            if (!Verify_Checksum(g_usb_usart_rx_buffer, 128)) {
                uint8_t cmd_code = g_usb_usart_rx_buffer[8];
                Handle_Setting_Command(g_usb_usart_rx_buffer, cmd_code);
//            }

            g_usb_usart_rx_sta = 0;
            memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
            buffer_overflowFlag = 0;
        } else {
            // 不合法帧，丢弃或调试输出
            g_usb_usart_rx_sta = 0;
            memset(g_usb_usart_rx_buffer, 0, USB_USART_REC_LEN);
        }
    }
}


/**
 * @brief       通过 USB 发送数据
 * @param       buf     : 要发送的数据缓冲区
 * @param       len     : 数据长度
 * @retval      无
 */


void udbd_cdc_tx_data(uint8_t *data, uint32_t Len)
{
    uint8_t retries = 0;
    uint8_t status;

    while (retries < MAX_RETRIES) {
        USBD_CDC_SetTxBuffer(&USBD_Device, data, Len);
        status = USBD_CDC_TransmitPacket(&USBD_Device);

        if (status == USBD_OK) {
					// 添加调试信息
					printf("Sent %d bytes\n", Len);
            break;  // 传输成功，退出循环
        }
        retries++;
        delay_ms(100);  // 等待一段时间后重试
    }

    if (retries == MAX_RETRIES) {
        // 达到最大重试次数，处理错误
        printf("数据传输失败，达到最大重试次数！\n");
    }
}

/**
 * @brief       通过 USB 格式化输出函数
 *   @note      通过USB VCP实现printf输出
 *              确保一次发送数据长度不超USB_USART_REC_LEN字节
 * @param       格式化输出
 * @retval      无
 */
void usb_printf(char *fmt, ...)
{
    uint16_t i;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)g_usb_usart_printf_buffer, fmt, ap);
    va_end(ap);
    i = strlen((const char *)g_usb_usart_printf_buffer);    /* 此次发送数据的长度 */
    udbd_cdc_tx_data(g_usb_usart_printf_buffer, i);          /* 发送数据 */
}











