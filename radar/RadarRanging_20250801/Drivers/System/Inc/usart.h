/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      
 * @version     V1.1
 * @date        2023-06-05
 * @brief       串口初始化代码(一般是串口1)，支持printf
 * @license     Copyright (c) 2020-2032, 常州潞城
 ****************************************************************************************************
 * 修改说明
 * V1.0 20250317
 * 第一次发布
 ****************************************************************************************************
 */

 #ifndef _USART_H
 #define _USART_H
 
 #include "stdio.h"
 #include "sys.h"
 #include "stm32f4xx_hal.h"
 
 /*******************************************************************************************************/
 /* 引脚 和 串口 定义 
  * 默认是针对USART6的.
  * 注意: 通过修改这12个宏定义,可以支持USART6~UART7任意一个串口.
  */

#define USART_TX_GPIO_PORT              GPIOC
#define USART_TX_GPIO_PIN               GPIO_PIN_6
#define USART_TX_GPIO_AF                GPIO_AF8_USART6
#define USART_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* 发送引脚时钟使能 */

#define USART_RX_GPIO_PORT              GPIOC
#define USART_RX_GPIO_PIN               GPIO_PIN_7
#define USART_RX_GPIO_AF                GPIO_AF8_USART6
#define USART_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* 接收引脚时钟使能 */

#define USART_UX                        USART6
#define USART_UX_IRQn                   USART6_IRQn
#define USART_UX_IRQHandler             USART6_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ __HAL_RCC_USART6_CLK_ENABLE(); }while(0)  /* USART6 时钟使能 */

/*******************************************************************************************************/

#define USART_REC_LEN   (1024)              	/* 定义最大接收字节数 1 KB */
#define USART_EN_RX     1                       /* 使能（1）/禁止（0）串口1接收 */
#define RXBUFFERSIZE    1                       /* 缓存大小 */

extern UART_HandleTypeDef g_uart6_handle;       /* UART句柄 */

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 */
extern uint16_t g_usart_rx_sta;                 /* 接收状态标记 */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL库USART接收Buffer */


void usart_init(uint32_t baudrate);             /* 串口初始化函数 */
HAL_StatusTypeDef usart_send(uint8_t *pData, uint16_t Size);		/* 串口发送函数 */

void usart_printf(char *fmt, ...);
#endif

