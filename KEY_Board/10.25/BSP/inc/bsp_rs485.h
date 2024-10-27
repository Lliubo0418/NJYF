/**
 * *****************************************************************************
 * @file    bsp_rs485.h
 * @brief   rs485 模块头文�?
 * *****************************************************************************
 */

/** Define to prevent recursive inclusion ----------------------------------- */
#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#ifdef __cplusplus
    extern "C" {
#endif

/** Includes ---------------------------------------------------------------- */
#include "bsp.h"

#ifndef BSP_RS485_EN
    #define BSP_RS485_EN            1
#endif
#if (BSP_RS485_EN)

/** Defines ----------------------------------------------------------------- */
/* Int config */
#define RS485_1_INT_PRIO            10


/* COM id */
typedef enum
{
    RS485_1 = 1,
    RS485_ID_MAX,
} bsp_rs485_id_t;

#define RS485_1_COM                 COM_1


/* Pins config */
/* RS485_1 */
#define RS485_1_CLK_ENABLE()        __HAL_RCC_USART1_CLK_ENABLE()
#define RS485_1_TX_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define RS485_1_TX_PORT             GPIOA
#define RS485_1_TX_PIN              GPIO_PIN_9
// #define RS485_1_TX_AF               GPIO_AF7_USART2
#define RS485_1_RX_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define RS485_1_RX_PORT             GPIOA
#define RS485_1_RX_PIN              GPIO_PIN_10
// #define RS485_1_RX_AF               GPIO_AF7_USART2


/* Uart baudrate and buf size config */
/* RS485_1 */
#define RS485_1_BAUDRATE            9600
#define RS485_1_TX_BUF_SIZE         128
#define RS485_1_RX_BUF_SIZE         128



/** Types ------------------------------------------------------------------- */
typedef struct
{
    USART_TypeDef       *rs485;
    uint8_t             *rxbuf;
    volatile uint16_t   rxbufsize;
    volatile uint16_t   rxread;
    volatile uint16_t   rxwrite;
    volatile uint16_t   rxcount;
    uint8_t             *txbuf;
    volatile uint16_t   txbufsize;
    volatile uint16_t   txread;
    volatile uint16_t   txwrite;
    volatile uint16_t   txcount;
    volatile uint8_t    sending;
} bsp_rs485_t;

/** Variables --------------------------------------------------------------- */
/* RS485_1 */
extern bsp_rs485_t  rs485_1;
extern uint8_t      rs485_1_rxbuf[RS485_1_RX_BUF_SIZE];
extern uint8_t      rs485_1_txbuf[RS485_1_TX_BUF_SIZE];



/** Functions --------------------------------------------------------------- */
void bsp_rs485_init(void);
int bsp_rs485_clear_rxbuf(bsp_rs485_id_t rs485_x);
int bsp_rs485_clear_txbuf(bsp_rs485_id_t rs485_x);
int8_t bsp_rs485_get_char(bsp_rs485_id_t rs485_x, uint8_t *pbyte);
uint16_t bsp_rs485_get_rxcount(bsp_rs485_id_t rs485_x);
void bsp_rs485_send_buf(bsp_rs485_id_t rs485_x, uint8_t *buf, uint16_t len);
void bsp_rs485_send_char(bsp_rs485_id_t rs485_x, uint8_t chr);
void bsp_rs485_send_str(bsp_rs485_id_t rs485_x, char *str);

/** ------------------------------------------------------------------------- */
#endif  /* BSP_RS485_EN */

#ifdef __cplusplus
    }
#endif

#endif  /* __BSP_RS485_H */

/******************************* (END OF FILE) ********************************/

