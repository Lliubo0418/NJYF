/**
 * *****************************************************************************
 * @file    bsp_rs485.c
 * @brief   rs485 模块文件
 * *****************************************************************************
 */

/** Includes ---------------------------------------------------------------- */
#include "bsp_rs485.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "usart.h"
#if (BSP_RS485_EN)

/** Debug ------------------------------------------------------------------- */
#define DEBUG_EN                    1

#if (DEBUG_EN) && (GLOBAL_DEBUG_EN)
    #define PRINTF(...)             printf(__VA_ARGS__)
#else
    #define PRINTF(...)
#endif

/** Defines ----------------------------------------------------------------- */

/** Variables --------------------------------------------------------------- */
/* RS485_1 */
bsp_rs485_t         rs485_1;
uint8_t             rs485_1_rxbuf[RS485_1_RX_BUF_SIZE];
uint8_t             rs485_1_txbuf[RS485_1_TX_BUF_SIZE];



/** Functions --------------------------------------------------------------- */
static int bsp_rs485_var_init(void);
static int bsp_rs485_msp_init(void);

/**
 * @brief   rs485 初�?�化
 * @param   none
 * @return  0 - OK; other - Error
 */
void bsp_rs485_init(void)
{
    bsp_rs485_var_init();
    bsp_rs485_msp_init();
    
    if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
   
}

/**
 * @brief   var init
 * @param   none
 * @return  none
 */
static int bsp_rs485_var_init(void)
{
    /* RS485_1 */
    rs485_1.rs485 = USART1;
    rs485_1.txbuf = rs485_1_txbuf;
    rs485_1.txbufsize = RS485_1_TX_BUF_SIZE;
    rs485_1.txread = 0;
    rs485_1.txwrite = 0;
    rs485_1.txcount = 0;
    rs485_1.rxbuf = rs485_1_rxbuf;
    rs485_1.rxbufsize = RS485_1_RX_BUF_SIZE;
    rs485_1.rxread = 0;
    rs485_1.rxwrite = 0;
    rs485_1.rxcount = 0;
    rs485_1.sending = 0;



    return 0;
}

/**
 * @brief   rs485 msp init
 * @param   none
 * @return  none
 */
static int bsp_rs485_msp_init(void)
{
    GPIO_InitTypeDef    gpio_init_cfg = {0};
    UART_HandleTypeDef  h_rs485;

    /* RS485_1 */
    /* Clock enable */
    RS485_1_CLK_ENABLE();
    RS485_1_TX_CLK_ENABLE();
    RS485_1_RX_CLK_ENABLE();

    /* Pins config */
    gpio_init_cfg.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_cfg.Mode = GPIO_MODE_AF_PP;
    gpio_init_cfg.Pull = GPIO_PULLUP;
    gpio_init_cfg.Pin = RS485_1_TX_PIN;
//    gpio_init_cfg.Alternate = RS485_1_TX_AF;
    HAL_GPIO_Init(RS485_1_TX_PORT, &gpio_init_cfg);
    gpio_init_cfg.Pin = RS485_1_RX_PIN;
//    gpio_init_cfg.Alternate = RS485_1_RX_AF;
    HAL_GPIO_Init(RS485_1_RX_PORT, &gpio_init_cfg);

    /* Baudrate config */
    h_rs485.Instance = USART1;
    h_rs485.Init.BaudRate = RS485_1_BAUDRATE;
    h_rs485.Init.Parity = UART_PARITY_NONE;
    h_rs485.Init.Mode = UART_MODE_TX_RX;
    h_rs485.Init.WordLength = UART_WORDLENGTH_8B;
    h_rs485.Init.StopBits = UART_STOPBITS_1;
    h_rs485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h_rs485.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&h_rs485);

    /* Int config */
    HAL_NVIC_SetPriority(USART1_IRQn, RS485_1_INT_PRIO, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    CLEAR_BIT(USART1->SR, USART_SR_TC);
    CLEAR_BIT(USART1->SR, USART_SR_RXNE);
    SET_BIT(USART1->CR1, USART_CR1_RXNEIE);



    return 0;
}

/**
 * @brief   从串口缓冲区读取一字节数据
 * @param   prs485 - 串口
 * @param   存放读取到的数据
 * @return  0 - 无数�?; 1 - 有数�?
 */
static uint8_t bsp_rs485_uart_get_char(bsp_rs485_t *prs485, uint8_t *pbyte)
{
    uint16_t    count;

    bsp_mcu_irq_disable();
    count = prs485->rxcount;
    bsp_mcu_irq_enable();

    if (count == 0)
    {
        return 0;
    }
    else
    {
        *pbyte = prs485->rxbuf[prs485->rxread];
        bsp_mcu_irq_disable();
        if (++prs485->rxread >= prs485->rxbufsize)
            prs485->rxread = 0;
        prs485->rxcount--;
        bsp_mcu_irq_enable();

        return 1;
    }
}

/**
 * @brief   发送数�?
 * @param   prs485 - 串口指针
 * @param   buf - 发送数�?缓冲�?
 * @param   len - 发送数�?长度
 * @return  none
 */
static void bsp_rs485_uart_send_buf(bsp_rs485_t *prs485, uint8_t *buf, uint16_t len)
{
    uint16_t    i;

    for (i = 0; i < len; i++)
    {
        prs485->rs485->DR = buf[i];
        while ((prs485->rs485->SR & USART_SR_TC) == 0);
    }
}

/**
 * @brief   �?�?服务
 * @param   prs485 - 串口指针
 * @return  none
 */
static void bsp_rs485_irq_handler(bsp_rs485_t *prs485)
{
    uint32_t    isrflags = READ_REG(prs485->rs485->SR);
    uint32_t    cr1its = READ_REG(prs485->rs485->CR1);

    if ((isrflags & USART_SR_RXNE) != RESET)
    {
        uint8_t dat = READ_REG(prs485->rs485->DR);
        prs485->rxbuf[prs485->rxwrite] = dat;
        if (++prs485->rxwrite >= prs485->rxbufsize)
            prs485->rxwrite = 0;
        if (prs485->rxcount < prs485->rxbufsize)
            prs485->rxcount++;
    }

    if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
        if (prs485->txcount == 0)
        {
            CLEAR_BIT(prs485->rs485->CR1, USART_CR1_TXEIE);
            SET_BIT(prs485->rs485->CR1, USART_CR1_TCIE);
        }
        else
        {
            prs485->sending = 1;
            prs485->rs485->DR = prs485->txbuf[prs485->txread];
            if (++prs485->txread >= prs485->txbufsize)
                prs485->txread = 0;
            prs485->txcount--;
        }
    }

    if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
    {
        if (prs485->txcount == 0)
        {
            CLEAR_BIT(prs485->rs485->CR1, USART_CR1_TCIE);
            prs485->sending = 0;
        }
        else
        {
            prs485->rs485->DR = prs485->txbuf[prs485->txread];
            if (++prs485->txread >= prs485->txbufsize)
                prs485->txread = 0;
            prs485->txcount--;
        }
    }
}



void USART1_IRQHandler(void)
{
    bsp_rs485_irq_handler(&rs485_1);
}



/**
 * @brief   rs485_x�?口转�?为rs485x指针
 * @param   rs485_x - �?口号
 * @return  串口指针
 */
static bsp_rs485_t *bsp_rs485x_to_rs485(bsp_rs485_id_t rs485_x)
{
    bsp_rs485_t *prs485 = NULL;

    switch (rs485_x)
    {
        case RS485_1:
        {
            prs485 = &rs485_1;
            break;
        }
        default:
        {
            prs485 = NULL;
            break;
        }
    }

    return prs485;
}

/**
 * @brief   清除接收缓冲�?
 * @param   rs485_x - �?口号
 * @return  0 - OK; other - Error
 */
int bsp_rs485_clear_rxbuf(bsp_rs485_id_t rs485_x)
{
    bsp_rs485_t *prs485;

    prs485 = bsp_rs485x_to_rs485(rs485_x);
    if (prs485 == 0)
        return 1;

    prs485->rxwrite = 0;
    prs485->rxread  = 0;
    prs485->rxcount = 0;
    return 0;
}

/**
 * @brief   清除发送缓冲区
 * @param   rs485_x - �?口号
 * @return  0 - OK; other - Error
 */
int bsp_rs485_clear_txbuf(bsp_rs485_id_t rs485_x)
{
    bsp_rs485_t *prs485;

    prs485 = bsp_rs485x_to_rs485(rs485_x);
    if (prs485 == 0)
        return 1;

    prs485->txread   = 0;
    prs485->txwrite  = 0;
    prs485->txcount  = 0;
    return 0;
}

/**
 * @brief   从接收缓冲区读取一字节数据
 * @param   rs485_x - �?口号
 * @param   pbyte - 读取到的数据
 * @return   0 - �?读取到数�?; 1 - 读取到数�?
 */
int8_t bsp_rs485_get_char(bsp_rs485_id_t rs485_x, uint8_t *pbyte)
{
    bsp_rs485_t *prs485;

    prs485 = bsp_rs485x_to_rs485(rs485_x);
    if (prs485 == 0)
        return 0;
    
    return bsp_rs485_uart_get_char(prs485, pbyte);
}

/**
 * @brief   获取接收缓冲区接收到的字节数
 * @param   rs485_x - �?口号
 * @return  0 - 表示无数�?; 1 - 表示接收到字节数
 */
uint16_t bsp_rs485_get_rxcount(bsp_rs485_id_t rs485_x)
{
    volatile uint16_t count = 0;
    bsp_rs485_t *prs485;

    prs485 = bsp_rs485x_to_rs485(rs485_x);
    if (prs485 == 0)
        return 0;

    bsp_mcu_irq_disable();
    count = prs485->rxcount;
    bsp_mcu_irq_enable();

    return count;
}

/**
 * @brief   串口发送数�?
 * @param   rs485_x - �?口号
 * @param   buf - 发送缓冲区
 * @param   len - 发送数�?长度
 * @return  none
 */
void bsp_rs485_send_buf(bsp_rs485_id_t rs485_x, uint8_t *buf, uint16_t len)
{
    bsp_rs485_t *prs485;

    prs485 = bsp_rs485x_to_rs485(rs485_x);
    if (prs485 == 0)
        return;

    bsp_rs485_uart_send_buf(prs485, buf, len);
}

/**
 * @brief   串口发送一�?字�??
 * @param   rs485_x - �?口号
 * @param   chr - 待发送字�?
 * @return  none
 */
void bsp_rs485_send_char(bsp_rs485_id_t rs485_x, uint8_t chr)
{
    bsp_rs485_send_buf(rs485_x, &chr, 1);
}

/**
 * @brief   串口发送一�?字�?�串
 * @param   rs485_x - �?口号
 * @param   str - 字�?�串指针
 * @return  none
 */
void bsp_rs485_send_str(bsp_rs485_id_t rs485_x, char *str)
{
    bsp_rs485_send_buf(rs485_x, (uint8_t *)str, strlen(str));
}

/** ------------------------------------------------------------------------- */
#endif /* BSP_RS485_EN */
/******************************* (END OF FILE) ********************************/


