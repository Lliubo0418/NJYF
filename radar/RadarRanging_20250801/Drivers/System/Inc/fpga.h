/**
 ****************************************************************************************************
 * @file        fpga.h
 * @author      
 * @version     V1.1
 * @date        2023-06-05
 * @brief       FPGA初始化代码
 * @license     Copyright (c) 2020-2032, 常州潞城
 ****************************************************************************************************
 * 修改说明
 * V1.0 20250320
 * 第一次发布
 ****************************************************************************************************
 */

#ifndef __FPGA_H
#define __FPGA_H

#include "sys.h"

#define FPGA_CTRL_GPIO_PORT             GPIOE
#define FPGA_CTRL_SIDE_PIN              GPIO_PIN_5
#define FPGA_CTRL_READ_PIN              GPIO_PIN_6
#define FPGA_CTRL_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define FPGA_SIDE_FLAG                  HAL_GPIO_ReadPin(FPGA_CTRL_GPIO_PORT, FPGA_CTRL_SIDE_PIN)     /* 读取FPGA_CTRL_SIDE_PIN引脚 */

#define FPGA_RUN_GPIO_PORT              GPIOF
#define FPGA_RUN_GPIO_PIN               GPIO_PIN_7
#define FPGA_RAM_GPIO_PIN               GPIO_PIN_8
#define FPGA_RUN_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define FPGA_RUN_FLAG(x)      do{ (x) ? \
    HAL_GPIO_WritePin(FPGA_RUN_GPIO_PORT, FPGA_RUN_GPIO_PIN, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(FPGA_RUN_GPIO_PORT, FPGA_RUN_GPIO_PIN, GPIO_PIN_RESET); \
}while(0)

#define FPGA_RAM_FLAG(x)      do{ (x) ? \
    HAL_GPIO_WritePin(FPGA_RUN_GPIO_PORT, FPGA_RAM_GPIO_PIN, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(FPGA_RUN_GPIO_PORT, FPGA_RAM_GPIO_PIN, GPIO_PIN_RESET); \
}while(0)


#define   FPGA_READ_IRQHandler     EXTI9_5_IRQHandler

extern     uint8_t fpga_read_flag;

void fpga_init(void);

#endif  /* End of __FPGA_H */
