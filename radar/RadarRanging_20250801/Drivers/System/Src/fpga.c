
/**
 ****************************************************************************************************
 * @file        fpga.c
 * @author
 * @version     V1.1
 * @date        2025-03-20
 * @brief       FPGA初始化代码
 * @license     Copyright (c) 2020-2032, 常州潞城
 ****************************************************************************************************

*/
#include "fpga.h"

uint8_t fpga_read_flag;

/**
 * @brief       初始化 FGPA
 * @param       无
 * @retval      无
 */
void fpga_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FPGA_CTRL_GPIO_CLK_ENABLE();    /* 使能GPIOE时钟 */
    FPGA_RUN_GPIO_CLK_ENABLE();     /* 使能GPIOF时钟 */

    gpio_init_struct.Pin = FPGA_CTRL_SIDE_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FPGA_CTRL_GPIO_PORT, &gpio_init_struct); /* SRAM_CS引脚模式设置 */

    gpio_init_struct.Pin = FPGA_RUN_GPIO_PIN | FPGA_RAM_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(FPGA_RUN_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = FPGA_CTRL_READ_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_RISING;            /* 上升沿触发 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(FPGA_CTRL_GPIO_PORT, &gpio_init_struct);  /* PE6 引脚模式设置 */

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


    FPGA_RUN_FLAG(0);
	HAL_GPIO_WritePin(FPGA_CTRL_GPIO_PORT, FPGA_CTRL_READ_PIN, GPIO_PIN_RESET);
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void FPGA_READ_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI9_5_IRQn 0 */

    /* USER CODE END EXTI9_5_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(FPGA_CTRL_READ_PIN);
    /* USER CODE BEGIN EXTI9_5_IRQn 1 */

    /* USER CODE END EXTI9_5_IRQn 1 */
}
 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case FPGA_CTRL_READ_PIN:
            fpga_read_flag = 1;
            break;
        default : break;
    }
}

