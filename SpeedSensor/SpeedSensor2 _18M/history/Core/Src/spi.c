/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = SCK_Pin|DIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DOUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DOUT_GPIO_Port, &GPIO_InitStruct);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, SCK_Pin|DOUT_Pin|DIN_Pin);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static HAL_StatusTypeDef SPI_Transmit(uint8_t* send_buf, uint16_t size)
{
    return HAL_SPI_Transmit(&hspi1, send_buf, size, 100);
}
static HAL_StatusTypeDef SPI_Receive(uint8_t* recv_buf, uint16_t size)
{
   return HAL_SPI_Receive(&hspi1, recv_buf, size, 100);
}
HAL_StatusTypeDef SPI_DAC8560_Write(uint8_t DATA_CMD, uint16_t DATA)
{
    uint8_t DATA_H = DATA >> 8;  // 分离高8位
    uint8_t DATA_L = DATA & 255; // 分离低8位
    uint8_t send_data[3] = {DATA_CMD, DATA_H, DATA_L};   // 待发送数据，命令 + 数据
    
    /* 使能片选 */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);  
    
    /* 发送并读取数据 */
    HAL_StatusTypeDef status = SPI_Transmit(send_data, 3);
    
    /* 取消片选 */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);  
    
    return status;
}

HAL_StatusTypeDef SPI_DAC8560_Read(uint8_t DATA_CMD, uint16_t Address, uint16_t* Read_DATA)
{
    uint8_t Address_H = Address >> 8;  // 分离高8位
    uint8_t Address_L = Address & 255; // 分离低8位
    uint8_t send_data[3] = {DATA_CMD, Address_H, Address_L};   // 待发送数据，命令 + 地址
    uint8_t recv_buf[2] = {0}; // 接收缓冲区
    
    /* 使能片选 */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    
    /* 发送地址并读取数据 */
    HAL_StatusTypeDef status = SPI_Transmit(send_data, 3);
    if (status == HAL_OK) 
    {
        status = SPI_Receive(recv_buf, 2);
        if (status == HAL_OK) 
        {
            *Read_DATA = (recv_buf[0] << 8) | recv_buf[1]; // 将接收到的高低字节合并为16位数据
        }
    }
    
    /* 取消片选 */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    
    return status;  // 返回传输状态
}
/* USER CODE END 1 */
