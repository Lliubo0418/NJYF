/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"
uint8_t Mpc1_Channel_Num = 0; // R1通道
uint8_t Mpc2_Channel_Num = 0; // R2通道
// sync
uint32_t Sync_capture_Buf[3] = {0}; // 存放计数值
uint8_t Sync_Cnt = 0;               // 状态标志位
uint32_t Sync_high_time;            // 高电平时间
// int1
uint16_t Int1_starttime = 0;
uint16_t Int1_endtime = 0;
uint8_t Int1_first_flag = 0;
uint16_t Int1_duration = 0;
// int2
uint16_t Int2_starttime = 0;
uint16_t Int2_endtime = 0;
uint8_t Int2_first_flag = 0;
uint16_t Int2_duration = 0;
// int3
uint16_t Int3_starttime = 0;
uint16_t Int3_endtime = 0;
uint8_t Int3_first_flag = 0;
uint16_t Int3_duration = 0;
// int4
uint16_t Int4_starttime = 0;
uint16_t Int4_endtime = 0;
uint8_t Int4_first_flag = 0;
uint16_t Int4_duration = 0;
extern osSemaphoreId_t HolesCountingSemHandle;

#if 0
//定义结构体，增加代码的可读性，避免重复调用函数
typedef struct {
    uint16_t start_time;
    uint16_t end_time;
    uint16_t duration;
    uint8_t first_flag;
} InterruptCaptureData;

InterruptCaptureData Int1 = {0};
InterruptCaptureData Int2 = {0};
InterruptCaptureData Int3 = {0};
InterruptCaptureData Int4 = {0};
#endif
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
/* TIM4 init function */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}
/* TIM6 init function */
void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */
  /*TIM7每500us溢出一次，用以检测SYNC信号以及判断钢板是否存在*/
  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 500-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}
/* TIM9 init function */
void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 50000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE14     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PB10     ------> TIM2_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */
  /* USER CODE END TIM7_MspInit 0 */
    /* TIM7 clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM9)
  {
  /* USER CODE BEGIN TIM9_MspInit 0 */

  /* USER CODE END TIM9_MspInit 0 */
    /* TIM9 clock enable */
    __HAL_RCC_TIM9_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM9 GPIO Configuration
    PE5     ------> TIM9_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* TIM9 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* USER CODE BEGIN TIM9_MspInit 1 */

  /* USER CODE END TIM9_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PE14     ------> TIM1_CH4
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_14);

    /* TIM1 interrupt Deinit */
  /* USER CODE BEGIN TIM1:TIM1_BRK_TIM9_IRQn disable */
    /**
     * Uncomment the line below to disable the "TIM1_BRK_TIM9_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn); */
  /* USER CODE END TIM1:TIM1_BRK_TIM9_IRQn disable */

    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PB10     ------> TIM2_CH3
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    /* TIM4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM9)
  {
  /* USER CODE BEGIN TIM9_MspDeInit 0 */

  /* USER CODE END TIM9_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM9_CLK_DISABLE();

    /**TIM9 GPIO Configuration
    PE5     ------> TIM9_CH1
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_5);

    /* TIM9 interrupt Deinit */
  /* USER CODE BEGIN TIM9:TIM1_BRK_TIM9_IRQn disable */
    /**
     * Uncomment the line below to disable the "TIM1_BRK_TIM9_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn); */
  /* USER CODE END TIM9:TIM1_BRK_TIM9_IRQn disable */

  /* USER CODE BEGIN TIM9_MspDeInit 1 */

  /* USER CODE END TIM9_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#if 0
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            CaptureHandler(&Int1, htim->Instance->CCR4);
        }
				// 其他定时器和通道类似处理
}	
/*
 *CaptureHandler 函数：抽象出捕获数据处理的通用逻辑，避免重复代码，增强了可读性和可维护性。
 *更好的模块化：通过结构体和函数封装，变量管理和逻辑处理都更加清晰，有助于未来的扩展和维护。
 *追求效率的话不建议封装
 */
void CaptureHandler(InterruptCaptureData *data, uint16_t captured_value) {
    if (data->first_flag == 0) {
        data->start_time = captured_value;
        data->first_flag = 1;
    } else {
        data->end_time = captured_value;
        data->duration = data->end_time - data->start_time;
        data->first_flag = 0;
    }
}

#endif
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
      switch (Sync_Cnt)
      {
      case 1:
        Sync_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3); // 获取当前的捕获值.
        TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_ICPOLARITY_FALLING); // 设置为下降沿捕获
        Sync_Cnt++;
        break;
      case 2:
        Sync_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3); // 获取当前的捕获值.
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);                              // 停止捕获
        Sync_Cnt++;
        break;
      }
    }
  }
#if 0
  //LNOTE:捕获单个脉宽
  else if (htim->Instance == TIM4)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Int1_first_flag == 0)
      {
        /*
         *如果处理时间过长，尝试去除函数封装，直接操作寄存器
         *__HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_4);或Int1_starttime = TIM1->CCR4;直接访问寄存器
         */
        Int1_starttime = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        Int1_first_flag = 1;
      }
      else
      {
        Int1_endtime = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        Int1_first_flag = 0;
        if (Int1_endtime <= Int1_starttime)
        {
          Int1_duration = 50000 + Int1_endtime - Int1_starttime;
          // printf("Int1_Duration: %d us\r\n", Int1_duration);
        }
        else
        {
          Int1_duration = Int1_endtime - Int1_starttime;
          // printf("Int1_Duration: %d us\r\n", Int1_duration);
        }
      }
    }
  }
  else if (htim->Instance == TIM9)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Int2_first_flag == 0)
      {
        Int2_starttime = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
        TIM_RESET_CAPTUREPOLARITY(&htim9, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim9, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        Int2_first_flag = 1;
      }
      else
      {
        Int2_endtime = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
        TIM_RESET_CAPTUREPOLARITY(&htim9, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim9, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        Int2_first_flag = 0;
        if (Int2_endtime <= Int2_starttime)
        {
          Int2_duration = 50000 + Int2_endtime - Int2_starttime;
          // printf("Int2_Duration: %d us\r\n", Int2_duration);
        }
        else
        {
          Int2_duration = Int2_endtime - Int2_starttime;
          // printf("Int2_Duration: %d us\r\n", Int2_duration);
        }
      }
    }
  }
  else if (htim->Instance == TIM3)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Int3_first_flag == 0)
      {
        Int3_starttime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        Int3_first_flag = 1;
      }
      else
      {
        Int3_endtime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        Int3_first_flag = 0;
        if (Int3_endtime <= Int3_starttime)
        {
          Int3_duration = 50000 + Int3_endtime - Int3_starttime;
          // printf("Int3_Duration: %d us\r\n", Int3_duration);
        }
        else
        {
          Int3_duration = Int3_endtime - Int3_starttime;
          // printf("Int3_Duration: %d us\r\n", Int3_duration);
        }
      }
    }
  }
  else if (htim->Instance == TIM1)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
      if (Int4_first_flag == 0)
      {
        Int4_starttime = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
        TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
        Int4_first_flag = 1;
      }
      else
      {
        Int4_endtime = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
        TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
        Int4_first_flag = 0;
        if (Int4_endtime <= Int4_starttime)
        {
          Int4_duration = 50000 + Int4_endtime - Int4_starttime;
          // printf("Int4_Duration: %d us\r\n", Int4_duration);
        }
        else
        {
          Int4_duration = Int4_endtime - Int4_starttime;
          // printf("Int4_Duration: %d us\r\n", Int4_duration);
        }
      }
    }
  }
#endif
#if 1
  // LNOTE:捕获下降沿到下降沿，此时为捕获到两个脉冲，即一个孔
  // LXXX:查看对应定时器初始化时IC是否为下降沿

  else if (htim->Instance == TIM4)
  {
    BaseType_t xHigherPriorityTaskWoken;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Int1_first_flag == 0)
      {
        /*
         *如果处理时间过长，尝试去除函数封装，直接操作寄存器
         *__HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_4);或Int1_starttime = TIM1->CCR4;直接访问寄存器
         */
        Int1_starttime = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        Int1_first_flag = 1;
      }
      else
      {
        Int1_endtime = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        Int1_first_flag = 0;
        if (Int1_endtime <= Int1_starttime)
        {
          Int1_duration = 50000 + Int1_endtime - Int1_starttime;
        }
        else
        {
          Int1_duration = Int1_endtime - Int1_starttime;
        }
      }
      if (Int1_duration == 0x05&&(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED))
      {
        xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
  else if (htim->Instance == TIM9)
  {
    BaseType_t xHigherPriorityTaskWoken;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Int2_first_flag == 0)
      {
        Int2_starttime = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
        Int2_first_flag = 1;
      }
      else
      {
        Int2_endtime = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
        Int2_first_flag = 0;
        if (Int2_endtime <= Int2_starttime)
        {
          Int2_duration = 50000 + Int2_endtime - Int2_starttime;
        }
        else
        {
          Int2_duration = Int2_endtime - Int2_starttime;
        }
      }
      if (Int2_duration == 0x05&&(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED))
      {
        xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
  else if (htim->Instance == TIM3)
  {
    BaseType_t xHigherPriorityTaskWoken;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Int3_first_flag == 0)
      {
        Int3_starttime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        Int3_first_flag = 1;
      }
      else
      {
        Int3_endtime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        Int3_first_flag = 0;
        if (Int3_endtime <= Int3_starttime)
        {
          Int3_duration = 50000 + Int3_endtime - Int3_starttime;
        }
        else
        {
          Int3_duration = Int3_endtime - Int3_starttime;
        }
      }
      if (Int3_duration == 0x05&&(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED))
      {
        xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
  else if (htim->Instance == TIM1)
  {
    BaseType_t xHigherPriorityTaskWoken;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (Int4_first_flag == 0)
      {
        Int4_starttime = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
        Int4_first_flag = 1;
      }
      else
      {
        Int4_endtime = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
        Int4_first_flag = 0;
        if (Int4_endtime <= Int4_starttime)
        {
          Int4_duration = 50000 + Int4_endtime - Int4_starttime;
        }
        else
        {
          Int4_duration = Int4_endtime - Int4_starttime;
        }
      }
      if (Int4_duration == 0x05&&(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED))
      {
        xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM13)
  {
    HAL_IncTick();
  }
  if (htim->Instance == TIM6)
  {
                                         //开启SYNC捕获
    Mpc1_Channel_Num = (Mpc1_Channel_Num + 1) % 16;
    Mpc2_Channel_Num = (Mpc2_Channel_Num + 1) % 16;
    R1Channel_Sel(Mpc1_Channel_Num);
    R2Channel_Sel(Mpc2_Channel_Num);
    if (Mpc1_Channel_Num == 0 && Mpc2_Channel_Num == 0)
    {
			Sync_Cnt=0;
      HAL_TIM_Base_Stop_IT(&htim6); // Stop timer for switch
      EN_R1_R2_close();             // 关闭接收
      //LTODO: 查看宏定义能否生效
      // __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
      // __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
      // __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
      // __HAL_TIM_DISABLE_IT(&htim9, TIM_IT_CC1);
    HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
    }
  }
#if 0
  if (htim->Instance == TIM7)
  {
    // LTODO: 孔的个数处理，500ms减一，释放二值信号量给继电器，PNP.NPN任务
  }

#endif //
}
/* USER CODE END 1 */
