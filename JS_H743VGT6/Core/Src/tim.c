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
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "gpio.h"
#include "event_groups.h"
extern osSemaphoreId_t HolesCountingSemHandle;
extern osThreadId_t Hole_ldentificaHandle;

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

// 根据镜片宽度设定循环次数
uint8_t circulation = 0;



extern osEventFlagsId_t PositionEvent1_MPC1Handle;
extern osEventFlagsId_t PositionEvent2_MPC1Handle;
extern osEventFlagsId_t PositionEvent3_MPC1Handle;
extern osEventFlagsId_t PositionEvent4_MPC1Handle;
extern osEventFlagsId_t PositionEvent1_MPC2Handle;
extern osEventFlagsId_t PositionEvent2_MPC2Handle;
extern osEventFlagsId_t PositionEvent3_MPC2Handle;
extern osEventFlagsId_t PositionEvent4_MPC2Handle;

uint8_t IsMpc1 = 1;

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 240-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
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
  htim2.Init.Prescaler = 240-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000-1;
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
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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
  htim3.Init.Prescaler = 240-1;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
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
  htim4.Init.Prescaler = 240-1;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
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
  htim6.Init.Prescaler = 240-1;
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

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 240-1;
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
/* TIM8 init function */
void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 240-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 50000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}
/* TIM15 init function */
void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 240-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 50000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_icHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE14     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = Sync_Int_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(Sync_Int_GPIO_Port, &GPIO_InitStruct);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PB11     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = Single_Int4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(Single_Int4_GPIO_Port, &GPIO_InitStruct);

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

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PC6     ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = Single_Int3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(Single_Int3_GPIO_Port, &GPIO_InitStruct);

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
    GPIO_InitStruct.Pin = Single_Int1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(Single_Int1_GPIO_Port, &GPIO_InitStruct);

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
  else if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspInit 0 */

  /* USER CODE END TIM8_MspInit 0 */
    /* TIM8 clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM8 GPIO Configuration
    PC7     ------> TIM8_CH2
    */
    GPIO_InitStruct.Pin = Single_Int5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(Single_Int5_GPIO_Port, &GPIO_InitStruct);

    /* TIM8 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspInit 0 */

  /* USER CODE END TIM15_MspInit 0 */
    /* TIM15 clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM15 GPIO Configuration
    PE6     ------> TIM15_CH2
    */
    GPIO_InitStruct.Pin = Single_Int2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;
    HAL_GPIO_Init(Single_Int2_GPIO_Port, &GPIO_InitStruct);

    /* TIM15 interrupt Init */
    HAL_NVIC_SetPriority(TIM15_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM15_IRQn);
  /* USER CODE BEGIN TIM15_MspInit 1 */

  /* USER CODE END TIM15_MspInit 1 */
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PE14     ------> TIM1_CH4
    */
    HAL_GPIO_DeInit(Sync_Int_GPIO_Port, Sync_Int_Pin);

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PB11     ------> TIM2_CH4
    */
    HAL_GPIO_DeInit(Single_Int4_GPIO_Port, Single_Int4_Pin);

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
    PC6     ------> TIM3_CH1
    */
    HAL_GPIO_DeInit(Single_Int3_GPIO_Port, Single_Int3_Pin);

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
    HAL_GPIO_DeInit(Single_Int1_GPIO_Port, Single_Int1_Pin);

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
  else if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();

    /**TIM8 GPIO Configuration
    PC7     ------> TIM8_CH2
    */
    HAL_GPIO_DeInit(Single_Int5_GPIO_Port, Single_Int5_Pin);

    /* TIM8 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspDeInit 0 */

  /* USER CODE END TIM15_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM15_CLK_DISABLE();

    /**TIM15 GPIO Configuration
    PE6     ------> TIM15_CH2
    */
    HAL_GPIO_DeInit(Single_Int2_GPIO_Port, Single_Int2_Pin);

    /* TIM15 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM15_IRQn);
  /* USER CODE BEGIN TIM15_MspDeInit 1 */

  /* USER CODE END TIM15_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
      switch (Sync_Cnt)
      {
      case 1:
        Sync_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4); // 获取当前的捕获值.
        TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_ICPOLARITY_FALLING); // 设置为下降沿捕获
        Sync_Cnt++;
        break;
      case 2:
        Sync_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4); // 获取当前的捕获值.
        HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_4);                              // 停止捕获
        Sync_Cnt++;
        break;
      }
    }
  }
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
      if (Int1_duration == 0x05 && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
      {
        if (IsMpc1)
        {
          xEventGroupSetBitsFromISR(PositionEvent1_MPC1Handle, 1 << ((Mpc1_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }
        else
        {
          xEventGroupSetBitsFromISR(PositionEvent1_MPC2Handle, 1 << ((Mpc2_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }

        // xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
  else if (htim->Instance == TIM15)
  {
    BaseType_t xHigherPriorityTaskWoken;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      if (Int2_first_flag == 0)
      {
        Int2_starttime = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);
        Int2_first_flag = 1;
      }
      else
      {
        Int2_endtime = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);
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
      if (Int2_duration == 0x05 && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
      {
        if (IsMpc1)
        {
          xEventGroupSetBitsFromISR(PositionEvent2_MPC1Handle, 1 << ((Mpc1_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }
        else
        {
          xEventGroupSetBitsFromISR(PositionEvent2_MPC2Handle, 1 << ((Mpc2_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }
        // xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
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
      if (Int3_duration == 0x05 && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
      {
        if (IsMpc1)
        {
          xEventGroupSetBitsFromISR(PositionEvent3_MPC1Handle, 1 << ((Mpc1_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }
        else
        {
          xEventGroupSetBitsFromISR(PositionEvent3_MPC2Handle, 1 << ((Mpc2_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }
        // xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
  else if (htim->Instance == TIM2)
  {
    BaseType_t xHigherPriorityTaskWoken;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
      if (Int4_first_flag == 0)
      {
        Int4_starttime = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
        Int4_first_flag = 1;
      }
      else
      {
        Int4_endtime = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
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
      if (Int4_duration == 0x05 && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
      {
        if (IsMpc1)
        {
          xEventGroupSetBitsFromISR(PositionEvent4_MPC1Handle, 1 << ((Mpc1_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }
        else
        {
          xEventGroupSetBitsFromISR(PositionEvent4_MPC2Handle, 1 << ((Mpc2_Channel_Num + 15) % 16), &xHigherPriorityTaskWoken);
        }
        // xSemaphoreGiveFromISR(HolesCountingSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BaseType_t xHigherPriorityTaskWoken;
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  if (htim->Instance == TIM6)
  {
    // 开启SYNC捕获
    if (IsMpc1)
    {
      Mpc1_Channel_Num = (Mpc1_Channel_Num + 1) % 16;
      R1_R3_Channel_Sel(Mpc1_Channel_Num);
      R2_R4_Channel_Sel(Mpc1_Channel_Num);
      if (Mpc1_Channel_Num == 0)
      {
        EN_R1_R2_close(); // 关闭接收管1
        EN_R3_R4_open();  // 打开接收管2
        IsMpc1 = 0;
      }
    }
    else
    {
      Mpc2_Channel_Num = (Mpc2_Channel_Num + 1) % 16;
      R1_R3_Channel_Sel(Mpc2_Channel_Num);
      R2_R4_Channel_Sel(Mpc2_Channel_Num);

      if (Mpc1_Channel_Num == 0 && Mpc2_Channel_Num == 0)
      {
        Sync_Cnt = 0;
        EN_R3_R4_close();
        HAL_TIM_Base_Stop_IT(&htim6); // Stop timer for switch

        HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
        HAL_TIM_IC_Stop_IT(&htim15, TIM_CHANNEL_2);
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);
        IsMpc1 = 1;
        vTaskNotifyGiveFromISR(Hole_ldentificaHandle, &xHigherPriorityTaskWoken);
      }
    }
  }
  // LTODO: 此处设置溢出中断周期为500us，根据需求更改,假设2个循环
  if (htim->Instance == TIM7)
  {
#if 0
    circulation++;
    circulation %= 2;
    if (circulation == 0)
    {
      // 2个循环完成，需要进行相关任务
      // LTODO: 当前只对标志组1进行置位，后面加上其他标志位的
      position_new = (osEventFlagsGet(PositionEvent1Handle) & 0xFFFF);
      position_xor = position_old ^ position_new;
      xEventGroupClearBitsFromISR(PositionEvent1Handle, 0xFFFF);    //循环结束，清楚所有标志位，以便新的一轮置位
    }
    else
    {
      // 1个循环完成，需要进行相关任务
      position_old =  (osEventFlagsGet(PositionEvent1Handle) & 0xFFFF);
      xEventGroupClearBitsFromISR(PositionEvent1Handle, 0xFFFF);     //循环结束，清楚所有标志位，以便新的一轮置位
    }

#endif

#if 0
    if (Isfirstcirculation)
    {
      position_old = (osEventFlagsGet(PositionEvent1Handle) & 0xFFFF); // 上电第一次循环
      Isfirstcirculation = 0;
      xEventGroupClearBitsFromISR(PositionEvent1Handle, 0xFFFF); // 必要的清0
    }
    else
    {
      position_new = (osEventFlagsGet(PositionEvent1Handle) & 0xFFFF); // 除第一次外的每次循环
      position_xor = position_old ^ position_new;
      position_new_plus_old = position_new - position_old;
      if ((position_xor != 0) && (position_new_plus_old < 65536) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) // 有差异且位置检测从无到有算孔，从有到无不考虑
      {
        vTaskNotifyGiveFromISR(AlarmTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
      position_old = position_new;
      xEventGroupClearBitsFromISR(PositionEvent1Handle, 0xFFFF); // 循环结束，清除所有标志位，以便新的一轮置位
    }

#endif
  }
}
/* USER CODE END 1 */
