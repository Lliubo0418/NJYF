/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#define NUM_CHANNELS 10
#define VOLTAGE_LOWER_LIMIT 0.35
#define VOLTAGE_UPPER_LIMIT 0.7

volatile uint16_t adc_value[10];
uint16_t adcx[10];
// float temp[10];
float sp[10];
uint16_t error_flag=0;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    GPIO_InitStruct.Pin = SP9_Pin|SP8_Pin|SP7_Pin|SP6_Pin
                          |SP5_Pin|SP4_Pin|SP3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SP2_Pin|SP1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    HAL_GPIO_DeInit(GPIOA, SP9_Pin|SP8_Pin|SP7_Pin|SP6_Pin
                          |SP5_Pin|SP4_Pin|SP3_Pin);

    HAL_GPIO_DeInit(GPIOB, SP2_Pin|SP1_Pin);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// void Adc_Get(){
//   HAL_ADC_PollForConversion(&hadc1,10);
//	for(int ch=0;ch<8;ch++){
//   adc_value[ch]=HAL_ADC_GetValue(&hadc1);
//   }
//
// }
uint16_t Get_Adc(uint32_t ch)
{
  ADC_ChannelConfTypeDef ADC1_ChanConf;

  ADC1_ChanConf.Channel = ch;                              // 通道
  ADC1_ChanConf.Rank = 1;                                  // 第1个序列，序列1
  ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; // 采样时间
  HAL_ADC_ConfigChannel(&hadc1, &ADC1_ChanConf);           // 通道配置

  HAL_ADC_Start(&hadc1); // 开启ADC

  HAL_ADC_PollForConversion(&hadc1, 10); // 轮询转换

  return (uint16_t)HAL_ADC_GetValue(&hadc1); // 返回最近一次ADC1规则组的转换结果
}
// 获取指定通道的转换值，取times次,然后平均
// times:获取次数
// 返回值:通道ch的times次转换结果平均值
uint16_t Get_Adc_Average(uint32_t ch, uint8_t times)
{
  uint32_t temp_val = 0;
  uint8_t t;
  for (t = 0; t < times; t++)
  {
    temp_val += Get_Adc(ch);
    HAL_Delay(5);
  }
  return temp_val / times;
}

void Read_ADC_Values()
{
  const uint32_t channels[NUM_CHANNELS] = {
       
      ADC_CHANNEL_9,ADC_CHANNEL_8, ADC_CHANNEL_7,ADC_CHANNEL_6,
			ADC_CHANNEL_5,ADC_CHANNEL_4,ADC_CHANNEL_3, ADC_CHANNEL_2, 
      ADC_CHANNEL_1, ADC_CHANNEL_1 // Channels for SEL_GPIO_Port states
  };
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    /*SEL切换*/
    if (i == 8)
    {
      HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_RESET); // NC
    }
    else if (i == 9)
    {
      HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_SET); // NO
    }

    adcx[i] = Get_Adc_Average(channels[i], 20); // Get ADC average
    sp[i] = (float)adcx[i] * (3.3f / 4096);     // Calculate voltage
    if ( sp[i] < VOLTAGE_LOWER_LIMIT)
    {
      error_flag |= (1 << i);
    }
    else
    {
      error_flag &= ~(1 << i);
    }
    HAL_Delay(5); // Delay
  }
  if (error_flag != 0)
  {
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET); // 告警
  }
  else
  {
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET); // 恢复正常
  }
}
/* USER CODE END 1 */
