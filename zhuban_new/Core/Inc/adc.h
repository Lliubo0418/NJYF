/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */
#define ADC1_CHANNEL_COUNT 8
#define ADC3_CHANNEL_COUNT 4
#define SAMPLES_PER_CHANNEL 5

#define ADC1_BUFFER_SIZE  (ADC1_CHANNEL_COUNT * SAMPLES_PER_CHANNEL)
#define ADC3_BUFFER_SIZE  (ADC3_CHANNEL_COUNT * SAMPLES_PER_CHANNEL)
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */


//typedef struct {
//    uint32_t *buffer;
//    uint32_t channelCount;
//} ADCMessage;

void CalculateAverage(uint32_t *buffer, float *average, uint32_t channelCount, uint32_t sampleCount);

void Adc_conv_start(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

