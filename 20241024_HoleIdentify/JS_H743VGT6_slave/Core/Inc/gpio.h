/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#if 1
void EN_R1_R2_open(void);
void EN_R1_R2_close(void);
void EN_R3_R4_open(void);
void EN_R3_R4_close(void);
#endif

void Holes_output_alarm_Reported(void);
void R1_R2_Channel_Sel(uint8_t ch);	
void R3_R4_Channel_Sel(uint8_t ch);
#if 0
void EN_R1_R2_R3_R4_open(void);
void EN_R1_R2_R3_R4_close(void);
#endif
void Holes_output_alarm_Reported(void );

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

