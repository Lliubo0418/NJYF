/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOE
#define DE_485_Pin GPIO_PIN_1
#define DE_485_GPIO_Port GPIOA
#define JCOUT_Pin GPIO_PIN_0
#define JCOUT_GPIO_Port GPIOB
#define JCOUT1_Pin GPIO_PIN_1
#define JCOUT1_GPIO_Port GPIOB
#define EN_R2_Pin GPIO_PIN_8
#define EN_R2_GPIO_Port GPIOD
#define A0_R2_Pin GPIO_PIN_9
#define A0_R2_GPIO_Port GPIOD
#define A1_R2_Pin GPIO_PIN_10
#define A1_R2_GPIO_Port GPIOD
#define A2_R2_Pin GPIO_PIN_11
#define A2_R2_GPIO_Port GPIOD
#define A3_R2_Pin GPIO_PIN_12
#define A3_R2_GPIO_Port GPIOD
#define EN_R1_Pin GPIO_PIN_0
#define EN_R1_GPIO_Port GPIOD
#define A0_R1_Pin GPIO_PIN_1
#define A0_R1_GPIO_Port GPIOD
#define A1_R1_Pin GPIO_PIN_2
#define A1_R1_GPIO_Port GPIOD
#define A2_R1_Pin GPIO_PIN_3
#define A2_R1_GPIO_Port GPIOD
#define A3_R1_Pin GPIO_PIN_4
#define A3_R1_GPIO_Port GPIOD
#define Sync_Alarm_Pin GPIO_PIN_1
#define Sync_Alarm_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */