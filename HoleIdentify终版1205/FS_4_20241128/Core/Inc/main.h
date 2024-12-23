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
#include "stm32f1xx_hal.h"

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
#define SP9_Pin GPIO_PIN_1
#define SP9_GPIO_Port GPIOA
#define SP8_Pin GPIO_PIN_2
#define SP8_GPIO_Port GPIOA
#define SP7_Pin GPIO_PIN_3
#define SP7_GPIO_Port GPIOA
#define SP6_Pin GPIO_PIN_4
#define SP6_GPIO_Port GPIOA
#define SP5_Pin GPIO_PIN_5
#define SP5_GPIO_Port GPIOA
#define SP4_Pin GPIO_PIN_6
#define SP4_GPIO_Port GPIOA
#define SP3_Pin GPIO_PIN_7
#define SP3_GPIO_Port GPIOA
#define SP2_Pin GPIO_PIN_0
#define SP2_GPIO_Port GPIOB
#define SP1_Pin GPIO_PIN_1
#define SP1_GPIO_Port GPIOB
#define SEL_Pin GPIO_PIN_13
#define SEL_GPIO_Port GPIOB
#define A_Pin GPIO_PIN_8
#define A_GPIO_Port GPIOA
#define SYS_LED_Pin GPIO_PIN_11
#define SYS_LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
