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
#define MCU_CX2IN_Pin GPIO_PIN_2
#define MCU_CX2IN_GPIO_Port GPIOA
#define MCU_CX1IN_Pin GPIO_PIN_3
#define MCU_CX1IN_GPIO_Port GPIOA
#define MCU_CX2P_Pin GPIO_PIN_4
#define MCU_CX2P_GPIO_Port GPIOA
#define MCU_CX1P_Pin GPIO_PIN_5
#define MCU_CX1P_GPIO_Port GPIOA
#define MCU_CXCON_Pin GPIO_PIN_6
#define MCU_CXCON_GPIO_Port GPIOA
#define MCU_CX1EN_Pin GPIO_PIN_7
#define MCU_CX1EN_GPIO_Port GPIOA
#define MCU_CX2EN_Pin GPIO_PIN_0
#define MCU_CX2EN_GPIO_Port GPIOB
#define MCU_A2_Pin GPIO_PIN_1
#define MCU_A2_GPIO_Port GPIOB
#define MCU_A3_Pin GPIO_PIN_10
#define MCU_A3_GPIO_Port GPIOB
#define MCU_A4_Pin GPIO_PIN_11
#define MCU_A4_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define MCU_BS0IN_Pin GPIO_PIN_13
#define MCU_BS0IN_GPIO_Port GPIOB
#define MCU_BS1IN_Pin GPIO_PIN_14
#define MCU_BS1IN_GPIO_Port GPIOB
#define MCU_BS_OUT4_Pin GPIO_PIN_15
#define MCU_BS_OUT4_GPIO_Port GPIOB
#define MCU_BS_OUT3_Pin GPIO_PIN_8
#define MCU_BS_OUT3_GPIO_Port GPIOA
#define MCU_F1_Pin GPIO_PIN_3
#define MCU_F1_GPIO_Port GPIOB
#define MCU_F4_Pin GPIO_PIN_4
#define MCU_F4_GPIO_Port GPIOB
#define MCU_F3_Pin GPIO_PIN_5
#define MCU_F3_GPIO_Port GPIOB
#define MCU_F2_Pin GPIO_PIN_6
#define MCU_F2_GPIO_Port GPIOB
#define MCU_A1_Pin GPIO_PIN_7
#define MCU_A1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void BS_SIGNAL_IC(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
