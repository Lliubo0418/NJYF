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
#include "stm32h7xx_hal.h"

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
#define Single_Int2_Pin GPIO_PIN_6
#define Single_Int2_GPIO_Port GPIOE
#define DE_485_Pin GPIO_PIN_2
#define DE_485_GPIO_Port GPIOA
#define JCOUT_Pin GPIO_PIN_1
#define JCOUT_GPIO_Port GPIOB
#define JCOUT_EXTI_IRQn EXTI1_IRQn
#define JCOUT1_Pin GPIO_PIN_2
#define JCOUT1_GPIO_Port GPIOB
#define JCOUT1_EXTI_IRQn EXTI2_IRQn
#define Sync_Int_Pin GPIO_PIN_14
#define Sync_Int_GPIO_Port GPIOE
#define Single_Int4_Pin GPIO_PIN_11
#define Single_Int4_GPIO_Port GPIOB
#define EN_R2_Pin GPIO_PIN_15
#define EN_R2_GPIO_Port GPIOB
#define EN_R4_Pin GPIO_PIN_8
#define EN_R4_GPIO_Port GPIOD
#define A0_R2_R4_Pin GPIO_PIN_9
#define A0_R2_R4_GPIO_Port GPIOD
#define A1_R2_R4_Pin GPIO_PIN_10
#define A1_R2_R4_GPIO_Port GPIOD
#define A2_R2_R4_Pin GPIO_PIN_11
#define A2_R2_R4_GPIO_Port GPIOD
#define A3_R2_R4_Pin GPIO_PIN_12
#define A3_R2_R4_GPIO_Port GPIOD
#define ACTION1_Pin GPIO_PIN_13
#define ACTION1_GPIO_Port GPIOD
#define ACTION_Pin GPIO_PIN_14
#define ACTION_GPIO_Port GPIOD
#define RELAY_CTRL_Pin GPIO_PIN_15
#define RELAY_CTRL_GPIO_Port GPIOD
#define Single_Int3_Pin GPIO_PIN_6
#define Single_Int3_GPIO_Port GPIOC
#define Single_Int5_Pin GPIO_PIN_7
#define Single_Int5_GPIO_Port GPIOC
#define EN_R3_Pin GPIO_PIN_0
#define EN_R3_GPIO_Port GPIOD
#define EN_R1_Pin GPIO_PIN_1
#define EN_R1_GPIO_Port GPIOD
#define A0_R1_R3_Pin GPIO_PIN_2
#define A0_R1_R3_GPIO_Port GPIOD
#define A1_R1_R3_Pin GPIO_PIN_3
#define A1_R1_R3_GPIO_Port GPIOD
#define A2_R1_R3_Pin GPIO_PIN_4
#define A2_R1_R3_GPIO_Port GPIOD
#define A3_R1_R3_Pin GPIO_PIN_5
#define A3_R1_R3_GPIO_Port GPIOD
#define Single_Int1_Pin GPIO_PIN_6
#define Single_Int1_GPIO_Port GPIOB
#define Output_Led_Pin GPIO_PIN_0
#define Output_Led_GPIO_Port GPIOE
#define Sync_Alarm_Pin GPIO_PIN_1
#define Sync_Alarm_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
