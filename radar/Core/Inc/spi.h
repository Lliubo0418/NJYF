/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

 
 #define SPI1_CS(x)      do{ x ? \
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin , GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin , GPIO_PIN_RESET); \
}while(0)

#define SPI2_CS_SIN(x)   do{ x ? \
    HAL_GPIO_WritePin(SPI2_CS_SIN_GPIO_Port, SPI2_CS_SIN_Pin , GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(SPI2_CS_SIN_GPIO_Port, SPI2_CS_SIN_Pin , GPIO_PIN_RESET); \
}while(0)

#define SPI2_CS_COS(x)   do{ x ? \
    HAL_GPIO_WritePin(SPI2_CS_COS_GPIO_Port, SPI2_CS_COS_Pin , GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(SPI2_CS_COS_GPIO_Port, SPI2_CS_COS_Pin , GPIO_PIN_RESET); \
}while(0)

/******************************************************************************************/

/* SPI总线速度设置 */
#define SPI_SPEED_2 0
#define SPI_SPEED_4 1
#define SPI_SPEED_8 2
#define SPI_SPEED_16 3
#define SPI_SPEED_32 4
#define SPI_SPEED_64 5
#define SPI_SPEED_128 6
#define SPI_SPEED_256 7


/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

