/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define FPGA_CTRL_SIDE_Pin GPIO_PIN_5
#define FPGA_CTRL_SIDE_GPIO_Port GPIOE
#define FPGA_CTRL_READ_Pin GPIO_PIN_6
#define FPGA_CTRL_READ_GPIO_Port GPIOE
#define FPGA_RUN_Pin GPIO_PIN_7
#define FPGA_RUN_GPIO_Port GPIOF
#define FPGA_RAM_Pin GPIO_PIN_8
#define FPGA_RAM_GPIO_Port GPIOF
#define SPI2_CS_SIN_Pin GPIO_PIN_10
#define SPI2_CS_SIN_GPIO_Port GPIOB
#define SPI2_CS_COS_Pin GPIO_PIN_11
#define SPI2_CS_COS_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA
#define SRAM_RD_Pin GPIO_PIN_4
#define SRAM_RD_GPIO_Port GPIOD
#define SRAM_WR_Pin GPIO_PIN_5
#define SRAM_WR_GPIO_Port GPIOD
#define SRAM_CS_Pin GPIO_PIN_7
#define SRAM_CS_GPIO_Port GPIOD
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FPGA_SIDE_FLAG                  HAL_GPIO_ReadPin(FPGA_CTRL_SIDE_GPIO_Port, FPGA_CTRL_SIDE_Pin)     /* ¶ÁÈ¡FPGA_CTRL_SIDE_PINÒý½Å */

#define FPGA_RUN_FLAG(x)      do{ (x) ? \
    HAL_GPIO_WritePin(FPGA_RUN_GPIO_Port, FPGA_RUN_Pin, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(FPGA_RUN_GPIO_Port, FPGA_RUN_Pin, GPIO_PIN_RESET); \
}while(0)

#define FPGA_RAM_FLAG(x)      do{ (x) ? \
    HAL_GPIO_WritePin(FPGA_RAM_GPIO_Port, FPGA_RAM_Pin, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(FPGA_RAM_GPIO_Port, FPGA_RAM_Pin, GPIO_PIN_RESET); \
}while(0)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
