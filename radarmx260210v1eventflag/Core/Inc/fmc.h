/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : FMC.h
  * Description        : This file provides code for the configuration
  *                      of the FMC peripheral.
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
#ifndef __FMC_H
#define __FMC_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN Private defines */



#define SRAM_SIZE      (1 << 19)   // 512KB SRAM   256k*16bit
#define HALF_SRAM_SIZE (1 << 18)   // 256KB
#define BUF_WORDS      512         // 512 x uint16_t = 1024 bytes
#define PATTERN_SIZE   2048        // 数据范围 0 到 0x7FF (2048 个值)
#define BUF_BYTES      (BUF_WORDS * 2) // 1024 bytes
#define SAMPLE_COUNT   400         // 采样点数
#define SAMPLE_INTERVAL 50         // 采样间隔
#define SIN_FLAG       0           // 正弦数据标志
#define COS_FLAG       1           // 余弦数据标志


#define SRAM_BASE_ADDR         0X60000000 
//SRAM位置
// #define DS_DATA_ADDR_END       0x13880        //0-0x1387E 40000个数据
#define DS_DATA_ADDR_END       0x9C40            //20000个数据


// #define WS_DATA_ADDR_END       0x27100       //0-0x270FE  40000个数据
#define WS_DATA_ADDR_END       0x13880       //20000个数据

// 错误码定义
typedef enum {
    SRAM_READ_OK = 0,
    SRAM_ADDR_OUT_OF_BOUNDS = -1,
    SRAM_RESULT_FULL = -2
} SramReadStatus;

extern uint8_t fpga_read_flag;


/* USER CODE END Private defines */

void MX_FMC_Init(void);
void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram);
void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram);

/* USER CODE BEGIN Prototypes */

void sram_init(void);
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read_u16(uint16_t *pbuf, uint32_t addr, uint32_t datalen);

SramReadStatus read_sram_sincos_data(uint8_t sin_cos_flag, uint16_t start_pos, uint16_t count);




/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__FMC_H */

/**
  * @}
  */

/**
  * @}
  */
