/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

extern __IO uint8_t               Uart2RxData[256];

extern __IO uint8_t               Uart1TxDoneFlag;
extern __IO uint8_t               Uart3TxDoneFlag;

extern __IO uint8_t KeyValue, KeyValueKeep;
extern const uint8_t              KeyIndex[49];

extern uint8_t KeyCmd[14];
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USB_Key_Update(uint8_t index);
uint16_t CalculateCRC16(uint8_t *data, uint16_t length);
void KeyModbusBy485_On(uint8_t index);
void KeyModbusBy485_Off(uint8_t index);
uint8_t CalculateChecksum(uint8_t *data, uint8_t length);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

