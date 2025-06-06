/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
extern uint16_t semavalue;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_Pin | Output_Led_Pin | Sync_Alarm_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_R2_GPIO_Port, EN_R2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EN_R4_Pin | A0_R3_R4_Pin | A1_R3_R4_Pin | A2_R3_R4_Pin | A3_R3_R4_Pin | ACTION1_Pin | ACTION_Pin | RELAY_CTRL_Pin | EN_R3_Pin | EN_R1_Pin | A0_R1_R2_Pin | A1_R1_R2_Pin | A2_R1_R2_Pin | A3_R1_R2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DE_485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DE_485_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = JCOUT_Pin | JCOUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = EN_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(EN_R2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin
                           PDPin PDPin */
  GPIO_InitStruct.Pin = EN_R4_Pin | A0_R3_R4_Pin | A1_R3_R4_Pin | A2_R3_R4_Pin | A3_R3_R4_Pin | ACTION1_Pin | ACTION_Pin | RELAY_CTRL_Pin | EN_R3_Pin | EN_R1_Pin | A0_R1_R2_Pin | A1_R1_R2_Pin | A2_R1_R2_Pin | A3_R1_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin */
  GPIO_InitStruct.Pin = Output_Led_Pin | Sync_Alarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/* USER CODE BEGIN 2 */
#if 1
void EN_R1_R2_open(void)
{
  HAL_GPIO_WritePin(EN_R1_GPIO_Port, EN_R1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_R2_GPIO_Port, EN_R2_Pin, GPIO_PIN_SET);
}
void EN_R1_R2_close(void)
{
  HAL_GPIO_WritePin(EN_R1_GPIO_Port, EN_R1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_R2_GPIO_Port, EN_R2_Pin, GPIO_PIN_RESET);
}
void EN_R3_R4_open(void)
{
  HAL_GPIO_WritePin(EN_R3_GPIO_Port, EN_R3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_R4_GPIO_Port, EN_R4_Pin, GPIO_PIN_SET);
}
void EN_R3_R4_close(void)
{
  HAL_GPIO_WritePin(EN_R3_GPIO_Port, EN_R3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_R4_GPIO_Port, EN_R4_Pin, GPIO_PIN_RESET);
}

#endif


void R1_R2_Channel_Sel(uint8_t ch)
{ 
  ch = 15 - ((ch + 15) & 0x0F); 

  GPIO_PinState pinA3 = (ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA2 = (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA1 = (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA0 = (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(A3_R1_R2_GPIO_Port, A3_R1_R2_Pin, pinA3);
  HAL_GPIO_WritePin(A2_R1_R2_GPIO_Port, A2_R1_R2_Pin, pinA2);
  HAL_GPIO_WritePin(A1_R1_R2_GPIO_Port, A1_R1_R2_Pin, pinA1);
  HAL_GPIO_WritePin(A0_R1_R2_GPIO_Port, A0_R1_R2_Pin, pinA0);
}
void R3_R4_Channel_Sel(uint8_t ch)
{ 
  ch = 15 - ((ch + 15) & 0x0F);

  GPIO_PinState pinA3 = (ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA2 = (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA1 = (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA0 = (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(A3_R3_R4_GPIO_Port, A3_R3_R4_Pin, pinA3);
  HAL_GPIO_WritePin(A2_R3_R4_GPIO_Port, A2_R3_R4_Pin, pinA2);
  HAL_GPIO_WritePin(A1_R3_R4_GPIO_Port, A1_R3_R4_Pin, pinA1);
  HAL_GPIO_WritePin(A0_R3_R4_GPIO_Port, A0_R3_R4_Pin, pinA0);
}

void Holes_output_alarm_Open(void)
{
  HAL_GPIO_WritePin(ACTION_GPIO_Port, ACTION_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACTION1_GPIO_Port, ACTION1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Output_Led_GPIO_Port, Output_Led_Pin, GPIO_PIN_RESET);
}
void Holes_output_alarm_Close(void)
{
  HAL_GPIO_WritePin(ACTION_GPIO_Port, ACTION_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ACTION1_GPIO_Port, ACTION1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Output_Led_GPIO_Port, Output_Led_Pin, GPIO_PIN_SET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_1)
  {
    Holes_output_alarm_Open();
    HAL_TIM_Base_Start_IT(&htim7);
  }
}
/* USER CODE END 2 */
