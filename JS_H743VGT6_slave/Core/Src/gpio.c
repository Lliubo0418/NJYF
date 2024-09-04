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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_485_GPIO_Port, DE_485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, JCOUT_Pin|JCOUT1_Pin|EN_R2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EN_R4_Pin|A0_R2_R4_Pin|A1_R2_R4_Pin|A2_R2_R4_Pin
                          |A3_R2_R4_Pin|EN_R1_Pin|A0_R1_R3_Pin|A1_R1_R3_Pin
                          |A2_R1_R3_Pin|A3_R1_R3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_R3_GPIO_Port, EN_R3_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pin = JCOUT_Pin|JCOUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = EN_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(EN_R2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = EN_R4_Pin|A0_R2_R4_Pin|A1_R2_R4_Pin|A2_R2_R4_Pin
                          |A3_R2_R4_Pin|EN_R3_Pin|EN_R1_Pin|A0_R1_R3_Pin
                          |A1_R1_R3_Pin|A2_R1_R3_Pin|A3_R1_R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
#if 0
void EN_R1_R2_R3_R4_open(void)
{
  HAL_GPIO_WritePin(EN_R1_GPIO_Port, EN_R1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_R2_GPIO_Port, EN_R2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_R3_GPIO_Port, EN_R3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_R4_GPIO_Port, EN_R4_Pin, GPIO_PIN_SET);
}
void EN_R1_R2_R3_R4_close(void)
{
  HAL_GPIO_WritePin(EN_R1_GPIO_Port, EN_R1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_R2_GPIO_Port, EN_R2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_R3_GPIO_Port, EN_R3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_R4_GPIO_Port, EN_R4_Pin, GPIO_PIN_RESET);
}

#endif

void R1_R3_Channel_Sel(uint8_t ch)
{ // MPC1通道切换
  // 取低四位，ch % 16 可以简化为 ch & 0x0F
  ch &= 0x0F;

  // 用于生成 GPIO 引脚的状态，A3、A2、A1、A0 对应 ch 的位 3、2、1、0
  GPIO_PinState pinA3 = (ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA2 = (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA1 = (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA0 = (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  // 一次性设置 GPIO 引脚状态
  HAL_GPIO_WritePin(A3_R1_R3_GPIO_Port, A3_R1_R3_Pin, pinA3);
  HAL_GPIO_WritePin(A2_R1_R3_GPIO_Port, A2_R1_R3_Pin, pinA2);
  HAL_GPIO_WritePin(A1_R1_R3_GPIO_Port, A1_R1_R3_Pin, pinA1);
  HAL_GPIO_WritePin(A0_R1_R3_GPIO_Port, A0_R1_R3_Pin, pinA0);
}
void R2_R4_Channel_Sel(uint8_t ch)
{ // MPC2通道切换
  // 取低四位，ch % 16 可以简化为 ch & 0x0F
  ch &= 0x0F;

  // 用于生成 GPIO 引脚的状态，A3、A2、A1、A0 对应 ch 的位 3、2、1、0
  GPIO_PinState pinA3 = (ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA2 = (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA1 = (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState pinA0 = (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  // 一次性设置 GPIO 引脚状态
  HAL_GPIO_WritePin(A3_R2_R4_GPIO_Port, A3_R2_R4_Pin, pinA3);
  HAL_GPIO_WritePin(A2_R2_R4_GPIO_Port, A2_R2_R4_Pin, pinA2);
  HAL_GPIO_WritePin(A1_R2_R4_GPIO_Port, A1_R2_R4_Pin, pinA1);
  HAL_GPIO_WritePin(A0_R2_R4_GPIO_Port, A0_R2_R4_Pin, pinA0);
}

void Holes_output_alarm_Reported(void)
{
  HAL_GPIO_TogglePin(JCOUT_GPIO_Port, JCOUT_Pin);
  HAL_Delay(5);
  HAL_GPIO_TogglePin(JCOUT_GPIO_Port, JCOUT_Pin);
  HAL_Delay(5);
}

/* USER CODE END 2 */
