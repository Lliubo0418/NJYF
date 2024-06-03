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

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ACTION1_Pin|RELAY_CTRL_Pin|A1_R_Pin|A0_R_Pin
                          |A3_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, sync_alarm_Pin|output_led_Pin|system_led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_R2_Pin|ACTION_Pin|EN_R1_Pin|A2_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = ACTION1_Pin|RELAY_CTRL_Pin|output_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = sync_alarm_Pin|system_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = EN_R2_Pin|EN_R1_Pin|A2_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = ACTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ACTION_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = A1_R_Pin|A0_R_Pin|A3_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void Channel_Sel(uint8_t ch){
	switch(ch%8){
		case 0:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_SET);
			break;
		case 1:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_RESET);			
			break;
		case 2:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_SET);
			break;
		case 3:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_RESET);			
			break;			
		case 4:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_SET);
			break;
		case 5:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_RESET);			
			break;
		case 6:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_SET);
			break;
		case 7:
		HAL_GPIO_WritePin(A2_R_GPIO_Port,A2_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A1_R_GPIO_Port,A1_R_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(A0_R_GPIO_Port,A0_R_Pin,GPIO_PIN_RESET);			
			break;	
	}
}
/* USER CODE END 2 */
