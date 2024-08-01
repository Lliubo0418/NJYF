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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PG6 PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

//功能已实现，待调�??

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// 	if(GPIO_Pin==GPIO_PIN_6){
  //   static uint32_t start_time_freq1=0;
  //   static int state_freq1=0;
	// 	static uint32_t Freq1=0;
  //     if(state_freq1==0){
  //       start_time_freq1=__HAL_TIM_GET_COUNTER(&htim2);                //get start time
  //       state_freq1=1;                                                 
  //     }
  //     else{
  //       uint32_t end_time_freq1=__HAL_TIM_GET_COUNTER(&htim2);          //get end time
  //       if(end_time_freq1>start_time_freq1){
  //         uint32_t duration=end_time_freq1-start_time_freq1;                    //周期
  //       }
  //       else{
  //         uint32_t duration=99999-start_time_freq1+end_time_freq1;          //周期
	// 				Freq1=1000000/duration;
  //       }
        
        

  //       state_freq1=0;
  //     }  
	// 	}
    
  // if(GPIO_Pin==GPIO_PIN_7){
  //   static uint32_t start_time_freq2=0;
  //   static int state_freq2=0;
	// 	static uint32_t Freq2=0;
  //     if(state_freq2==0){
  //       start_time_freq2=__HAL_TIM_GET_COUNTER(&htim2);                //get start time
  //       state_freq2=1;                                                 
  //     }
  //     else{
  //       uint32_t end_time_freq2=__HAL_TIM_GET_COUNTER(&htim2);          //get end time
  //       if(end_time_freq2>start_time_freq2){
  //         uint32_t duration=end_time_freq2-start_time_freq2;                    //周期
  //       }
  //       else{
  //         uint32_t duration=99999-start_time_freq2+end_time_freq2;          //周期
	// 				Freq2=1000000/duration;
  //       }
        
  //       state_freq2=0;
  //     }     

  // }

//}
/* USER CODE END 2 */
