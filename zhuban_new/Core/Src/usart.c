/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"					//FreeRTOS使用
#include "task.h"
#include "queue.h"

extern QueueHandle_t AudioQueueHandle;

uint16_t USART_RX_STA=0;       //接收状态标记，对14和15位做判断和置为1

uint8_t aRxBuffer[RXBUFFERSIZE];
uint8_t USART_RX_BUF[USART_REC_LEN];

/* USER CODE END 0 */

UART_HandleTypeDef huart6;

/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
	HAL_UART_Receive_IT(&huart6,(uint8_t *)aRxBuffer,RXBUFFERSIZE);
  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG9     ------> USART6_RX
    PG14     ------> USART6_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG9     ------> USART6_RX
    PG14     ------> USART6_TX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9|GPIO_PIN_14);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};
FILE __stdout;
void _sys_exit(int x)
{
    x = x;
}
int fputc(int ch, FILE *f)
{      
    HAL_UART_Transmit(&huart6,(uint8_t*)&ch,1,0xffff);
    return ch;
}

void Uart_XFS_broadcast(uint8_t *Data_to_play){                               //合成播放
	uint8_t Data_XFS[60];
	uint16_t len=0;
	len=strlen(Data_to_play);
	
	Data_XFS[0] = 0xFD ; 			
  Data_XFS[1] = 0x00 ; 			//高字节
  Data_XFS[2] = len+2; 		//低字节
  Data_XFS[3] = 0x01 ; 				 		 
  Data_XFS[4] = 0x00;       //文本编码格式：GB2312
  memcpy(&Data_XFS[5], Data_to_play, len);
	HAL_UART_Transmit(&huart6,(uint8_t*)&Data_XFS,5+len,0xffff);
		
}
//void XFS_Ready(void){	
//		while(aRxBuffer[0]!=0x4F);
//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance==USART6){
        if((USART_RX_STA&0x8000)==0){            //接收未完成
            if(USART_RX_STA&0x4000){               //接收到0x0d（换行符）
                if(aRxBuffer[0]!=0x0a)
									{
										USART_RX_STA=0;        //接受错误，重新开始
									}
										else USART_RX_STA|=0x8000;                        //接收完成了
            }
            else{                                                                        //没接收到0x0d（换行符）
                if(aRxBuffer[0]!=0x0a)
									{
									USART_RX_STA|=0x4000;            
										}
									else{
                    USART_RX_BUF[USART_RX_STA&0x3FFF]=aRxBuffer[0];
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误，重新开始接收
                }
            }
        }
				HAL_UART_Receive_IT(&huart6,(uint8_t *)aRxBuffer,RXBUFFERSIZE);
	
#if 0				
				if(aRxBuffer[0]==0x4F){
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){
    xQueueSendFromISR(AudioQueueHandle, &aRxBuffer[0], &xHigherPriorityTaskWoken);
	
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}	
}

				
#endif				
				
#if 0				


    /* 灏嗕腑鏂俊鍙峰彂閫佸埌闃熷垪 */
	
//    xQueueOverwrite(AudioQueueHandle, aRxBuffer);
	
//		xQueueSend(AudioQueueHandle, &aRxBuffer[0],200);
	
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;	


	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED){
    xQueueSendFromISR(AudioQueueHandle, &aRxBuffer[0], &xHigherPriorityTaskWoken);
	
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}	
#endif
    } 
}

/* USER CODE END 1 */
