/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef Txheader;
CAN_RxHeaderTypeDef Rxheader;
//uint8_t Txmessage[8]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};
uint8_t Rxmessage[8];

CAN_FilterTypeDef hcanfilter;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	hcanfilter.FilterActivation=ENABLE;     //使能
	hcanfilter.FilterBank=0;         //过滤器0
	hcanfilter.FilterFIFOAssignment=CAN_RX_FIFO0;    //接收到的报文放入FIFO0
	hcanfilter.FilterIdHigh=0x000;    //32位高16位屏蔽
	hcanfilter.FilterIdLow=0x000;			 //32位低16位屏蔽
	hcanfilter.FilterMaskIdHigh=0x000;   //32位高16位掩码屏蔽
	hcanfilter.FilterMaskIdLow=0x000;			//32位低16位掩码屏蔽
	hcanfilter.FilterMode=CAN_FILTERMODE_IDMASK;          //过滤器掩码模式
	hcanfilter.FilterScale=CAN_FILTERSCALE_32BIT;         //过滤器宽度
	hcanfilter.SlaveStartFilterBank=0;     //过滤器组设置
	if(HAL_CAN_ConfigFilter(&hcan,&hcanfilter)!=HAL_OK){
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan)!=HAL_OK){
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK){
		Error_Handler();
	}
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback  ( CAN_HandleTypeDef *  hcan ) {
	if(hcan->Instance==CAN1){
		HAL_CAN_GetRxMessage(hcan,CAN_FilterFIFO0,&Rxheader,Rxmessage);
	}
}

uint8_t CAN1_Send_Msg(uint8_t *msg,uint8_t len){
	   uint8_t i=0;
	uint32_t TxMailbox;
	uint8_t message[8];
    Txheader.StdId=0X12;        //标准标识符
		Txheader.ExtId=0x12;        //扩展标识符(29位)
    Txheader.IDE=CAN_ID_STD;    //使用标准帧
    Txheader.RTR=CAN_RTR_DATA;  //数据帧
    Txheader.DLC=len;                
    for(i=0;i<len;i++)
    {
		message[i]=msg[i];
	}
    if(HAL_CAN_AddTxMessage(&hcan, &Txheader, message, &TxMailbox) != HAL_OK)//发送
	{
		return 1;
	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}
    return 0;

}
/* USER CODE END 1 */
