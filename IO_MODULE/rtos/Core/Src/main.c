/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "BS.h"
#include "CXxEN.h"
#include "CXxIN.h"
#include "input.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern CAN_TxHeaderTypeDef   TxHeader;
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               TxData[8];
extern uint8_t               RxData[8];
extern uint32_t              TxMailbox;

extern uint32_t capture_Buf[3] ;   //存放计数值
extern uint8_t capture_Cnt ;    //状态标志位
extern uint32_t high_time;   //高电平时间


uint8_t overflow_cnt=0;   //是否溢出


#define CXCON_ON 1           //电子开关闭合
#define CXCON_OFF 0          //电子开关断开
#define CX1EN_ON  0          //查询脉冲使能
#define CX1EN_OFF  1         //查询脉冲失能
#define CX2EN_ON  0          //查询脉冲使能
#define CX2EN_OFF  1         //查询脉冲失能
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	
	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	Set_MCU_CXCON_ON_OFF(CXCON_OFF);         //断开电子开关
	Set_MCU_CX1EN_ON_OFF(CX1EN_OFF);				 //cx1发送脉冲失能
	Set_MCU_CX2EN_ON_OFF(CX2EN_OFF);					//cx2发送脉冲失能
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);              //电子开关(高电平导通，低电平断开）
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);        //MCU_OUT1
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6); 
		              
		HAL_GPIO_TogglePin(GPIOB,LED1_Pin);                    
		HAL_Delay(500);
		
//	TxData[0] = 0x11;    //CAN收发正常
//	TxData[1] = 0xAD;
//	TxData[2] = 0x33;
//	TxData[3] = 0x55;
//	TxData[4] = 0x77;
//	TxData[5] = 0x44;
//	TxData[6] = 0x88;
//	TxData[7] = 0x99;
//	
//	TxHeader.StdId = 0x321;
//  TxHeader.ExtId = 0x00;
//  TxHeader.RTR = CAN_RTR_DATA;
//  TxHeader.IDE = CAN_ID_STD;
//  TxHeader.DLC = 8;
//        
//        /* Start the Transmission process */
//	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//        {
//          /* Transmission request Error */
//          Error_Handler();
//        }
//        HAL_Delay(10);
//			if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0,&RxHeader, RxData)!=HAL_OK){
//					Error_Handler();														 
//				}
//				
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);           //cx2p使能
//	HAL_Delay(1000);

	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);



    /* USER CODE END WHILE */
 
    /* USER CODE BEGIN 3 */
	
		TERM_SIGNAL_IC();            //终端查询信号捕获
		CPU_SIGNAL_IC();
		MCU_F1_freq_get();
		MCU_F2_freq_get();
		MCU_F3_freq_get();
		MCU_F4_freq_get();



       
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



struct __FILE
{
	int handle;
};

FILE __stdout;
void _sys_exit(int x){
x=x;
}
int fputc(int ch,FILE *f){
		HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 

//}; 

//FILE __stdout;       
////定义_sys_exit()以避免使用半主机模式    
//void _sys_exit(int x) 
//{ 
//	x = x; 
//} 
////重定义fputc函数 
//int fputc(int ch, FILE *f)
//{      
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//    USART1->DR = (uint8_t) ch;      
//	return ch;
//}
//#endif 
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
