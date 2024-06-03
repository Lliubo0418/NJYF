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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
/*ͬ���ź�*/
 extern uint32_t Sync_capture_Buf[3];   //��ż���ֵ
 extern uint8_t Sync_Cnt ;    //״̬��־λ
 extern uint32_t Sync_high_time;   //�ߵ�ƽʱ��
 
 /*����ź�*/
 extern uint32_t Signal_capture_Buf[3] ;   //��ż���ֵ
 extern uint8_t Signal_Cnt ;    //״̬��־λ
 extern uint32_t Signal_high_time;   //�ߵ�ƽʱ��
 
 float time;
 float count=0;
 extern uint32_t duration ;

 extern uint16_t up_edge_cnt;
 uint8_t count_flag=0;
 
 uint8_t i=1;
 extern __IO uint32_t uDelayValue;
 
 static uint32_t fac_us=0;
 void EN_R1_open(void);
 void EN_R2_open(void);
void delay_us(uint32_t us);

void EN_R1_R2_close(void);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim2); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_GPIO_TogglePin(EN_R1_GPIO_Port,EN_R1_Pin);
//		delay_us(100);
//	EN_R1_open();
//		delay_us(10);
//		EN_R2_open();
//		delay_us(10);
switch (Sync_Cnt){
	case 0:
		Sync_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);	//�������벶��       ����: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(Sync_capture_Buf[1]<Sync_capture_Buf[0]){                  //�������Ȳ���ߵ�ƽ�󲶻�͵�ƽcapture_Buf[1]<capture_Buf[0],�϶����������
			Sync_high_time = 0xC350+Sync_capture_Buf[1]- Sync_capture_Buf[0];
		}
		else{
		Sync_high_time = Sync_capture_Buf[1]- Sync_capture_Buf[0];    //�ߵ�ƽʱ��  

		}
			
		Sync_Cnt = 0;
	if(Sync_high_time==0x1B){
		Sync_Cnt = 5;
		//count_flag=1;
		EN_R1_open();
			delay_us(1);                  //��ʱһ΢�������л�
			for(i=1;i<=16;i++){
	
			Channel_Sel(i);
			delay_us(8);
		if(i==8){
			EN_R2_open();
		}
		if(i==16){
			EN_R1_R2_close();
			Sync_Cnt=0;
			}
		}
//		up_edge_cnt=0;
//		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
//		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	}
		break;
	}
//	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_11);
//	delay_us(10);

	
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
void EN_R1_open(void){
		HAL_GPIO_WritePin(EN_R1_GPIO_Port,EN_R1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(EN_R2_GPIO_Port,EN_R2_Pin,GPIO_PIN_RESET);
}
void EN_R2_open(void){
		HAL_GPIO_WritePin(EN_R1_GPIO_Port,EN_R1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_R2_GPIO_Port,EN_R2_Pin,GPIO_PIN_SET);
}
void EN_R1_R2_close(void){
		HAL_GPIO_WritePin(EN_R1_GPIO_Port,EN_R1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_R2_GPIO_Port,EN_R2_Pin,GPIO_PIN_RESET);
}

//void HAL_Delay_us(uint32_t us) {
//    uint32_t start = SysTick->VAL;
//    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

//    while ((SysTick->VAL - start) < delayTicks) {
//        // Wait until the difference between the current value and the start value is equal to delayTicks
//    }
//}
//void delay_us(uint32_t nus)
//{		
//	uint32_t ticks;
//	uint32_t told,tnow,tcnt=0;
//	uint32_t reload=SysTick->LOAD;				//LOAD��ֵ	    	 
//	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
//	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
//	while(1)
//	{
//		tnow=SysTick->VAL;	
//		if(tnow!=told)
//		{	    
//			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
//			else tcnt+=reload-tnow+told;	    
//			told=tnow;
//			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
//		}  
//	}
//}
void delay_us(uint32_t us)
{
//    HAL_TIM_Base_Start(&htim2); // ������ʱ��
//    __HAL_TIM_SET_COUNTER(&htim2, 0); // ������������
//    while (__HAL_TIM_GET_COUNTER(&htim2) < us); // �ȴ��������ﵽ�����΢����
//    HAL_TIM_Base_Stop(&htim2); // ֹͣ��ʱ��

//	    HAL_TIM_Base_Start(&htim2); // ������ʱ��
//    __HAL_TIM_SET_COUNTER(&htim2, 0); // ������������
//    while (1) {
//        if (__HAL_TIM_GET_COUNTER(&htim2) >= us) {
//            break; // ���������ﵽ�����΢����ʱ�˳�ѭ��
//        }
//    }
//    HAL_TIM_Base_Stop(&htim2); // ֹͣ��ʱ��
    __HAL_TIM_SET_COUNTER(&htim2, 0); // ��������ֵ����Ϊ0
    while (__HAL_TIM_GET_COUNTER(&htim2) < us); // �ȴ��������ﵽָ����΢����


}

//void delay_us(uint32_t us)
//{
//    uint32_t usTickStart = uwTick;
//    uint32_t ticksToWait = us * (SystemCoreClock / 1000000UL);

//    while ((uwTick - usTickStart) < ticksToWait)
//    {
//        /* �ȴ������ﵽ��ʱʱ�� */
//    }
//}
//void delay_us(uint32_t us)
//{
//    uint32_t start = get_time_us();
//    uint32_t expected = start + us;
//    
//    /* �ȴ��ﵽԤ��ʱ�� */
//    while (get_time_us() < expected);
//}
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
