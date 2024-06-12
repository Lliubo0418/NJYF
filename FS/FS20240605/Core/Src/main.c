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
#include "adc.h"
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
	uint16_t adcx[8];
	float temp[8];
	float sp[8];

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
extern volatile uint16_t adc_value[8];
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_Delay_us_init(72);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim4);
	
//	HAL_TIMEx_PWMN_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		for(int i=0;i<8;i++){
//      switch (i)
//      {
//      case 0:
//        ch=ADC_CHANNEL_2;
//        break;
//      case 1:
//        ch=ADC_CHANNEL_3;
//        break;
//      case 2:
//        ch=ADC_CHANNEL_4;
//        break;
//      case 3:
//        ch=ADC_CHANNEL_5;
//        break;
//      case 4:
//        ch=ADC_CHANNEL_6;
//        break;
//      case 5:
//        ch=ADC_CHANNEL_7;
//        break;
//      case 6:
//        ch=ADC_CHANNEL_8;
//        break;
//      case 7:
//        ch=ADC_CHANNEL_9;
//        break;
//      default:
//        break;
//      }
//      adcx[i]=Get_Adc_Average(ch,20);
//      temp[i]=(float)adcx[i]*(3.3/4096); 
//    }
		
    adcx[0]=Get_Adc_Average(ADC_CHANNEL_2,20);//获取通道1的转换值，20次取平均
		temp[0]=(float)adcx[0]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);	
		
    adcx[1]=Get_Adc_Average(ADC_CHANNEL_3,20);//获取通道1的转换值，20次取平均
		temp[1]=(float)adcx[1]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);

    adcx[2]=Get_Adc_Average(ADC_CHANNEL_4,20);//获取通道1的转换值，20次取平均
		temp[2]=(float)adcx[2]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);	
		
    adcx[3]=Get_Adc_Average(ADC_CHANNEL_5,20);//获取通道1的转换值，20次取平均
		temp[3]=(float)adcx[3]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);

    adcx[4]=Get_Adc_Average(ADC_CHANNEL_6,20);//获取通道1的转换值，20次取平均
		temp[4]=(float)adcx[4]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);	
		
    adcx[5]=Get_Adc_Average(ADC_CHANNEL_7,20);//获取通道1的转换值，20次取平均
		temp[5]=(float)adcx[5]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);

    adcx[6]=Get_Adc_Average(ADC_CHANNEL_8,20);//获取通道1的转换值，20次取平均
		temp[6]=(float)adcx[6]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);	
		
    adcx[7]=Get_Adc_Average(ADC_CHANNEL_9,20);//获取通道1的转换值，20次取平均
		temp[7]=(float)adcx[7]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		HAL_Delay(50);
	
////    adcx[7]=Get_Adc_Average(ADC_CHANNEL_9,20);//获取通道1的转换值，20次取平均
////		temp[7]=(float)adcx[7]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
////		HAL_Delay(50);
////    adcx[6]=Get_Adc_Average(ADC_CHANNEL_8,20);//获取通道1的转换值，20次取平均
////		temp[6]=(float)adcx[6]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
////		HAL_Delay(50);	
		
////////		for(int i=0;i<8;i++)
////////      {
//////////          HAL_ADC_Start(&hadc1);
////////          HAL_ADC_PollForConversion(&hadc1,10);
////////          adc_value[i]=HAL_ADC_GetValue(&hadc1);
////////				sp[i] = (float)adc_value[i]*3.3/4096;
//////////				sp[i+1] = (float)adc_value[i]*3.3/4096;               ？？？why
////////      }
//      HAL_ADC_Stop(&hadc1);

	  /*SEL切换*/
//		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
