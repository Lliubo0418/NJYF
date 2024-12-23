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
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "limits.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint32_t Sync_capture_Buf[2]; // 存放计数值
extern uint8_t Sync_Cnt;             // 状态标志位
extern uint32_t Sync_high_time;      // 高电平时间

uint16_t semavalue[4] = {0};

uint8_t IsfirstPos_conf = 1;
uint8_t circule_times = 4;

int32_t position_old[4] = {0};
int32_t position_and[4] = {0};
int32_t position_new[4] = {0};
int32_t position_new_plus_old[4] = {0};
uint8_t position_num = 0;

uint16_t Mpc1_Position1 = 0;
uint16_t Mpc2_Position1 = 0;
uint16_t Mpc1_Position2 = 0;
uint16_t Mpc2_Position2 = 0;
uint16_t Mpc1_Position3 = 0;
uint16_t Mpc2_Position3 = 0;
uint16_t Mpc1_Position4 = 0;
uint16_t Mpc2_Position4 = 0;

uint16_t *Mpc_Position[] = {
    &Mpc1_Position2, &Mpc2_Position2,
    &Mpc1_Position1, &Mpc2_Position1,      //TIM4   
    &Mpc1_Position4, &Mpc2_Position4,      //TIM2
    &Mpc1_Position3, &Mpc2_Position3};     //TIM3

extern uint8_t pulse_end_flag;

extern bool IsSteel;

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
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void UpdateAndCheckPositions(void);

void InitializePositions(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  // MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch (Sync_Cnt)
    {
    case 0:
      Sync_Cnt++;
      TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4);
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
      HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4); 
      break;

    case 3:
      
      if (Sync_capture_Buf[1] < Sync_capture_Buf[0])
      {
        Sync_high_time = 50000 + Sync_capture_Buf[1] - Sync_capture_Buf[0];
      }
      else
      {
        Sync_high_time = Sync_capture_Buf[1] - Sync_capture_Buf[0];
      }

      
      if (Sync_high_time >= 0x19 && Sync_high_time <= 0x30)
      {
        memset(&semavalue, 0, sizeof(semavalue));
        IsSteel = 1;
        for (uint8_t i = 0; i < sizeof(Mpc_Position) / sizeof(Mpc_Position[0]); i++)
        {
          *Mpc_Position[i] = 0;
        }
        test4++;
        Sync_Cnt = 5;

        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        HAL_NVIC_EnableIRQ(TIM15_IRQn);

        HAL_TIM_Base_Start_IT(&htim6); // 启动 R1_channel 切换的定时器

        HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
        HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
      }
      else
      {
        Sync_Cnt = 0;
      }
      break;

    case 5:

      if (pulse_end_flag == 1)
      {
        if (IsSteel != 0)
        {
					if(IsfirstPos_conf==1){
					memset(&position_old,0,sizeof(position_old));
					HAL_Delay(2);
						if (circule_times != 0)
          {
            InitializePositions();
            if (circule_times == 1)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                  if (i > 0 && position_old[i - 1] == 0)
                  {
                    position_old[i] = ~position_old[i];                
                    position_old[i] &= 0xFFFFFFFC;        
                  }
                  else
                  {
                    position_old[i] = ~position_old[i]; 
                  }
                }
							IsfirstPos_conf =0;
            }
            circule_times -= 1;
          }
				}
          else
          {
            __disable_irq(); 
            UpdateAndCheckPositions();
            __enable_irq(); 

          }
        }
        pulse_end_flag = 0;
        Sync_Cnt = 0;
      }
      break;
    default:
      break;
    }
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

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void InitializePositions(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    uint16_t pos_low = *Mpc_Position[i * 2 + 1];   
    uint16_t pos_high = *Mpc_Position[i * 2];      
    position_old[i] |= (pos_low | (pos_high << 16)); 
  }
}
void UpdateAndCheckPositions(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    uint16_t pos_low = *(Mpc_Position[i * 2 + 1]) & 0xFFFF;
    uint16_t pos_high = *(Mpc_Position[i * 2]) & 0xFFFF;
    position_new[i] = ((int32_t)pos_high << 16) | pos_low;
  }
  for (uint8_t i = 0; i < 4; i++)
  {
    position_and[i] = position_old[i] & position_new[i];
    position_new_plus_old[i] = position_new[i] - position_old[i];

    if (position_and[i] != 0 && position_new_plus_old[i] > 0)
    {
      semavalue[i] = __builtin_popcount(position_and[i]);
      if (semavalue[i] > 0 && semavalue[i] < 2)
      {
        HAL_GPIO_WritePin(JCOUT_GPIO_Port,JCOUT_Pin,GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim7);
      }
    }
  }
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

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

#ifdef USE_FULL_ASSERT
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
