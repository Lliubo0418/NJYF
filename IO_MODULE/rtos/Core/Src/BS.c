#include "BS.h"
#include "main.h"





void Set_MCU_OUT_status(unsigned char flag){    
	
	if(flag){                                             
		//HAL_GPIO_WritePin(MCU_BS_OUT1_GPIO_Port,MCU_BS_OUT1_Pin,GPIO_PIN_SET);       //去掉串口后打开
		//HAL_GPIO_WritePin(MCU_BS_OUT2_GPIO_Port,MCU_BS_OUT2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MCU_BS_OUT3_GPIO_Port,MCU_BS_OUT3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MCU_BS_OUT4_GPIO_Port,MCU_BS_OUT4_Pin,GPIO_PIN_SET);
	}
	else{                                  //有闭锁，关闭输出
		//HAL_GPIO_WritePin(MCU_BS_OUT1_GPIO_Port,MCU_BS_OUT1_Pin,GPIO_PIN_RESET);       //去掉串口后打开
		//HAL_GPIO_WritePin(MCU_BS_OUT2_GPIO_Port,MCU_BS_OUT2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MCU_BS_OUT3_GPIO_Port,MCU_BS_OUT3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MCU_BS_OUT4_GPIO_Port,MCU_BS_OUT4_Pin,GPIO_PIN_RESET);	
	}
}


unsigned char Get_CPU_BS0IN_status(void){      //查询BS0引脚信号，1为无闭锁，0为线上有闭锁       
	
return HAL_GPIO_ReadPin(MCU_BS0IN_GPIO_Port,MCU_BS0IN_Pin);

}
