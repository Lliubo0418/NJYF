#include "BS.h"
#include "main.h"
#include "stdio.h"





void Set_MCU_OUT_status(unsigned char flag){    
	
	if(flag){                                             
		//HAL_GPIO_WritePin(MCU_BS_OUT1_GPIO_Port,MCU_BS_OUT1_Pin,GPIO_PIN_SET);       //去掉串口后打开
		//HAL_GPIO_WritePin(MCU_BS_OUT2_GPIO_Port,MCU_BS_OUT2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MCU_BS_OUT3_GPIO_Port,MCU_BS_OUT3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MCU_BS_OUT4_GPIO_Port,MCU_BS_OUT4_Pin,GPIO_PIN_SET);
		printf("无闭锁，正常输出\r\n");
	}
	else{                                  //有闭锁，关闭输出
		//HAL_GPIO_WritePin(MCU_BS_OUT1_GPIO_Port,MCU_BS_OUT1_Pin,GPIO_PIN_RESET);       //去掉串口后打开
		//HAL_GPIO_WritePin(MCU_BS_OUT2_GPIO_Port,MCU_BS_OUT2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MCU_BS_OUT3_GPIO_Port,MCU_BS_OUT3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MCU_BS_OUT4_GPIO_Port,MCU_BS_OUT4_Pin,GPIO_PIN_RESET);	
		printf("有闭锁，关闭输出\r\n");
	}
}


void Get_CPU_BS0IN_status(void){      //查询BS0引脚信号，1为无闭锁，0为线上有闭锁  

	unsigned char BS_status=0;	
	BS_status=HAL_GPIO_ReadPin(MCU_BS0IN_GPIO_Port,MCU_BS0IN_Pin);
	Set_MCU_OUT_status(BS_status);

}
