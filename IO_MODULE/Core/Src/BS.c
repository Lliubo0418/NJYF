#include "BS.h"
#include "main.h"
#include "stdio.h"





void Set_MCU_OUT_status(unsigned char flag){    
	
	if(flag){                                             
		//HAL_GPIO_WritePin(MCU_BS_OUT1_GPIO_Port,MCU_BS_OUT1_Pin,GPIO_PIN_SET);       //ȥ�����ں��
		//HAL_GPIO_WritePin(MCU_BS_OUT2_GPIO_Port,MCU_BS_OUT2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MCU_BS_OUT3_GPIO_Port,MCU_BS_OUT3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MCU_BS_OUT4_GPIO_Port,MCU_BS_OUT4_Pin,GPIO_PIN_SET);
		printf("�ޱ������������\r\n");
	}
	else{                                  //�б������ر����
		//HAL_GPIO_WritePin(MCU_BS_OUT1_GPIO_Port,MCU_BS_OUT1_Pin,GPIO_PIN_RESET);       //ȥ�����ں��
		//HAL_GPIO_WritePin(MCU_BS_OUT2_GPIO_Port,MCU_BS_OUT2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MCU_BS_OUT3_GPIO_Port,MCU_BS_OUT3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MCU_BS_OUT4_GPIO_Port,MCU_BS_OUT4_Pin,GPIO_PIN_RESET);	
		printf("�б������ر����\r\n");
	}
}


void Get_CPU_BS0IN_status(void){      //��ѯBS0�����źţ�1Ϊ�ޱ�����0Ϊ�����б���  

	unsigned char BS_status=0;	
	BS_status=HAL_GPIO_ReadPin(MCU_BS0IN_GPIO_Port,MCU_BS0IN_Pin);
	Set_MCU_OUT_status(BS_status);

}
