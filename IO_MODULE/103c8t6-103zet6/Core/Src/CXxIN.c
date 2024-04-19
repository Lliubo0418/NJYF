#include "CXxIN.h"
#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

 uint32_t capture_Buf[3] = {0};   //��ż���ֵ
 uint8_t capture_Cnt = 0;    //״̬��־λ
 uint32_t high_time;   //�ߵ�ƽʱ��



void TERM_SIGNAL_IC(){
		  switch (capture_Cnt){
	case 0:
		capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);	//�������벶��       ����: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(capture_Buf[1]<=capture_Buf[0]){                  //�������Ȳ���ߵ�ƽ�󲶻�͵�ƽcapture_Buf[1]<capture_Buf[0],�϶����������
			high_time = 0xC350+capture_Buf[1]- capture_Buf[0];
		}
		else{
		high_time = capture_Buf[1]- capture_Buf[0];    //�ߵ�ƽʱ��  
		}

	printf("high_time is %d us\r\n",(int)round(high_time));         //ȡ�ٽ�����
		
		capture_Cnt = 0;  //��ձ�־λ
		
		break;
	}
			

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	if(TIM2 == htim->Instance)
	{
		switch(capture_Cnt){
			case 1:
				capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//��ȡ��ǰ�Ĳ���ֵ.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				capture_Cnt++;
				break;
			case 2:
				capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//��ȡ��ǰ�Ĳ���ֵ.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_3); //ֹͣ����   ����: __HAL_TIM_DISABLE(&htim5);
				capture_Cnt++;    
		}
	
	}
	
}
