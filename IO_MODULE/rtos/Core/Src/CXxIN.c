#include "CXxIN.h"
#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"


 uint32_t Term_capture_Buf[3] = {0};   //��ż���ֵ
 uint8_t Term_capture_Cnt = 0;    //״̬��־λ
 uint32_t Term_high_time;   //�ߵ�ƽʱ��

 uint32_t Cpu_capture_Buf[3] = {0};   //��ż���ֵ
 uint8_t Cpu_capture_Cnt = 0;    //״̬��־λ
 uint32_t Cpu_high_time;   //�ߵ�ƽʱ��

extern uint8_t  MCU_F3_CAPTURE_STA;							//���벶��״̬		    				
extern uint16_t	MCU_F3_CAPTURE_VAL;							//���벶��ֵ(TIM5��16λ)

extern uint8_t  MCU_F2_CAPTURE_STA;						  				
extern uint16_t	MCU_F2_CAPTURE_VAL;						

 uint32_t MCU_F1_capture_Buf[3] = {0};   //��ż���ֵ
 uint8_t MCU_F1_capture_Cnt = 0;    //״̬��־λ
 uint32_t MCU_F1_high_time;   //�ߵ�ƽʱ��						

extern uint8_t  MCU_F4_CAPTURE_STA;								    				
extern uint16_t	MCU_F4_CAPTURE_VAL;

 
void TERM_SIGNAL_IC(){
		  switch (Term_capture_Cnt){
	case 0:
		Term_capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);	//�������벶��       ����: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(Term_capture_Buf[1]<Term_capture_Buf[0]){                  //�������Ȳ���ߵ�ƽ�󲶻�͵�ƽTerm_capture_Buf[1]<Term_capture_Buf[0],�϶����������
			Term_high_time = 0xC350+Term_capture_Buf[1]- Term_capture_Buf[0];
		}
		else{
		Term_high_time = Term_capture_Buf[1]- Term_capture_Buf[0];    //�ߵ�ƽʱ��  
		}
	HAL_Delay(500);
	printf("Term_high_time is %d us\r\n",(int)round(Term_high_time));         //ȡ�ٽ�����
		
		
		Term_capture_Cnt = 0;  //��ձ�־λ
		
		break;
	}
			

}
void CPU_SIGNAL_IC(){
		  switch (Cpu_capture_Cnt){
	case 0:
		Cpu_capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);	//�������벶��       ����: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(Cpu_capture_Buf[1]<Cpu_capture_Buf[0]){                  //�������Ȳ���ߵ�ƽ�󲶻�͵�ƽcapture_Buf[1]<capture_Buf[0],�϶����������
			Cpu_high_time = 0xC350+Cpu_capture_Buf[1]- Cpu_capture_Buf[0];
		}
		else{
		Cpu_high_time = Cpu_capture_Buf[1]- Cpu_capture_Buf[0];    //�ߵ�ƽʱ��  
		}
	HAL_Delay(500);
	printf("Cpu_high_time is %d us\r\n",(int)round(Cpu_high_time));         //ȡ�ٽ�����
		
		Cpu_capture_Cnt = 0;  //��ձ�־λ
		
		break;
	}
			

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	if(TIM2 == htim->Instance)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3){                     //CX1
		switch(Term_capture_Cnt){
			case 1:
				Term_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//��ȡ��ǰ�Ĳ���ֵ.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				Term_capture_Cnt++;
				break;
			case 2:
				Term_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//��ȡ��ǰ�Ĳ���ֵ.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_3); //ֹͣ����   ����: __HAL_TIM_DISABLE(&htim2);
				Term_capture_Cnt++;    
		}
	}
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4){                 //CX2
		switch(Cpu_capture_Cnt){
			case 1:
				Cpu_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);//��ȡ��ǰ�Ĳ���ֵ.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				Cpu_capture_Cnt++;
				break;
			case 2:
				Cpu_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);//��ȡ��ǰ�Ĳ���ֵ.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_4); //ֹͣ����   ����: __HAL_TIM_DISABLE(&htim2);
				Cpu_capture_Cnt++;    
		}
		
	}
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){
		switch(MCU_F1_capture_Cnt){
			case 1:
				MCU_F1_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);//��ȡ��ǰ�Ĳ���ֵ.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
				MCU_F1_capture_Cnt++;
				break;
			case 2:
				MCU_F1_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);//��ȡ��ǰ�Ĳ���ֵ.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_2); //ֹͣ����   ����: __HAL_TIM_DISABLE(&htim2);
				MCU_F1_capture_Cnt++;    
		}
		
		
		}

	}
	if(TIM4 == htim->Instance)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
			if((MCU_F2_CAPTURE_STA&0X80)==0)				//��δ�ɹ�����
	{
		if(MCU_F2_CAPTURE_STA&0X40)				//����һ���½��� 		
		{	  			
			MCU_F2_CAPTURE_STA|=0X80;				//��ǳɹ�����һ�θߵ�ƽ����
            MCU_F2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);//��ȡ��ǰ�Ĳ���ֵ.
			TIM_RESET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
            TIM_SET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//����TIM4ͨ��2�����ز���
		}else  										//��δ��ʼ,��һ�β���������
		{
			MCU_F2_CAPTURE_STA=0;					//���
			MCU_F2_CAPTURE_VAL=0;
			MCU_F2_CAPTURE_STA|=0X40;				//��ǲ�����������
			__HAL_TIM_DISABLE(&htim4);      	//�رն�ʱ��2
			__HAL_TIM_SET_COUNTER(&htim4,0);
			TIM_RESET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
			TIM_SET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//��ʱ��4ͨ��1����Ϊ�½��ز���
			__HAL_TIM_ENABLE(&htim4);		//ʹ�ܶ�ʱ��4
				}		    
			}	
		}
	}
	
		if(TIM3 == htim->Instance)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
			if((MCU_F4_CAPTURE_STA&0X80)==0)				//��δ�ɹ�����
	{
		if(MCU_F4_CAPTURE_STA&0X40)				//����һ���½��� 		
		{	  			
			MCU_F4_CAPTURE_STA|=0X80;				//��ǳɹ�����һ�θߵ�ƽ����
            MCU_F4_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//��ȡ��ǰ�Ĳ���ֵ.
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
            TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//����TIM4ͨ��2�����ز���
		}else  										//��δ��ʼ,��һ�β���������
		{
			MCU_F4_CAPTURE_STA=0;					//���
			MCU_F4_CAPTURE_VAL=0;
			MCU_F4_CAPTURE_STA|=0X40;				//��ǲ�����������
			__HAL_TIM_DISABLE(&htim3);      	//�رն�ʱ��2
			__HAL_TIM_SET_COUNTER(&htim3,0);
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//��ʱ��4ͨ��1����Ϊ�½��ز���
			__HAL_TIM_ENABLE(&htim3);		//ʹ�ܶ�ʱ��4
				}		    
			}	
		}
		
				if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){
			if((MCU_F3_CAPTURE_STA&0X80)==0)				//��δ�ɹ�����
	{
		if(MCU_F3_CAPTURE_STA&0X40)				//����һ���½��� 		
		{	  			
			MCU_F3_CAPTURE_STA|=0X80;				//��ǳɹ�����һ�θߵ�ƽ����
            MCU_F3_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//��ȡ��ǰ�Ĳ���ֵ.
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2);   //һ��Ҫ�����ԭ�������ã���
            TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//����TIM4ͨ��2�����ز���
		}else  										//��δ��ʼ,��һ�β���������
		{
			MCU_F3_CAPTURE_STA=0;					//���
			MCU_F3_CAPTURE_VAL=0;
			MCU_F3_CAPTURE_STA|=0X40;				//��ǲ�����������
			__HAL_TIM_DISABLE(&htim3);      	//�رն�ʱ��2
			__HAL_TIM_SET_COUNTER(&htim3,0);
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2);   //һ��Ҫ�����ԭ�������ã���
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//��ʱ��4ͨ��1����Ϊ�½��ز���
			__HAL_TIM_ENABLE(&htim3);		//ʹ�ܶ�ʱ��4
				}		    
			}	
		}
	}
}
