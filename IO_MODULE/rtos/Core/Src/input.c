#include "input.h"
#include "stdio.h"
#include "main.h"
#include "tim.h"
#include "math.h"


/* MCU_F1��MCU_F2��MCU_F3��MCU_F4ΪƵ�������ź�
 * MCU_F1_freq_get()��MCU_F2_freq_get()��MCU_F3_freq_get()��MCU_F4_freq_get()�ĸ�������ȡ��Ӧͨ����Ƶ��ֵ
 
 ***ģ�����������ѳ�ʼ��***
  GPIO_InitStruct.Pin = MCU_A2_Pin|MCU_A3_Pin|MCU_A4_Pin|MCU_A7_Pin;          
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
 
*/


//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���
uint8_t  MCU_F3_CAPTURE_STA=0;							//���벶��״̬		    				
uint16_t	MCU_F3_CAPTURE_VAL;							//���벶��ֵ(TIM5��16λ)

uint8_t  MCU_F2_CAPTURE_STA=0;						  				
uint16_t	MCU_F2_CAPTURE_VAL;						

extern uint32_t MCU_F1_capture_Buf[3] ;   //��ż���ֵ
extern uint8_t MCU_F1_capture_Cnt ;    //״̬��־λ
extern uint32_t MCU_F1_high_time;   //�ߵ�ƽʱ��							

uint8_t  MCU_F4_CAPTURE_STA=0;								    				
uint16_t	MCU_F4_CAPTURE_VAL;							



void MCU_F3_freq_get(){
	long long f3_freq=0;
	if(MCU_F3_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			f3_freq=MCU_F3_CAPTURE_STA&0X3F; 
			f3_freq*=65536;		 	    	//���ʱ���ܺ�
			f3_freq+=MCU_F3_CAPTURE_VAL;      //�õ��ܵĸߵ�ƽʱ��
			printf("MCU_F3 HIGH:%lld us\r\n",(long long)round(f3_freq/10.0)*10);//��ӡ�ܵĸߵ�ƽʱ��
			printf("MCU_F3 Ƶ��Ϊ %d Hz\r\n",1000000/((int)round(f3_freq/10.0)*10));
			MCU_F3_CAPTURE_STA=0;          //������һ�β���
		}

}

void MCU_F2_freq_get(){
	long long f2_freq=0;
	if(MCU_F2_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			f2_freq=MCU_F2_CAPTURE_STA&0X3F; 
			f2_freq*=65536;		 	    	//���ʱ���ܺ�
			f2_freq+=MCU_F2_CAPTURE_VAL;      //�õ��ܵĸߵ�ƽʱ��
			printf("MCU_F2 HIGH:%lld us\r\n",(long long)round(f2_freq/10.0)*10);//��ӡ�ܵĸߵ�ƽʱ��
			printf("MCU_F2 Ƶ��Ϊ %d Hz\r\n",1000000/((int)round(f2_freq/10.0)*10));
			MCU_F2_CAPTURE_STA=0;          //������һ�β���
		}

}

void MCU_F1_freq_get(){
		  switch (MCU_F1_capture_Cnt){
	case 0:
		MCU_F1_capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);	//�������벶��       ����: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(MCU_F1_capture_Buf[1]<MCU_F1_capture_Buf[0]){                  //�������Ȳ���ߵ�ƽ�󲶻�͵�ƽcapture_Buf[1]<capture_Buf[0],�϶����������
			MCU_F1_high_time = 0xC350+MCU_F1_capture_Buf[1]- MCU_F1_capture_Buf[0];
		}
		else{
		MCU_F1_high_time = MCU_F1_capture_Buf[1]- MCU_F1_capture_Buf[0];    //�ߵ�ƽʱ��  
		}
	HAL_Delay(500);
	printf("MCU_F1_high_time is %d us\r\n",(int)round(MCU_F1_high_time));         //ȡ�ٽ�����
	printf("MCU_F1 Ƶ��Ϊ %d Hz\r\n",1000000/((int)round(MCU_F1_high_time/10.0)*10));
		
		MCU_F1_capture_Cnt = 0;  //��ձ�־λ
		
		break;
	}

}

void MCU_F4_freq_get(){
	long long f4_freq=0;
	if(MCU_F4_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			f4_freq=MCU_F4_CAPTURE_STA&0X3F; 
			f4_freq*=65536;		 	    	//���ʱ���ܺ�
			f4_freq+=MCU_F4_CAPTURE_VAL;      //�õ��ܵĸߵ�ƽʱ��
			printf("MCU_F4 HIGH:%lld us\r\n",(long long)round(f4_freq/10.0)*10);//��ӡ�ܵĸߵ�ƽʱ��
			printf("MCU_F4 Ƶ��Ϊ %d Hz\r\n",1000000/((int)round(f4_freq/10.0)*10));
			MCU_F4_CAPTURE_STA=0;          //������һ�β���
		}

}



//��ʱ�������жϣ�����������жϴ���ص������� �ú�����HAL_TIM_IRQHandler�лᱻ����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//�����жϣ����������ʱִ��
{
	if(htim->Instance==TIM2){
	if((MCU_F4_CAPTURE_STA&0X80)==0)				//��δ�ɹ�����
	{
		if(MCU_F4_CAPTURE_STA&0X40)				//�Ѿ����񵽸ߵ�ƽ��
		{
			if((MCU_F4_CAPTURE_STA&0X3F)==0X3F)	//�ߵ�ƽ̫����
			{
				MCU_F4_CAPTURE_STA|=0X80;			//��ǳɹ�������һ��
				MCU_F4_CAPTURE_VAL=0XFFFF;
				}else MCU_F4_CAPTURE_STA++;
			}	 
		}
	}
}





