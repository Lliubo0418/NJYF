#include "CXxEN.h"
#include "main.h"
#include "tim.h"


 /*
 * PA4 -----> MCU_CX2P  �����źŷ��Ϳ�2
 * PA5 -----> MCU_CX1P  �����źŷ��Ϳ�1
 * PA6 -----> MCU_CXCON  ��ѯ���ӿ���MC14066B
 * PA7 -----> MCU_CX1EN  ��ѯ1����ʹ�ܿ�
 * PB0 -----> MCU_CX2EN  ��ѯ2����ʹ�ܿ�
 *
 * MCU_CXCON�ڿ��Ƶ��ӿ���MC14066B �պ�ʱCX1��CX2������һ�𱾻�������ѯ�źţ��ź�ͨ����һ���豸
 * �Ͽ�ʱCX1��CX2�Ͽ�������������CX1IN��CX2IN������Ҳ�ѯ�ź� ������CX1��CX2�Ϸ����ź�
 *
 * MCU_CX1EN�ڿ���MCU_CX1P���ܷ��Ͳ�ѯ����
 * MCU_CX2EN�ڿ���MCU_CX2P���ܷ��Ͳ�ѯ����
 *
 */
 
 
uint16_t MCU_CX1_Pulse_width=0;
uint16_t MCU_CX2_Pulse_width=0;
unsigned char MCU_CX1_Pulse_enable=0;
unsigned char MCU_CX2_Pulse_enable=0;
unsigned char MCU_CX1_enable=0;
unsigned char MCU_CX2_enable=0;
 
 void Set_MCU_CXCON_ON_OFF(unsigned char status){
		if(status==0){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);              //���ӿ���(�ߵ�ƽ��ͨ���͵�ƽ�Ͽ���
		}
		else{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);              //���ӿ���(�ߵ�ƽ��ͨ���͵�ƽ�Ͽ���
		}
 
 }
 
 void Set_MCU_CX1EN_ON_OFF(unsigned char flag)//�����Ƿ�������CX1�������ź� 1 ��ֹ 0 ����
{
	if(flag==1){//�ߵ�ƽ���ӹض�
		HAL_GPIO_WritePin(MCU_CX1EN_GPIO_Port,MCU_CX1EN_Pin ,GPIO_PIN_SET);
		MCU_CX1_enable=0;
		}
	else
	{
		HAL_GPIO_WritePin(MCU_CX1EN_GPIO_Port,MCU_CX1EN_Pin ,GPIO_PIN_RESET);//�͵�ƽ��ͨ
		MCU_CX1_enable=1;
	}
}

void Set_MCU_CX2EN_ON_OFF(unsigned char flag)//�����Ƿ�������CX2�������ź� 1 ��ֹ 0 ����
{
	if(flag==1){//�ߵ�ƽ���ӹض�
		HAL_GPIO_WritePin(MCU_CX2EN_GPIO_Port,MCU_CX2EN_Pin ,GPIO_PIN_SET);
		MCU_CX2_enable=0;
		}
	else
	{
		HAL_GPIO_WritePin(MCU_CX2EN_GPIO_Port,MCU_CX2EN_Pin ,GPIO_PIN_RESET);//�͵�ƽ��ͨ
		MCU_CX2_enable=1;
	}
}

void Set_MCU_CX1P(uint8_t Pulse_width){       //��msΪ��λ
	if(Pulse_width!=0&&MCU_CX1_enable==1){
	MCU_CX1_Pulse_enable=1;
	MCU_CX1_Pulse_width=Pulse_width;
	}
	else{
	MCU_CX1_Pulse_enable=0;
	}
}

void Set_MCU_CX2P(uint8_t Pulse_width){       //��msΪ��λ
	if(Pulse_width&&MCU_CX2_enable==1){
	MCU_CX2_Pulse_enable=1;
	MCU_CX2_Pulse_width=Pulse_width;
	}
	else{
	MCU_CX2_Pulse_enable=0;
	}	


}



