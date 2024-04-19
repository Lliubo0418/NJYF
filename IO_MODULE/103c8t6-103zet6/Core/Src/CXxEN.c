#include "CXxEN.h"
#include "main.h"


 /*
 * PA4 -----> MCU_CX2P  脉宽信号发送口2
 * PA5 -----> MCU_CX1P  脉宽信号发送口1
 * PA6 -----> MCU_CXCON  查询电子开关MC14066B
 * PA7 -----> MCU_CX1EN  查询1发送使能口
 * PB0 -----> MCU_CX2EN  查询2发送使能口
 *
 * MCU_CXCON口控制电子开关MC14066B 闭合时CX1和CX2连接在一起本机不检测查询信号，信号通到下一个设备
 * 断开时CX1和CX2断开，本机可以在CX1IN和CX2IN检测左右查询信号 或者在CX1和CX2上发送信号
 *
 * MCU_CX1EN口控制MCU_CX1P口能否发送查询脉冲
 * MCU_CX2EN口控制MCU_CX2P口能否发送查询脉冲
 *
 */
 
 
 void Set_MCU_CXCON_ON_OFF(unsigned char status){
		if(status==0){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);              //电子开关(高电平导通，低电平断开）
		}
		else{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);              //电子开关(高电平导通，低电平断开）
		}
 
 }
 
 void Set_MCU_CX1EN_ON_OFF(unsigned char flag)//设置是否允许在CX1处发送信号 1 允许 0 禁止
{
	if(flag==0)//高电平管子关断
		HAL_GPIO_WritePin(MCU_CX1EN_GPIO_Port,MCU_CX1EN_Pin ,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(MCU_CX1EN_GPIO_Port,MCU_CX1EN_Pin ,GPIO_PIN_RESET);
}

void Set_MCU_CX2EN_ON_OFF(unsigned char flag)//设置是否允许在CX2处发送信号 1 允许 0 禁止
{
	if(flag==0)//高电平管子关断
		HAL_GPIO_WritePin(MCU_CX2EN_GPIO_Port,MCU_CX2EN_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(MCU_CX2EN_GPIO_Port,MCU_CX2EN_Pin,GPIO_PIN_RESET);
}