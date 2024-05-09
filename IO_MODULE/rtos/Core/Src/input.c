#include "input.h"
#include "stdio.h"
#include "main.h"
#include "tim.h"
#include "math.h"


/* MCU_F1、MCU_F2、MCU_F3、MCU_F4为频率输入信号
 * MCU_F1_freq_get()、MCU_F2_freq_get()、MCU_F3_freq_get()、MCU_F4_freq_get()四个函数获取相应通道的频率值
 
 ***模拟输入引脚已初始化***
  GPIO_InitStruct.Pin = MCU_A2_Pin|MCU_A3_Pin|MCU_A4_Pin|MCU_A7_Pin;          
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
 
*/


//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
uint8_t  MCU_F3_CAPTURE_STA=0;							//输入捕获状态		    				
uint16_t	MCU_F3_CAPTURE_VAL;							//输入捕获值(TIM5是16位)

uint8_t  MCU_F2_CAPTURE_STA=0;						  				
uint16_t	MCU_F2_CAPTURE_VAL;						

extern uint32_t MCU_F1_capture_Buf[3] ;   //存放计数值
extern uint8_t MCU_F1_capture_Cnt ;    //状态标志位
extern uint32_t MCU_F1_high_time;   //高电平时间							

uint8_t  MCU_F4_CAPTURE_STA=0;								    				
uint16_t	MCU_F4_CAPTURE_VAL;							



void MCU_F3_freq_get(){
	long long f3_freq=0;
	if(MCU_F3_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			f3_freq=MCU_F3_CAPTURE_STA&0X3F; 
			f3_freq*=65536;		 	    	//溢出时间总和
			f3_freq+=MCU_F3_CAPTURE_VAL;      //得到总的高电平时间
			printf("MCU_F3 HIGH:%lld us\r\n",(long long)round(f3_freq/10.0)*10);//打印总的高点平时间
			printf("MCU_F3 频率为 %d Hz\r\n",1000000/((int)round(f3_freq/10.0)*10));
			MCU_F3_CAPTURE_STA=0;          //开启下一次捕获
		}

}

void MCU_F2_freq_get(){
	long long f2_freq=0;
	if(MCU_F2_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			f2_freq=MCU_F2_CAPTURE_STA&0X3F; 
			f2_freq*=65536;		 	    	//溢出时间总和
			f2_freq+=MCU_F2_CAPTURE_VAL;      //得到总的高电平时间
			printf("MCU_F2 HIGH:%lld us\r\n",(long long)round(f2_freq/10.0)*10);//打印总的高点平时间
			printf("MCU_F2 频率为 %d Hz\r\n",1000000/((int)round(f2_freq/10.0)*10));
			MCU_F2_CAPTURE_STA=0;          //开启下一次捕获
		}

}

void MCU_F1_freq_get(){
		  switch (MCU_F1_capture_Cnt){
	case 0:
		MCU_F1_capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);	//启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(MCU_F1_capture_Buf[1]<MCU_F1_capture_Buf[0]){                  //不存在先捕获高电平后捕获低电平capture_Buf[1]<capture_Buf[0],肯定是有了溢出
			MCU_F1_high_time = 0xC350+MCU_F1_capture_Buf[1]- MCU_F1_capture_Buf[0];
		}
		else{
		MCU_F1_high_time = MCU_F1_capture_Buf[1]- MCU_F1_capture_Buf[0];    //高电平时间  
		}
	HAL_Delay(500);
	printf("MCU_F1_high_time is %d us\r\n",(int)round(MCU_F1_high_time));         //取临近整数
	printf("MCU_F1 频率为 %d Hz\r\n",1000000/((int)round(MCU_F1_high_time/10.0)*10));
		
		MCU_F1_capture_Cnt = 0;  //清空标志位
		
		break;
	}

}

void MCU_F4_freq_get(){
	long long f4_freq=0;
	if(MCU_F4_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			f4_freq=MCU_F4_CAPTURE_STA&0X3F; 
			f4_freq*=65536;		 	    	//溢出时间总和
			f4_freq+=MCU_F4_CAPTURE_VAL;      //得到总的高电平时间
			printf("MCU_F4 HIGH:%lld us\r\n",(long long)round(f4_freq/10.0)*10);//打印总的高点平时间
			printf("MCU_F4 频率为 %d Hz\r\n",1000000/((int)round(f4_freq/10.0)*10));
			MCU_F4_CAPTURE_STA=0;          //开启下一次捕获
		}

}



//定时器更新中断（计数溢出）中断处理回调函数， 该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//更新中断（溢出）发生时执行
{
	if(htim->Instance==TIM2){
	if((MCU_F4_CAPTURE_STA&0X80)==0)				//还未成功捕获
	{
		if(MCU_F4_CAPTURE_STA&0X40)				//已经捕获到高电平了
		{
			if((MCU_F4_CAPTURE_STA&0X3F)==0X3F)	//高电平太长了
			{
				MCU_F4_CAPTURE_STA|=0X80;			//标记成功捕获了一次
				MCU_F4_CAPTURE_VAL=0XFFFF;
				}else MCU_F4_CAPTURE_STA++;
			}	 
		}
	}
}





