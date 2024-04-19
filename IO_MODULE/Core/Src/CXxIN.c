#include "CXxIN.h"
#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

 uint32_t capture_Buf[3] = {0};   //存放计数值
 uint8_t capture_Cnt = 0;    //状态标志位
 uint32_t high_time;   //高电平时间



void TERM_SIGNAL_IC(){
		  switch (capture_Cnt){
	case 0:
		capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);	//启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(capture_Buf[1]<=capture_Buf[0]){                  //不存在先捕获高电平后捕获低电平capture_Buf[1]<capture_Buf[0],肯定是有了溢出
			high_time = 0xC350+capture_Buf[1]- capture_Buf[0];
		}
		else{
		high_time = capture_Buf[1]- capture_Buf[0];    //高电平时间  
		}

	printf("high_time is %d us\r\n",(int)round(high_time));         //取临近整数
		
		capture_Cnt = 0;  //清空标志位
		
		break;
	}
			

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	if(TIM2 == htim->Instance)
	{
		switch(capture_Cnt){
			case 1:
				capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//获取当前的捕获值.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				capture_Cnt++;
				break;
			case 2:
				capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//获取当前的捕获值.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_3); //停止捕获   或者: __HAL_TIM_DISABLE(&htim5);
				capture_Cnt++;    
		}
	
	}
	
}
