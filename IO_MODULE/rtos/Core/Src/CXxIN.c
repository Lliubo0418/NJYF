#include "CXxIN.h"
#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"


 uint32_t Term_capture_Buf[3] = {0};   //存放计数值
 uint8_t Term_capture_Cnt = 0;    //状态标志位
 uint32_t Term_high_time;   //高电平时间

 uint32_t Cpu_capture_Buf[3] = {0};   //存放计数值
 uint8_t Cpu_capture_Cnt = 0;    //状态标志位
 uint32_t Cpu_high_time;   //高电平时间

extern uint8_t  MCU_F3_CAPTURE_STA;							//输入捕获状态		    				
extern uint16_t	MCU_F3_CAPTURE_VAL;							//输入捕获值(TIM5是16位)

extern uint8_t  MCU_F2_CAPTURE_STA;						  				
extern uint16_t	MCU_F2_CAPTURE_VAL;						

 uint32_t MCU_F1_capture_Buf[3] = {0};   //存放计数值
 uint8_t MCU_F1_capture_Cnt = 0;    //状态标志位
 uint32_t MCU_F1_high_time;   //高电平时间						

extern uint8_t  MCU_F4_CAPTURE_STA;								    				
extern uint16_t	MCU_F4_CAPTURE_VAL;

 
void TERM_SIGNAL_IC(){
		  switch (Term_capture_Cnt){
	case 0:
		Term_capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);	//启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(Term_capture_Buf[1]<Term_capture_Buf[0]){                  //不存在先捕获高电平后捕获低电平Term_capture_Buf[1]<Term_capture_Buf[0],肯定是有了溢出
			Term_high_time = 0xC350+Term_capture_Buf[1]- Term_capture_Buf[0];
		}
		else{
		Term_high_time = Term_capture_Buf[1]- Term_capture_Buf[0];    //高电平时间  
		}
	HAL_Delay(500);
	printf("Term_high_time is %d us\r\n",(int)round(Term_high_time));         //取临近整数
		
		
		Term_capture_Cnt = 0;  //清空标志位
		
		break;
	}
			

}
void CPU_SIGNAL_IC(){
		  switch (Cpu_capture_Cnt){
	case 0:
		Cpu_capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);	//启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
		break;
	case 3:
		if(Cpu_capture_Buf[1]<Cpu_capture_Buf[0]){                  //不存在先捕获高电平后捕获低电平capture_Buf[1]<capture_Buf[0],肯定是有了溢出
			Cpu_high_time = 0xC350+Cpu_capture_Buf[1]- Cpu_capture_Buf[0];
		}
		else{
		Cpu_high_time = Cpu_capture_Buf[1]- Cpu_capture_Buf[0];    //高电平时间  
		}
	HAL_Delay(500);
	printf("Cpu_high_time is %d us\r\n",(int)round(Cpu_high_time));         //取临近整数
		
		Cpu_capture_Cnt = 0;  //清空标志位
		
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
				Term_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//获取当前的捕获值.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				Term_capture_Cnt++;
				break;
			case 2:
				Term_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//获取当前的捕获值.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_3); //停止捕获   或者: __HAL_TIM_DISABLE(&htim2);
				Term_capture_Cnt++;    
		}
	}
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4){                 //CX2
		switch(Cpu_capture_Cnt){
			case 1:
				Cpu_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);//获取当前的捕获值.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				Cpu_capture_Cnt++;
				break;
			case 2:
				Cpu_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);//获取当前的捕获值.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_4); //停止捕获   或者: __HAL_TIM_DISABLE(&htim2);
				Cpu_capture_Cnt++;    
		}
		
	}
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){
		switch(MCU_F1_capture_Cnt){
			case 1:
				MCU_F1_capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);//获取当前的捕获值.
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				MCU_F1_capture_Cnt++;
				break;
			case 2:
				MCU_F1_capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);//获取当前的捕获值.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_2); //停止捕获   或者: __HAL_TIM_DISABLE(&htim2);
				MCU_F1_capture_Cnt++;    
		}
		
		
		}

	}
	if(TIM4 == htim->Instance)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
			if((MCU_F2_CAPTURE_STA&0X80)==0)				//还未成功捕获
	{
		if(MCU_F2_CAPTURE_STA&0X40)				//捕获到一个下降沿 		
		{	  			
			MCU_F2_CAPTURE_STA|=0X80;				//标记成功捕获到一次高电平脉宽
            MCU_F2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);//获取当前的捕获值.
			TIM_RESET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
            TIM_SET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//配置TIM4通道2上升沿捕获
		}else  										//还未开始,第一次捕获上升沿
		{
			MCU_F2_CAPTURE_STA=0;					//清空
			MCU_F2_CAPTURE_VAL=0;
			MCU_F2_CAPTURE_STA|=0X40;				//标记捕获到了上升沿
			__HAL_TIM_DISABLE(&htim4);      	//关闭定时器2
			__HAL_TIM_SET_COUNTER(&htim4,0);
			TIM_RESET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
			TIM_SET_CAPTUREPOLARITY(&htim4,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//定时器4通道1设置为下降沿捕获
			__HAL_TIM_ENABLE(&htim4);		//使能定时器4
				}		    
			}	
		}
	}
	
		if(TIM3 == htim->Instance)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
			if((MCU_F4_CAPTURE_STA&0X80)==0)				//还未成功捕获
	{
		if(MCU_F4_CAPTURE_STA&0X40)				//捕获到一个下降沿 		
		{	  			
			MCU_F4_CAPTURE_STA|=0X80;				//标记成功捕获到一次高电平脉宽
            MCU_F4_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//获取当前的捕获值.
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
            TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//配置TIM4通道2上升沿捕获
		}else  										//还未开始,第一次捕获上升沿
		{
			MCU_F4_CAPTURE_STA=0;					//清空
			MCU_F4_CAPTURE_VAL=0;
			MCU_F4_CAPTURE_STA|=0X40;				//标记捕获到了上升沿
			__HAL_TIM_DISABLE(&htim3);      	//关闭定时器2
			__HAL_TIM_SET_COUNTER(&htim3,0);
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//定时器4通道1设置为下降沿捕获
			__HAL_TIM_ENABLE(&htim3);		//使能定时器4
				}		    
			}	
		}
		
				if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){
			if((MCU_F3_CAPTURE_STA&0X80)==0)				//还未成功捕获
	{
		if(MCU_F3_CAPTURE_STA&0X40)				//捕获到一个下降沿 		
		{	  			
			MCU_F3_CAPTURE_STA|=0X80;				//标记成功捕获到一次高电平脉宽
            MCU_F3_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//获取当前的捕获值.
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
            TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//配置TIM4通道2上升沿捕获
		}else  										//还未开始,第一次捕获上升沿
		{
			MCU_F3_CAPTURE_STA=0;					//清空
			MCU_F3_CAPTURE_VAL=0;
			MCU_F3_CAPTURE_STA|=0X40;				//标记捕获到了上升沿
			__HAL_TIM_DISABLE(&htim3);      	//关闭定时器2
			__HAL_TIM_SET_COUNTER(&htim3,0);
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//定时器4通道1设置为下降沿捕获
			__HAL_TIM_ENABLE(&htim3);		//使能定时器4
				}		    
			}	
		}
	}
}
