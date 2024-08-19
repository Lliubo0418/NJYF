2024/08/19
验证定时器四路通道独立功能
TIM3:CHANNEL1  输出比较，相位延迟0.1ms
TIM3:CHANNEL2  输入捕获
TIM3:CHANNEL3  PWM 5ms 周期10ms
TIM3:CHANNEL4  PWM 2ms 周期10ms

TIM1:CHANNEL1  PWM 500ns 周期1ms
定时器需要考虑溢出问题，捕获定时器周期尽量大于待捕获定时器周期