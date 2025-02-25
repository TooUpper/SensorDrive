#include "timer1.h"

void Timer1_Config(void) {
    // 启用定时器1的时钟
    RCC_APB2PeriphClockCmd(TIMERx_CLK, ENABLE);
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIMERx);
    
    // 配置定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    // 计算定时器的预分频器和自动重载值，假设使用72MHz的时钟，目标为1秒
	// 
    
    TIM_TimeBaseStructure.TIM_Period = TIMERx_PERIOD;            // 自动重载值，计数到 9999 触发中断
    TIM_TimeBaseStructure.TIM_Prescaler = TIMERx_PRESCALER;      // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数模式
	// 定时器更新事件的触发频率,
	// 设置 TIM_RepetitionCounter = 1 时，定时器会在 每次更新事件 之后 延迟一个周期
	TIM_TimeBaseStructure.TIM_RepetitionCounter = TIMERx_REPETITION_COUNTER; 
    TIM_TimeBaseInit(TIMERx, &TIM_TimeBaseStructure);
    
    // 启用定时器1更新中断
    TIM_ITConfig(TIMERx, TIMER_INT_FLAG, ENABLE);
    
    // 启动定时器1
    TIM_Cmd(TIMERx, ENABLE);
}

void Timer1_NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 配置定时器1的中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIMERx_IRQn;  // 定时器1更新中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMERx_IRQ_PRIORITY;  // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMERx_IRQ_SUB_PRIORITY;         // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = TIMERx_IRQ_ENABLE;            // 启用中断
    NVIC_Init(&NVIC_InitStructure);
}

void TIMERx_IRQHandler(void) {
    // 检查是否是更新中断
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        
        // 打印"run"
        printf("Run...");
    }
}



void Timer1_Init(void){
	Timer1_NVIC_Config();
	Timer1_Config();
}
