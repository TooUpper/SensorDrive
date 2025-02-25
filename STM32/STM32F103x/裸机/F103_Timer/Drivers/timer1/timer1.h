#ifndef __TIMER1_H__
#define __TIMER1_H__

#include "stm32f10x_tim.h"
#include "stdio.h"

/* Timer */
// 定义定时器相关的宏，便于修改
#define TIMERx                    TIM1                          // 定时器选择宏
#define TIMERx_CLK		          RCC_APB2Periph_TIM1  		    // 定时器时钟使能宏
#define TIMERx_PRESCALER          (7200 - 1)                    // 定时器预分频器
#define TIMERx_PERIOD             (10000 - 1)                   // 定时器自动重载值
#define TIMERx_REPETITION_COUNTER 0                             // 定时器重复计数器
#define TIMER_INT_FLAG            TIM_IT_Update                 // 定时器中断标志
#define TIMERx_IRQHandler         TIM1_UP_IRQHandler            // 定时器中断服务函数
#define TIMER_CLOCK_DIV        	  TIM_CKD_DIV1				    // 定时器时钟分频

/* NVIC */
// 定义与定时器中断相关的宏
#define TIMERx_IRQn                TIM1_UP_IRQn   // 定时器中断编号（根据实际定时器修改）
#define TIMERx_IRQ_PRIORITY        1              // 抢占优先级
#define TIMERx_IRQ_SUB_PRIORITY    2              // 子优先级
#define TIMERx_IRQ_ENABLE          ENABLE;  	  // 启用中断
#define TIMERx_IRQ_DISABLE         DISABLE; 	 // 禁用中断

/**/
void Timer1_Init(void);

#endif
