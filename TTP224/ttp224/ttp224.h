#ifndef _TTP224_H_
#define _TTP224_H_

#include "stm32f10x.h"

#define RCC_TTP     RCC_APB2Periph_GPIOA
#define PORT_TTP    GPIOA

#define GPIO_IN1    GPIO_Pin_1
#define GPIO_IN2    GPIO_Pin_2
#define GPIO_IN3    GPIO_Pin_3
#define GPIO_IN4    GPIO_Pin_4

#define KEY_IN1   GPIO_ReadInputDataBit(PORT_TTP, GPIO_IN1)
#define KEY_IN2   GPIO_ReadInputDataBit(PORT_TTP, GPIO_IN2)
#define KEY_IN3   GPIO_ReadInputDataBit(PORT_TTP, GPIO_IN3)
#define KEY_IN4   GPIO_ReadInputDataBit(PORT_TTP, GPIO_IN4)

void TTP224_GPIO_Init(void);//引脚初始化
char Key_IN1_Scanf(void);//触摸按键1的输入状态
char Key_IN2_Scanf(void);//触摸按键2的输入状态
char Key_IN3_Scanf(void);//触摸按键3的输入状态
char Key_IN4_Scanf(void);//触摸按键4的输入状态

#endif