#ifndef __EXTI_H
 #define __EXTI_H
#include "gd32f4xx.h"


/*
MLX90393 数据准备好之后，可以通过 INT 管脚输出高电平，告知 MCU 数据准备 OK。
这里使用该管脚，进行中断通知。使用 PC13 作为输入。
具体中断函数在 MLX90393 的驱动 C 文件实现，因为要引用大量 MLX90393 寄存器数据，不方便在这里实现。具体配置如下。
*/

void INT_Input_init(void);//输入中断初始化
void EXTI10_15_IRQHandler(void);//中断查询数据并打印

#endif