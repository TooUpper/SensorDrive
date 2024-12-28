#ifndef __MLX90393_H
 #define __MLX90393_H
#include "gd32f4xx.h"
#include "gd32f4xx_it.h"
#include "systick.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "myiic.h"

void MLX90393_Init(void);//初始化MLX90393
void get_iic_addr(void);//轮询查找IIC地址
void get_data(void);//主动查询数据并打印

#endif