/********************************************************************************
  * 文 件 名: main.c
  * 版 本 号: 初版
  * 修改作者: LC
  * 修改日期: 2022年10月5日
  * 功能介绍:
  ******************************************************************************
  * 注意事项:
*********************************************************************************/
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "mlx90393.h"
#include "myiic.h"
#include "myexti.h"
/************************************************
函数名称 ： main
功    能 ： 主函数
参    数 ： 无
返 回 值 ： 无
作    者 ： Kwin
*************************************************/
int main(void)
{
        nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
        systick_config();   // 滴答定时器初始化
        led_gpio_config();  // led初始化
        usart_gpio_config(115200);
        IIC_Init();
        gpio_bit_write(PORT_LED2,PIN_LED2,RESET);                          // LED2输出高电平
        printf("helloJLC\r\n");
        delay_1ms(5000);
        MLX90393_Init();
//        INT_Input_init();
        while(1) {
                delay_1ms(1000);
                gpio_bit_toggle(PORT_LED2,PIN_LED2);//led2闪烁
//                get_iic_addr();
                get_data();
        }
}