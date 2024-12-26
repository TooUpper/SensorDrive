#ifndef _AHT21_H_
#define _AHT21_H_

#include "gd32f4xx.h"

#define RCU_AHT21_SCL       RCU_GPIOD
#define PORT_AHT21_SCL      GPIOD
#define GPIO_AHT21_SCL      GPIO_PIN_4

#define RCU_AHT21_SDA       RCU_GPIOD
#define PORT_AHT21_SDA      GPIOD
#define GPIO_AHT21_SDA      GPIO_PIN_5


#define SDA_IN()  {gpio_mode_set(PORT_AHT21_SDA, GPIO_MODE_INPUT,  GPIO_PUPD_PULLUP, GPIO_AHT21_SDA);}  //SDA输入模式
#define SDA_OUT() {gpio_mode_set(PORT_AHT21_SDA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_AHT21_SDA);}  //SDA输出模式

#define SCL(BIT)  gpio_bit_write( PORT_AHT21_SCL, GPIO_AHT21_SCL, BIT?SET:RESET)
#define SDA(BIT)  gpio_bit_write( PORT_AHT21_SDA, GPIO_AHT21_SDA, BIT?SET:RESET)
#define GETSDA()  gpio_input_bit_get( PORT_AHT21_SDA, GPIO_AHT21_SDA)


void aht21_gpio_init(void);
char aht21_read_data(void);
float get_temperature(void);
float get_humidity(void);
#endif