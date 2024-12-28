#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H
#include "gd32f4xx.h"


#define delay100ns                {__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();\
                                        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();\
                                        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();}
#define delay200ns                {delay100ns;delay100ns;}
#define delay300ns                {delay100ns;delay200ns;}
#define delay400ns                {delay200ns;delay200ns;}
#define delay500ns                {delay300ns;delay200ns;}


/*************************** IIC ************************************/
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);                                //发送IIC开始信号
void IIC_Stop(void);                                  //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);                        //IIC发送一个字节
uint8_t IIC_Read_Byte(void);//IIC读取一个字节

uint8_t        IIC_Wait_Ack(void);                                 //IIC等待ACK信号
void IIC_Ack(void);                                        //IIC发送ACK信号
void IIC_NAck(void);                                //IIC不发送ACK信号

#endif