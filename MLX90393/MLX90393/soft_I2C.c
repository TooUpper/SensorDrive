#include "soft_I2C.h"


/**************     IIC端口移植    *******************/
#define RCU_SPI_SCL              RCU_GPIOE                                //SCK
#define PORT_SPI_SCL             GPIOE
#define GPIO_SPI_SCL             GPIO_PIN_2

#define RCU_SPI_SDA              RCU_GPIOE                                //SDA
#define PORT_SPI_SDA             GPIOE
#define GPIO_SPI_SDA             GPIO_PIN_6

/*************************** IIC  ************************************/
// SCL:PE2 SDA1:PE6
#define IIC_SDA_IN         gpio_mode_set(PORT_SPI_SDA,GPIO_MODE_INPUT,GPIO_PUPD_PULLUP,GPIO_SPI_SDA)
#define IIC_SDA_OUT        {gpio_mode_set(PORT_SPI_SDA,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_SPI_SDA);\
                                gpio_output_options_set(PORT_SPI_SDA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_SPI_SDA);}

#define IIC_SDA_L                gpio_bit_write(PORT_SPI_SDA, GPIO_SPI_SDA, RESET)
#define IIC_SDA_H                gpio_bit_write(PORT_SPI_SDA, GPIO_SPI_SDA, SET)
#define IIC_READ_SDA           gpio_input_bit_get(PORT_SPI_SDA, GPIO_SPI_SDA)

#define IIC_SCL_L                gpio_bit_write(PORT_SPI_SCL, GPIO_SPI_SCL, RESET)
#define IIC_SCL_H                gpio_bit_write(PORT_SPI_SCL, GPIO_SPI_SCL, SET)

/*************************** IIC ************************************/

void IIC_Init(void)                //初始化IIC的IO口
{
        /* 使能时钟 */
        rcu_periph_clock_enable(RCU_SPI_SCL);
        rcu_periph_clock_enable(RCU_SPI_SDA);
        /* 配置SCL */
        gpio_mode_set(PORT_SPI_SCL,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_SPI_SCL);
        gpio_output_options_set(PORT_SPI_SCL,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_SPI_SCL);
        /* 配置SDA */
        gpio_mode_set(PORT_SPI_SDA,GPIO_MODE_OUTPUT,GPIO_PUPD_PULLUP,GPIO_SPI_SDA);
        gpio_output_options_set(PORT_SPI_SDA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_SPI_SDA);
        gpio_bit_write(PORT_SPI_SDA, GPIO_SPI_SDA, SET);
}
//产生IIC起始信号
void IIC_Start(void)
{
        IIC_SDA_OUT;//sda线输出
        IIC_SDA_H;
        IIC_SCL_H;
        delay500ns;
         IIC_SDA_L;
        delay500ns;
        IIC_SCL_L;//钳住I2C总线，准备发送或接收数据
}



//产生IIC停止信号
void IIC_Stop(void)
{
        IIC_SDA_OUT;//sda线输出
        IIC_SCL_L;
        IIC_SDA_L;
        //STOP:when CLK is high DATA change form low to high
         delay300ns;
        IIC_SCL_H;
        delay500ns;
        IIC_SDA_H;        //发送I2C总线结束信号
        delay200ns;
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
        uint16_t ucErrTime=0;
        IIC_SDA_IN;        //SDA设置为输入
        IIC_SDA_H;        delay500ns;
        IIC_SCL_H;        delay300ns;
        while(IIC_READ_SDA)
        {
                ucErrTime++;
                if(ucErrTime>1000)
                {
                        IIC_Stop();
                        return 1;
                }
        }
        IIC_SCL_L;//时钟输出0
        return 0;
}
//产生ACK应答
void IIC_Ack(void)
{
        IIC_SCL_L;
        IIC_SDA_OUT;//sda线输出
        IIC_SDA_L;
        delay500ns;
        IIC_SCL_H;
        delay500ns;
        IIC_SCL_L;
}
//不产生ACK应答
void IIC_NAck(void)
{
        IIC_SCL_L;
        IIC_SDA_OUT;//sda线输出
        IIC_SDA_H;
        delay500ns;
        IIC_SCL_H;
        delay500ns;
        IIC_SCL_L;
}
//IIC发送一个字
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t,tmp=txd;
                IIC_SDA_OUT;//sda线输出
                IIC_SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
                if((tmp<<t)&0x80)
                {IIC_SDA_H;}
                else
                {IIC_SDA_L;}
                delay400ns;   //对TEA5767这三个延时都是必须的
                IIC_SCL_H;
                delay500ns;
                IIC_SCL_L;
                delay100ns;
    }
}


//读1个字节
uint8_t IIC_Read_Byte(void)
{
        uint8_t i,tmp=0;
        IIC_SDA_IN;        //SDA设置为输入
    for(i=0;i<8;i++ )
                {
                        IIC_SCL_L;
                        delay500ns;
                        tmp<<=1;
                        IIC_SCL_H;
                        delay500ns;
                        if(IIC_READ_SDA)
                                tmp++;

                }
        IIC_SCL_L;
        return tmp;
}