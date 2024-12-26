
#include "AHT21.h"
#include "systick.h"
#include "bsp_usart.h"
#include "stdio.h"

static float temperature = 0;
static float humidity = 0;
/******************************************************************
 * 函 数 名 称：aht21_gpio_init
 * 函 数 说 明：对AHT21的IIC引脚初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：
******************************************************************/
void aht21_gpio_init(void){
    //打开SDA与SCL的引脚时钟
    rcu_periph_clock_enable(RCU_AHT21_SCL);
    rcu_periph_clock_enable(RCU_AHT21_SDA);

    //设置SCL引脚模式为上拉输出
    gpio_mode_set(PORT_AHT21_SCL, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_AHT21_SCL);
    //设置引脚为开漏模式，翻转速度2MHz
    gpio_output_options_set(PORT_AHT21_SCL, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_AHT21_SCL);
    //设置引脚输出高电平SCL等待信号
    gpio_bit_write(PORT_AHT21_SCL, GPIO_AHT21_SCL, SET);

    //设置SDA引脚
    gpio_mode_set(PORT_AHT21_SDA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_AHT21_SDA);
    gpio_output_options_set(PORT_AHT21_SDA, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_AHT21_SDA);
    gpio_bit_write(PORT_AHT21_SDA, GPIO_AHT21_SDA, SET);
}

/******************************************************************
 * 函 数 名 称：IIC_Start
 * 函 数 说 明：发送起始信号
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void IIC_Start(void){
        SDA_OUT();
        SDA(1);
        SCL(1);
        delay_1us(5);
        SDA(0);
        delay_1us(5);
        SCL(0);
        delay_1us(5);
}

/******************************************************************
 * 函 数 名 称：IIC_Stop
 * 函 数 说 明：停止信号
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void IIC_Stop(void){
        SDA_OUT();
        SCL(0);
        SDA(0);
        SCL(1);
        delay_1us(5);
        SDA(1);
        delay_1us(5);
}

/******************************************************************
 * 函 数 名 称：IIC_Send_Nack
 * 函 数 说 明：发送非应答
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void IIC_Send_Nack(void)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        SDA(1);
        SCL(1);
        delay_1us(5);
        SCL(0);
        SDA(0);
}

/******************************************************************
 * 函 数 名 称：IIC_Send_Ack
 * 函 数 说 明：发送应答
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void IIC_Send_Ack(void)
{
        SDA_OUT();
        SCL(0);
        SDA(1);
        SDA(0);
        SCL(1);
        delay_1us(5);
        SCL(0);
        SDA(1);
}

/**********************************************************
 * 函 数 名 称：I2C_WaitAck
 * 函 数 功 能：等待从机应答
 * 传 入 参 数：无
 * 函 数 返 回：1=非应答         0=应答
 * 作       者：LCKFB
 * 备       注：无
**********************************************************/
unsigned char I2C_WaitAck(void)
{
        char ack = 0;
        unsigned char ack_flag = 10;
        SCL(0);
        SDA(1);
        SDA_IN();
        delay_1us(5);
        SCL(1);
    delay_1us(5);

        while( (GETSDA()==1)  &&  ( ack_flag ) )
        {
                ack_flag--;
                delay_1us(5);
        }

        //非应答
        if( ack_flag <= 0 )
        {
                IIC_Stop();
                return 1;
        }
        else//应答
        {
                SCL(0);
                SDA_OUT();
        }
        return ack;
}

/******************************************************************
 * 函 数 名 称：IIC_Send_Byte
 * 函 数 说 明：发送一个字节
 * 函 数 形 参：dat：发送的字节数据
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void IIC_Send_Byte(uint8_t dat)
{
        int i = 0;
        SDA_OUT();
        SCL(0);

        for( i = 0; i < 8; i++ )
        {
                SDA( (dat & 0x80) >> 7 );
                delay_1us(1);
                SCL(1);
                delay_1us(5);
                SCL(0);
                delay_1us(5);
                dat<<=1;
        }
}

/******************************************************************
 * 函 数 名 称：IIC_Read_Byte
 * 函 数 说 明：接收一个字节
 * 函 数 形 参：无
 * 函 数 返 回：接收到的数据
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
unsigned char IIC_Read_Byte(void)
{
        unsigned char i,receive=0;
        SDA_IN();//SDA设置为输入
        for(i=0;i<8;i++ )
        {
                SCL(0);
                delay_1us(5);
                SCL(1);
                delay_1us(5);
                receive<<=1;
                if( GETSDA() )
                {
                        receive |= 1;
                }
        }
        SCL(0);
        return receive;
}

/******************************************************************
 * 函 数 名 称：aht21_read_status
 * 函 数 说 明：读取AHT21的状态寄存器
 * 函 数 形 参：无
 * 函 数 返 回：读取到的状态数据
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
uint8_t aht21_read_status(void)
{
    uint8_t status_register_address = 0x71;

        uint8_t status_byte;

        IIC_Start();

    IIC_Send_Byte( status_register_address );
    if( I2C_WaitAck() == 1 ) printf("warning -1\r\n");

        status_byte = IIC_Read_Byte();
        IIC_Send_Nack();
        IIC_Stop();

        return status_byte;
}


/******************************************************************
 * 函 数 名 称：aht21_send_gather_command
 * 函 数 说 明：向AHT21发送采集命令
 * 函 数 形 参：无
 * 函 数 返 回：1：器件识别失败
 *              2：发送采集命令失败
 *              3：发送数据1失败
 *              4：发送数据2失败
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
uint8_t aht21_send_gather_command(void)
{
    uint8_t device_addr = 0x70;//器件地址
    uint8_t gather_command = 0xac;//采集命令
    uint8_t gather_command_parameter_1 = 0x33;//采集参数1
    uint8_t gather_command_parameter_2 = 0x00;//采集参数2

    IIC_Start();
        IIC_Send_Byte(device_addr);//发送器件地址
        if( I2C_WaitAck() == 1 ) return 1;
        IIC_Send_Byte(gather_command);//发送采集命令
        if( I2C_WaitAck() == 1 ) return 2;
        IIC_Send_Byte(gather_command_parameter_1);//发送采集参数1
        if( I2C_WaitAck() == 1 ) return 3;
        IIC_Send_Byte(gather_command_parameter_2);//发送采集参数2
        if( I2C_WaitAck() == 1 ) return 4;
        IIC_Stop();
    return 0;
}


/******************************************************************
 * 函 数 名 称：aht21_device_init
 * 函 数 说 明：通过命令字节初始化AHT21
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void aht21_device_init(void)
{
    uint8_t device_addr = 0x70;//器件地址
    uint8_t init_command = 0xBE;//初始化命令
    uint8_t init_command_parameter_1 = 0x08;//采集参数1
    uint8_t init_command_parameter_2 = 0x00;//采集参数2

    IIC_Start();
        IIC_Send_Byte(device_addr);//发送器件地址
        if( I2C_WaitAck() == 1 ) printf("warning -5\r\n");
        IIC_Send_Byte(init_command);//发送初始化命令
        if( I2C_WaitAck() == 1 ) printf("warning -6\r\n");
        IIC_Send_Byte(init_command_parameter_1);//发送初始化参数1
        if( I2C_WaitAck() == 1 ) printf("warning -7\r\n");
        IIC_Send_Byte(init_command_parameter_2);//发送初始化参数2
        if( I2C_WaitAck() == 1 ) printf("warning -8\r\n");
        IIC_Stop();
}

/******************************************************************
 * 函 数 名 称：aht21_read_data
 * 函 数 说 明：读取温湿度
 * 函 数 形 参：无
* 函 数 返 回：1：未校准  2：读取超时  0：读取成功
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
char aht21_read_data(void)
{
    uint8_t data[6] = {0};
    uint32_t temp = 0;

    uint8_t aht21_status_byte = 0;
    uint8_t timeout = 0;
    //读取AHT21的状态
    aht21_status_byte = aht21_read_status();
    //如果未校准，则返回1
    if( (aht21_status_byte & (1<<3)) == 0 )
    {
        aht21_device_init();
        delay_1ms(50);
        return 1;
    }

    //发送采集命令
    aht21_send_gather_command();

    do
    {
        delay_1ms(1);
        timeout++;
        //读取AHT21的状态
        aht21_status_byte = aht21_read_status();
    }while( ( ( aht21_status_byte & (1<<7) ) != 0 ) && ( timeout >= 80 ) );
    //如果读取超时，则返回2
    if( timeout >= 80 ) return 2;

    IIC_Start();
    IIC_Send_Byte(0x71);
    if( I2C_WaitAck() == 1 ) printf("error -1\r\n");
        IIC_Read_Byte();//读取状态,不需要保存
        IIC_Send_Ack();

    //读取6位数据
    data[0] = IIC_Read_Byte();
    IIC_Send_Ack();
    data[1] = IIC_Read_Byte();
    IIC_Send_Ack();
    data[2] = IIC_Read_Byte();
    IIC_Send_Ack();
    data[3] = IIC_Read_Byte();
    IIC_Send_Ack();
    data[4] = IIC_Read_Byte();
    IIC_Send_Ack();
    data[5] = IIC_Read_Byte();
    IIC_Send_Nack();

    IIC_Stop();

    //整合湿度数据
    temp = (data[0]<<12) | (data[1]<<4);
    temp = temp | (data[2]>>4);
    //换算湿度数据
    //2的20次方 = 1048576
    humidity = temp / 1048576.0 * 100.0;


    //整合湿度数据
    temp = ( (data[2]&0x0f)<< 16 ) | (data[3]<<8);
    temp = temp | data[4];
    //换算湿度数据
    //2的20次方 = 1048576
    temperature = temp / 1048576.0 * 200.0 - 50;

    return 0;
}


/******************************************************************
 * 函 数 名 称：get_temperature
 * 函 数 说 明：返回读取过的温度
 * 函 数 形 参：无
 * 函 数 返 回：温度值
 * 作       者：LCKFB
 * 备       注：使用前，请确保调用 aht21_read_data 采集过数据
******************************************************************/
float get_temperature(void){
    return temperature;
}

/******************************************************************
 * 函 数 名 称：get_humidity
 * 函 数 说 明：返回读取过的湿度
 * 函 数 形 参：无
 * 函 数 返 回：湿度值
 * 作       者：LCKFB
 * 备       注：使用前，请确保调用 aht21_read_data 采集过数据
******************************************************************/
float get_humidity(void)
{
    return humidity;
}