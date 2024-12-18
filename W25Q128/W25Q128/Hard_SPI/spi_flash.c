/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-08-02     LCKFB-LP    first version
 */
#include "spi_flash.h"
#include "board.h"

/**********************************************************
 * 函 数 名 称：bsp_spi_init
 * 函 数 功 能：初始化SPI
 * 传 入 参 数：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void bsp_spi_init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能 GPIO 时钟 */
    RCC_AHB1PeriphClockCmd (BSP_GPIO_RCU, ENABLE);

    /* SPI时钟使能 */
    RCC_APB2PeriphClockCmd(BSP_SPI_RCU, ENABLE);

    /* 设置引脚复用 */
    GPIO_PinAFConfig(BSP_GPIO_PORT, BSP_SPI_SCK_PINSOURCE, BSP_GPIO_AF); 
    GPIO_PinAFConfig(BSP_GPIO_PORT, BSP_SPI_MISO_PINSOURCE, BSP_GPIO_AF); 
    GPIO_PinAFConfig(BSP_GPIO_PORT, BSP_SPI_MOSI_PINSOURCE, BSP_GPIO_AF); 

    /* 配置SPI引脚引脚SCK */
    GPIO_InitStructure.GPIO_Pin = BSP_SPI_SCK;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  

    GPIO_Init(BSP_GPIO_PORT, &GPIO_InitStructure);

    /* 配置SPI引脚MISO */
    GPIO_InitStructure.GPIO_Pin = BSP_SPI_MISO;
    GPIO_Init(BSP_GPIO_PORT, &GPIO_InitStructure);

    /* 配置SPI引脚MOSI */
    GPIO_InitStructure.GPIO_Pin = BSP_SPI_MOSI;
    GPIO_Init(BSP_GPIO_PORT, &GPIO_InitStructure);  

    /* 配置SPI引脚CS */
    GPIO_InitStructure.GPIO_Pin = BSP_SPI_NSS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(BSP_GPIO_PORT, &GPIO_InitStructure);

    /* CS引脚高电平 */
    W25QXX_CS_ON(1);

    /* FLASH_SPI 模式配置 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 传输模式全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                // 配置为主机
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;            // 8位数据
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                // 极性相位
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                   // 软件cs
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // SPI时钟预调因数为2
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;        //高位在前
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(BSP_SPI, &SPI_InitStructure);

    /* 使能 FLASH_SPI  */
    SPI_Cmd(BSP_SPI, ENABLE);
	
	W25QXX_CS_ON(1); 	// 片选拉高
}


uint8_t spi_read_write_byte(uint8_t dat)
{
    //等待发送缓冲区为空
    while(RESET == SPI_I2S_GetFlagStatus(BSP_SPI,  SPI_I2S_FLAG_TXE) );
    //通过SPI4发送一个字节数据 
    SPI_I2S_SendData(BSP_SPI, dat);
 
    //等待接收缓冲区不空标志
    while(RESET == SPI_I2S_GetFlagStatus(BSP_SPI,  SPI_I2S_FLAG_RXNE) );
    //读取并返回在SPI读取到的单字节数据
    return SPI_I2S_ReceiveData(BSP_SPI);
}

//读取芯片ID          
//读取设备ID
uint16_t W25Q128_readID(void)
{
    uint16_t  temp = 0;     
    //将CS端拉低为低电平     
    W25QXX_CS_ON(0);        
    //发送指令90h    
    spi_read_write_byte(0x90);//发送读取ID命令      
    //发送地址  000000H    
    spi_read_write_byte(0x00);             
    spi_read_write_byte(0x00);             
    spi_read_write_byte(0x00); 
        
    //接收数据
    //接收制造商ID
    temp |= spi_read_write_byte(0xFF)<<8;  
    //接收设备ID
    temp |= spi_read_write_byte(0xFF);        
    //恢复CS端为高电平
    W25QXX_CS_ON(1);      
    //返回ID                  
    return temp;
}

//发送写使能
void W25Q128_write_enable(void)   
{
    //拉低CS端为低电平
    W25QXX_CS_ON(0);                          
    //发送指令06h
    spi_read_write_byte(0x06);                  
    //拉高CS端为高电平
    W25QXX_CS_ON(1);
}

/**********************************************************
 * 函 数 名 称：W25Q128_wait_busy
 * 函 数 功 能：检测线路是否繁忙
 * 传 入 参 数：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void W25Q128_wait_busy(void)   
{   
    unsigned char byte = 0;
    do
     { 
        //拉低CS端为低电平
        W25QXX_CS_ON(0); 
        //发送指令05h                           
        spi_read_write_byte(0x05);                
        //接收状态寄存器值
        byte = spi_read_write_byte(0Xff);       
        //恢复CS端为高电平
        W25QXX_CS_ON(1);      
     //判断BUSY位是否为1 如果为1说明在忙，重新读写BUSY位直到为0   
     }while( ( byte & 0x01 ) == 1 );  
}

/**********************************************************
 * 函 数 名 称：W25Q128_erase_sector
 * 函 数 功 能：擦除一个扇区
 * 传 入 参 数：addr=擦除的扇区号
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：addr=擦除的扇区号，范围=0~4096。

W25Q128将16M的容量分为256个块（Block),每个块大小为64K（64000）个字节，每个块又分为16个扇区（Sector),每个扇区4K个字节。
W25Q128的最小擦除单位为一个扇区，也就是每次必须擦除4K个字节。
**********************************************************/
void W25Q128_erase_sector(uint32_t addr)   
{
        //计算扇区号，一个扇区4KB=4096
        addr *= 4096;
        W25Q128_write_enable();  //写使能   
        W25Q128_wait_busy();     //判断忙，如果忙则一直等待
        //拉低CS端为低电平
        W25QXX_CS_ON(0);  
        //发送指令20h                                     
        spi_read_write_byte(0x20);
        //发送24位扇区地址的高8位                
        spi_read_write_byte((uint8_t)((addr)>>16));      
        //发送24位扇区地址的中8位    
        spi_read_write_byte((uint8_t)((addr)>>8));   
        //发送24位扇区地址的低8位    
        spi_read_write_byte((uint8_t)addr);
        //恢复CS端为高电平  
        W25QXX_CS_ON(1);                  
        //等待擦除完成                                                  
        W25Q128_wait_busy();   
}

/**********************************************************
 * 函 数 名 称：W25Q128_write
 * 函 数 功 能：写数据到W25Q128进行保存
 * 传 入 参 数：buffer=写入的数据内容  addr=写入地址  numbyte=写入数据的长度
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void W25Q128_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte)
{    
    unsigned int i = 0;
    //擦除扇区数据
    W25Q128_erase_sector(addr/4096);
    //写使能 
    W25Q128_write_enable();  
    //忙检测  
    W25Q128_wait_busy();    
    //写入数据
    //拉低CS端为低电平
    W25QXX_CS_ON(0);         
    //发送指令02h                              
    spi_read_write_byte(0x02);                 
    //发送写入的24位地址中的高8位   
    spi_read_write_byte((uint8_t)((addr)>>16));  
    //发送写入的24位地址中的中8位
    spi_read_write_byte((uint8_t)((addr)>>8));   
    //发送写入的24位地址中的低8位
    spi_read_write_byte((uint8_t)addr);   
    //根据写入的字节长度连续写入数据buffer
    for(i=0;i<numbyte;i++)
    {
        spi_read_write_byte(buffer[i]);  
    }
    //恢复CS端为高电平
    W25QXX_CS_ON(1);
    //忙检测 
    W25Q128_wait_busy();      
}

/**********************************************************
 * 函 数 名 称：W25Q128_read
 * 函 数 功 能：读取W25Q128的数据
 * 传 入 参 数：buffer=读出数据的保存地址  read_addr=读取地址   read_length=读去长度
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void W25Q128_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length)   
{ 
        uint16_t i;                   
        //拉低CS端为低电平
        W25QXX_CS_ON(0);    
        //发送指令03h        
        spi_read_write_byte(0x03);  
        //发送24位读取数据地址的高8位                         
        spi_read_write_byte((uint8_t)((read_addr)>>16));     
        //发送24位读取数据地址的中8位      
        spi_read_write_byte((uint8_t)((read_addr)>>8));   
        //发送24位读取数据地址的低8位
        spi_read_write_byte((uint8_t)read_addr);   
        //根据读取长度读取出地址保存到buffer中
        for(i=0;i<read_length;i++)
        { 
            buffer[i]= spi_read_write_byte(0XFF);  
        }
        //恢复CS端为高电平
        W25QXX_CS_ON(1);                                    
}
