#include "uart1.h"

// 接收RX的缓冲区
uint8_t TxBuffer[128] = {1};

// GPIO 初始化函数
void UART1_GPIO_Init(void){
    // 使能 GPIOA 和 USART1 时钟
    RCC_APB2PeriphClockCmd(UART1_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        
    // 配置 PA9 为 UART1_TX (复用功能)
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART1_TX_Pin;
    GPIO_InitStructure.GPIO_Mode = UART1_TX_Mode;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = UART1_TX_Spend;
    GPIO_Init(UART1_TX_GPIO, &GPIO_InitStructure);
    
    // 配置 PA10 为 UART1_RX (复用功能)
    GPIO_InitStructure.GPIO_Pin = UART1_RX_Pin;
    GPIO_InitStructure.GPIO_Mode = UART1_RX_Mode;  // 浮空输入
    GPIO_Init(UART1_RX_GPIO, &GPIO_InitStructure);
}

// NVIC 配置
void NVIC_UART1_Config(void){
	
    NVIC_InitTypeDef NVIC_InitStructure;

    // 配置 USART1 的中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQ_CHANNEL;  // USART1 中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_IRQ_PREEMPTION_PRIORITY;  // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART1_IRQ_SUB_PRIORITY;  // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = USART1_IRQ_ENABLE;  // 使能中断
    NVIC_Init(&NVIC_InitStructure);
}

// DMA 配置
void UART1_DMA_Config(void){
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStructure;
	// DMA1 通道 4 UART1_TX
	DMA_DeInit(DMA1_Channel4);
	// 外设地址,这个地址是外设的数据寄存器的基地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_Addr_Base;
	// 内存基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer;
	// 传输方向
	DMA_InitStructure.DMA_DIR = UART1_DMA_DIR;
	// 发送缓冲区大小
	DMA_InitStructure.DMA_BufferSize = TxBufferSize1;
	// 外设地址不递增
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// 内存地址偏移
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	// 正常模式 或者循环模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	// 优先级
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	// 禁用内存到内存模式
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	// 初始化
	DMA_Init(USART1_Tx_DMA_Channel, &DMA_InitStructure);
	
	/*DMA使能*/
	//这里先不给使能，初始化后不会立刻工作，等后续调用Transfer后，再开始
	DMA_Cmd(USART1_Tx_DMA_Channel, DISABLE);	

}

// UART1 配置函数
void UART1_Config(void){    
	
	RCC_APB2PeriphClockCmd(UART1_CLK, ENABLE);

    // 配置 USART1 参数
	USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = UART1_BAUDRATE;  // 设置波特率为 115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8 数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 1 个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;  // 无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件流控制
    USART_InitStructure.USART_Mode = UART1_TransceiverMode;  // 使能发送和接收功能
	
    // 初始化 USART1
    USART_Init(USART_Handle, &USART_InitStructure);
	
	// 使能DMA请求
	USART_DMACmd(USART_Handle, USART_DMAReq_Tx, ENABLE);
	
	// 启用接收中断
    USART_ITConfig(USART_Handle, USART_IT_RXNE, ENABLE);  // 接收中断
	
    // 使能 USART1
    USART_Cmd(USART_Handle, ENABLE);				
}

// UART1 中断服务函数
void USART1_IRQHandler(void){
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // 从 UART1 接收寄存器读取数据
//        UART1_RXDate = USART_ReceiveData(USART1);
//		UART1_SendByte(UART1_RXDate);				
    }
	
	// 清除中断标志
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);   
}

// 重定向 printf 使用 UART1 输出
int fputc(int ch, FILE *f) {
    // 等待 UART1 发送缓冲区空
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    // 发送字符
    UART1_SendByte(ch);
    return ch;
}

void UART1_SendByte(uint8_t Byte){
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void UART1_SendArray(uint8_t *Array, uint16_t Length){
	uint16_t i;
	for (i = 0; i < Length; i ++){		//遍历数组
		UART1_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

void UART1_SendString(const char* str){
    while (*str){ // 当字符串未结束时
        UART1_SendByte(*str);  // 发送当前字符
        str++;  // 移动到下一个字符
    }
}

// UART1 Init
void UART1_Init(void){
	
	UART1_GPIO_Init();
	NVIC_UART1_Config();
	UART1_DMA_Config();
	UART1_Config();
	
}
