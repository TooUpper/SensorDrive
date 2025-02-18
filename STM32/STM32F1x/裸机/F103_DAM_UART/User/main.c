#include "stm32f10x.h" 
#include "systick.h"
#include "main.h"
#include "uart1.h"

// UART1 RX 接收的内容
extern uint8_t TxBuffer[128];

int main() {
    systick_config();
    // 优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 初始化 UART 和 DMA
    UART1_Init();
    

    // 填充 TxBuffer 数据
    for (int i = 0; i < 24; i++) {
        TxBuffer[i] = 'A' + (i % 26);  // 填充字母 A-Z
    }

    printf("Run..\n");

    // 循环运行
    while(1) {
		// 1. 禁用DMA通道（确保可以重新配置）
		DMA_Cmd(DMA1_Channel4, DISABLE);

		// 2. 设置本次传输的数据长度（24字节）
		DMA_SetCurrDataCounter(DMA1_Channel4, 128);

		// 3. 清除DMA传输完成标志
		DMA_ClearFlag(DMA1_FLAG_TC4);

		// 4. 启动DMA传输
		DMA_Cmd(DMA1_Channel4, ENABLE);

//		printf("Run2...\n");

		// 5. 等待DMA传输完成
		while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);
		printf("\n");
		printf("Run3...\n");

		// 6. 延迟1秒
		delay_1ms(1000);
	}  
}

