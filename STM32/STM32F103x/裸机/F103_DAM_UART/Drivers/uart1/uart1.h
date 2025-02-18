#ifndef __UART1_H__
#define __UART1_H__

#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"

/************* GPIO ********************/
#define UART1_GPIO_CLK 	RCC_APB2Periph_GPIOA
// PA9
#define UART1_TX_GPIO   GPIOA
#define UART1_TX_Pin    GPIO_Pin_9
#define UART1_TX_Mode   GPIO_Mode_AF_PP
#define UART1_TX_Spend  GPIO_Speed_50MHz

// PA10
#define UART1_RX_GPIO  GPIOA
#define UART1_RX_Pin   GPIO_Pin_10
#define UART1_RX_Mode  GPIO_Mode_IN_FLOATING

/************ DMA *****************/
#define USART1_Addr_Base (uint32_t)&USART1->DR	// UART1 数据寄存器地址
#define UART1_DMA_DIR  DMA_DIR_PeripheralDST	// 发送方向
#define TxBufferSize1 128						// 发送缓冲区大小
#define USART1_Tx_DMA_Channel DMA1_Channel4		// DMA1通道4



/***************** UART1 *****************/
#define UART1_CLK 			   RCC_APB2Periph_USART1
#define UART1_BAUDRATE 		   115200
#define UART1_TransceiverMode (USART_Mode_Tx | USART_Mode_Rx)
#define USART_Handle 		   USART1


/******************* NVIC ***********************/
#define USART1_IRQ_CHANNEL               USART1_IRQn	// USART1 中断
#define USART1_IRQ_PREEMPTION_PRIORITY   1              // USART1 抢占优先级
#define USART1_IRQ_SUB_PRIORITY          1              // USART1 子优先级
#define USART1_IRQ_ENABLE                ENABLE         // 使能 USART1 中断

/**/
void UART1_Init(void);

void UART1_SendByte(uint8_t Byte);

void UART1_SendArray(uint8_t *Array, uint16_t Length);

void UART1_SendString(const char* str);

#endif
