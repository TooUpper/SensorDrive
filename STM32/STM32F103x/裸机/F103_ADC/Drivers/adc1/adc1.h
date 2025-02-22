#ifndef __ADC1_H__
#define __ADC1_H__

#include "stm32f10x_gpio.h"

/**/
// GPIO
#define ADC_GPIO_PORT  GPIOA           // 选择GPIO端口A
#define ADC_GPIO_PIN   GPIO_Pin_0       // 选择引脚PA0（ADC1_IN0）
#define ADC_GPIO_CLK   RCC_APB2Periph_GPIOA  // GPIOA时钟使能
#define ADC_GPIO_MODE  GPIO_Mode_AIN    // 模拟输入模式

// ADC1
#define ADCx 				ADC1
#define ADC_PERIPH_CLK      RCC_APB2Periph_ADC1      // ADC1时钟使能
#define ADC_CLK_DIV         RCC_PCLK2_Div6           // ADC时钟分频
#define ADC_MODE            ADC_Mode_Independent     // 独立模式
#define ADC_SCAN_MODE       DISABLE                  // 单通道模式
#define ADC_CONTINUOUS_MODE DISABLE                  // 单次转换模式
#define ADC_TRIGGER         ADC_ExternalTrigConv_None// 软件触发
#define ADC_DATA_ALIGN      ADC_DataAlign_Right      // 右对齐
#define ADC_CHANNEL         ADC_Channel_0            // ADC1_CHANNEL_0（对应PA0）
#define ADC_SAMPLETIME      ADC_SampleTime_239Cycles5 // 239.5周期适合高阻抗信号
#define ADC_NBR_OF_CHANNELS 1                        // 转换通道数为1

/**/
void ADC1_Init(void);

uint16_t ADC_GetValue(void);

#endif

