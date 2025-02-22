#include "adc1.h"


void ADC1_GPIO_Config(void) {
    // 启用GPIOA时钟
    RCC_APB2PeriphClockCmd(ADC_GPIO_CLK, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    // 配置PA0为模拟输入模式（ADC1_IN0）
    GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = ADC_GPIO_MODE;    // 模拟输入模式
    GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);
}

void ADC1_Config(void) {
    // 启用ADC1时钟
    RCC_APB2PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);
    
    // 配置ADC时钟分频（PCLK2为72MHz时，分频6得到12MHz）
    RCC_ADCCLKConfig(ADC_CLK_DIV);
    
    ADC_InitTypeDef ADC_InitStructure;
    // ADC基础配置
    ADC_InitStructure.ADC_Mode = ADC_MODE;        // 独立模式
    ADC_InitStructure.ADC_ScanConvMode = ADC_SCAN_MODE;             // 单通道，禁用扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ADC_CONTINUOUS_MODE;       // 单次转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_TRIGGER; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DATA_ALIGN;    // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = ADC_NBR_OF_CHANNELS;                   // 转换通道数为1
    ADC_Init(ADCx, &ADC_InitStructure);
    
    // 配置通道0（IN0）的采样时间（239.5周期，适合高阻抗信号）
    ADC_RegularChannelConfig(ADCx, ADC_CHANNEL, 1, ADC_SAMPLETIME);
    
    // 启用ADC
    ADC_Cmd(ADCx, ENABLE);
    
    // ADC校准流程
    ADC_ResetCalibration(ADCx);               // 复位校准寄存器
    while(ADC_GetResetCalibrationStatus(ADCx)); // 等待复位完成
    ADC_StartCalibration(ADCx);               // 开始校准
    while(ADC_GetCalibrationStatus(ADCx));    // 等待校准完成
}


// 读取ADC值的函数, 这里获取到的是对应引脚上电压的模拟值
/*

假设 V_ref = 3.3V，那么：

如果 ADC 的输出值为 0，则输入电压为 0V。
如果 ADC 的输出值为 2048，则输入电压大约为 3.3V / 2 ≈ 1.65V。
如果 ADC 的输出值为 4095，则输入电压为 3.3V。

*/
uint16_t ADC_GetValue(void) {
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);      // 启动转换
    while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC)); // 等待转换完成
    return ADC_GetConversionValue(ADCx);         // 返回转换结果
}

void ADC1_Init(void){
	ADC1_GPIO_Config();
	ADC1_Config();	
}
