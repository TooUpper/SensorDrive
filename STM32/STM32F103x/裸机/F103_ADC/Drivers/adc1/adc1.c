#include "adc1.h"


void ADC1_GPIO_Config(void) {
    // 启用GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    // 配置PA0为模拟输入模式（ADC1_IN0）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;    // 模拟输入模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void ADC1_Config(void) {
    // 启用ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    // 配置ADC时钟分频（PCLK2为72MHz时，分频6得到12MHz）
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    ADC_InitTypeDef ADC_InitStructure;
    // ADC基础配置
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;        // 独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;             // 单通道，禁用扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;       // 单次转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;    // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                   // 转换通道数为1
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // 配置通道0（IN0）的采样时间（239.5周期，适合高阻抗信号）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    
    // 启用ADC
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC校准流程
    ADC_ResetCalibration(ADC1);               // 复位校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1)); // 等待复位完成
    ADC_StartCalibration(ADC1);               // 开始校准
    while(ADC_GetCalibrationStatus(ADC1));    // 等待校准完成
}


// 读取ADC值的函数, 这里获取到的是对应引脚上电压的模拟值
/*

假设 V_ref = 3.3V，那么：

如果 ADC 的输出值为 0，则输入电压为 0V。
如果 ADC 的输出值为 2048，则输入电压大约为 3.3V / 2 ≈ 1.65V。
如果 ADC 的输出值为 4095，则输入电压为 3.3V。

*/
uint16_t ADC_GetValue(void) {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);      // 启动转换
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // 等待转换完成
    return ADC_GetConversionValue(ADC1);         // 返回转换结果
}

void ADC1_Init(void){
	ADC1_GPIO_Config();
	ADC1_Config();	
}
