#include "stm32f10x.h" 
#include "systick.h"
#include "main.h"
#include "uart1.h"
#include "adc1.h"

// UART1 RX 接收的内容
extern uint8_t UART1_RXDate;

int main(){
	
	systick_config();
	// 优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	UART1_Init();
	ADC1_Init();
	
	while(1){
		
		uint16_t ADC1Value = ADC_GetValue();
		printf("%d", ADC1Value);
		delay_1ms(1000);
		
	}
	
}
