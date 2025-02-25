#include "stm32f10x.h" 
#include "systick.h"
#include "main.h"
#include "uart1.h"
#include "Timer1.h"

// UART1 RX 接收的内容
extern uint8_t UART1_RXDate;

int main(){
	
	systick_config();
	// 优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	UART1_Init();
	Timer1_Init();
	
	while(1){
		
//		printf("%d\n", SystemCoreClock);
//		delay_1ms(1000);
		
	}
	
}
