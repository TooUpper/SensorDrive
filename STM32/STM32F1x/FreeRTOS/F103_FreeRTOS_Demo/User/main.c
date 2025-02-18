#include "stm32f10x.h"                  // Device header
#include "systick.h"
#include "stdio.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

int a1 = 0;
int a2 = 0;

void task1(void *p){
	while(1){
		a1 = 1;
		a2 = 0;
	}
}

void task2(void *p){
	while(1){
		a1 = 0;
		a2 = 1;
	}
}


int main(){
	
	xTaskCreate(task1, "task1", 10, "Task 1 is Running", 1, NULL);
	xTaskCreate(task2, "task2", 10, "Task 1 is Running", 1, NULL);
	
	vTaskStartScheduler();
	
	while(1);
} 
