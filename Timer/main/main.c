#include "main.h"

uint32_t Current, LastUpdate, Time;
uint8_t Index = 0;

int main(void)
{
	/* LED GPIO PIn Configuration */
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOD, ENABLE);
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	Delay_Init(72);
	/* USART1 Configuration */
	UART_Init(115200);
	TIMConfig();
	GPIOA->BSRR = GPIO_Pin_8;//Blue LED off
	GPIOD->BSRR = GPIO_Pin_8;//Red LED off
	while(1)
	{
		GPIOA->BRR = GPIO_Pin_8;//Blue LED on
		LastUpdate = GetMicrosecond();
		USART_SendData(USART1, 'A');
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//		USART_SendData(USART1, 'B');
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//		USART_SendData(USART1, 'C');
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//		USART_SendData(USART1, 'D');
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//		USART_SendData(USART1, 'E');
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//		Delay_ms(637);
		Current = GetMicrosecond();
		GPIOA->BSRR = GPIO_Pin_8;//Blue LED off
		if(Current >= LastUpdate)
			Time = Current - LastUpdate;
		else
			Time = Current + (0xFFFFFFFF - LastUpdate);
		printf("\r\nMicros : %dus \r\n", Time - 1);
		GPIOD->BRR = GPIO_Pin_8;//Red LED on
		Delay_ms(100);
		GPIOD->BSRR = GPIO_Pin_8;//Red LED off
		Delay_ms(500);
		Index ++;
		printf("Count : %d\r\n", Index);
	}
}
