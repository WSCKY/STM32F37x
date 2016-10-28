#include "main.h"

int main(void)
{
	/* LED GPIO PIn Configuration */
	GPIO_InitTypeDef  GPIO_InitStructure;
	Delay_Init(72);
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
	
	/* USART1 Configuration */
	UART_Init(115200);
	
	GPIOA->BSRR = GPIO_Pin_8;//Blue LED off
	GPIOD->BSRR = GPIO_Pin_8;//Red LED off
	ADC1_CH4_DMA_Config();
	ADC_SoftwareStartConv(ADC1);
	while(1)
	{
		GPIOA->ODR ^= GPIO_Pin_8;//Toggle LED
		Delay_ms(500);
		GPIOD->ODR ^= GPIO_Pin_8;
		Delay_ms(500);
		printf("Vlotage = %f \r\n", (double)ADC1ConvertedValue * 3/0xFFF);
	}
}
