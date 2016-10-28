#include "main.h"

void Delay(__IO uint32_t nCount);

int main(void)
{
	/* LED GPIO PIn Configuration */
	GPIO_InitTypeDef  GPIO_InitStructure;

	SCB->VTOR = FLASH_BASE | 0x5200;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* USART1 Configuration */
	UART_Init(115200);
	
	GPIOB->BSRR = GPIO_Pin_14;//Red LED off
	GPIOB->BSRR = GPIO_Pin_15;//Blue LED off
	while(1)
	{
		GPIOB->ODR ^= GPIO_Pin_14;//Toggle LED
		Delay(0x1FFFFF);
		GPIOB->ODR ^= GPIO_Pin_15;
		Delay(0x1FFFFF);
		printf("Hello YUNEEC!\r\n");
//		printf("a");
	}
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
