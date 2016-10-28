#include "main.h"

uint32_t TimeCnt = 0;

uint8_t KeyFlag = 0, KeyPressed = 0, KeyPreCnt = 0, SteerRunning = 0, RunStep = 0;

//Proj -1
int main(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	Delay_Init(72);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOD, ENABLE);
//	Configure the GPIO_LED pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	PWM_GeneraterInit();
//	PWM_DutyRateSet(0, 10, 0, 0);
	PWM_CCRSet(0,0,0,1000);
	while(1)
	{
//		PWM_CCRSet(0,0,0,1000);//最终
//		GPIOB->ODR ^= GPIO_Pin_14;
//		Delay_ms(1000);
//		PWM_CCRSet(0,0,0,2000);
//		GPIOB->ODR ^= GPIO_Pin_15;
//		Delay_ms(1000);

		TimeCnt ++;
		if(TimeCnt % 5 == 1)
		{
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == RESET)
			{
				if(KeyPreCnt < 10)
					KeyPreCnt ++;
				else
					KeyFlag = 1;
			}
			else
			{
				KeyPreCnt = 0;
				if(KeyFlag == 1)
					KeyPressed = 1;
				KeyFlag = 0;
			}
		}
		if(KeyPressed == 1)
		{
			KeyPressed = 0;
			SteerRunning = 1;
			RunStep = 0;
			GPIOB->BSRR = GPIO_Pin_14;
			GPIOB->BSRR = GPIO_Pin_15;
		}
		if(SteerRunning == 1)//启动舵机运行程序
		{
			if(TimeCnt % 800 == 0)
			{
				RunStep ++;
				switch(RunStep)
				{
//					case 1:
//						PWM_CCRSet(0,0,0,1000);//最终
//					break;
					case 1:
						PWM_CCRSet(0,0,0,2000);
						GPIOB->BRR = GPIO_Pin_15;
					break;
					case 2:
						PWM_CCRSet(0,0,0,1000);//最终
						GPIOB->BRR = GPIO_Pin_14;
						SteerRunning = 0;
						RunStep = 0;
					break;
					default :
						RunStep = 0;
					break;
				}
			}
		}
		if(TimeCnt >= 60000) TimeCnt = 0;
		Delay_ms(1);
	}
}

//Proj -2
/*int main(void)
{
	int8_t Dir = 1;
	uint32_t pwm = 1000;
	uint32_t DelayTime = 0;
	GPIO_InitTypeDef  GPIO_InitStructure;

	Delay_Init(72);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOD, ENABLE);
//	Configure the GPIO_LED pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	PWM_GeneraterInit();
	PWM_CCRSet(0,0,0,1000);

	while(1)//loop time :1s
	{
		if(DelayTime > 0)
			DelayTime --;
		else
		{
			pwm += Dir;
			PWM_CCRSet(0, 0, 0, pwm);
			if(pwm >= 2000)
			{
				Dir = -1;
				DelayTime = 500;
			}
			else if(pwm <= 1000)
			{
				Dir = 1;
				DelayTime = 500;
			}
		}
		Delay_ms(1);
	}
}
*/
