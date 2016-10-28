/**
  ******************************************************************************
  * @file    PWM.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    16-June-2015
  * @brief   Use TIM19 Module to Generate PWM.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "PWM.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void TIM_PinConfig(void);

/**
  * @brief  TIM1 Pin configuration
  * @param  None
  * @retval None
  */
static void TIM_PinConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM19 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM19, ENABLE);

	/* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* GPIOA Configuration: TIM19 CH1 (PA0), TIM19 CH2 (PA1), TIM19 CH3 (PA2) and TIM19 CH4 (PA3) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM19 pins to AF2 */  
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_11);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_11);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_11);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_11);
}

/**
  * @brief  PWM generater init.
  * @param  None
  * @retval None
  */
void PWM_GeneraterInit(void)
{
/* -----------------------------------------------------------------------
	1/ TIM19 Configuration: generate 4 PWM signal with 1 duty cycle.
	
	In this example TIM19 input clock (TIM19CLK) is set to APB2 clock (PCLK2), 
	since APB2 prescaler is different from 1.   
	TIM19CLK = PCLK2
	PCLK2 = HCLK
	=> TIM19CLK = HCLK = SystemCoreClock (72MHz)

	To get TIM19 counter clock at 1 MHz, the prescaler is computed as follows:
	Prescaler = (TIM19CLK / TIM1 counter clock) - 1
	Prescaler = ((SystemCoreClock) / 1 MHz) - 1 = 71

	To get TIM1 output clock at 50Hz, the period (ARR)) is computed as follows:
	ARR = (TIM1 counter clock / TIM1 output clock) - 1
		= 19999

	TIM1 Channel1 duty cycle = (CCR1_Val/ TIM1_ARR)* 100%
	TIM1 Channel2 duty cycle = (CCR2_Val/ TIM1_ARR)* 100%
	TIM1 Channel3 duty cycle = (CCR3_Val/ TIM1_ARR)* 100%
	TIM1 Channel4 duty cycle = (CCR4_Val/ TIM1_ARR)* 100%

	Note: 
	SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f37x.c file.
	Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	function to update SystemCoreClock variable value. Otherwise, any configuration
	based on this variable will be incorrect.    
----------------------------------------------------------------------- */   
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;

	TIM_PinConfig();

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock) / 1000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 19999;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM19, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = Default_CCR1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OC1Init(TIM19, &TIM_OCInitStructure);

//	/* PWM1 Mode configuration: Channel2 */
//	TIM_OCInitStructure.TIM_Pulse = Default_CCR2;
//	TIM_OC2Init(TIM19, &TIM_OCInitStructure);

//	/* PWM1 Mode configuration: Channel3 */
//	TIM_OCInitStructure.TIM_Pulse = Default_CCR3;
//	TIM_OC3Init(TIM19, &TIM_OCInitStructure);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_Pulse = Default_CCR4;
	TIM_OC4Init(TIM19, &TIM_OCInitStructure);

	TIM_ARRPreloadConfig(TIM19, ENABLE);

	/* TIM19 enable counter */
	TIM_Cmd(TIM19, ENABLE);
}

/**
  * @brief  Set CCR Value.
  * @param  PWM1, PWM2, PWM3, PWM4.
  *         @note :this value should be 0 ~ 20000.
  * @retval None
  */
void PWM_CCRSet(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3, uint16_t PWM4)
{
//	TIM_SetCompare1(TIM19, PWM1);
//	TIM_SetCompare2(TIM19, PWM2);
//	TIM_SetCompare3(TIM19, PWM3);
	TIM_SetCompare4(TIM19, PWM4);
}

/**
  * @brief  Set PWM duty rate.
  * @param  PWMRate1, PWMRate2, PWMRate3, PWMRate4.
  *         @note :this value should be 0 ~ 100.
  * @retval None
  */
void PWM_DutyRateSet(uint16_t PWMRate1, uint16_t PWMRate2, uint16_t PWMRate3, uint16_t PWMRate4)
{
	TIM_SetCompare1(TIM19, PWMRate1 * 8);
	TIM_SetCompare2(TIM19, PWMRate2 * 8);
	TIM_SetCompare3(TIM19, PWMRate3 * 8);
	TIM_SetCompare4(TIM19, PWMRate4 * 8);
}

/*************************************** END OF FILE ***************************************/
