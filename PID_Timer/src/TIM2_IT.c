/**
  ******************************************************************************
  * @file    TIM2_IT.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    18-June-2015
  * @brief   TIM2 Configuration.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "TIM2_IT.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void TIM2_ITConfig(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */
static void TIM2_ITConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Initialize TIM2 as IT mode.
  * @param  None
  * @retval None
  */
void TIM2_ITInit(void)
{
/* -----------------------------------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:

    In this example TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM2CLK = PCLK1  
      PCLK1 = HCLK 
      => TIM2CLK = HCLK = SystemCoreClock

    To get TIM2 counter clock at 100 KHz, the prescaler is computed as follows:
       Prescaler = (TIM2CLK / TIM2 counter clock) - 1
       Prescaler = (SystemCoreClock /100 KHz) - 1

	To get TIM2 output clock at 1K Hz, the period (ARR)) is computed as follows:
		ARR = (TIM2 counter clock / TIM2 output clock) - 1
			= 99
----------------------------------------------------------------------------------------- */

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t PrescalerValue = 0;
	
	TIM2_ITConfig();

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 100000) - 1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 99;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM2, ENABLE);
}

/*************************************** END OF FILE **************************************/
