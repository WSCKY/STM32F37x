/**
  ******************************************************************************
  * @file    TIMCounter.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    16-June-2015
  * @brief   Count time by TIM4&TIM5.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "TIMCounter.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t PrescalerValue = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure TIM4 & TIM5.
  * @param  None
  * @retval None
  */
void TIMConfig(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t)((SystemCoreClock) / 1000000) - 1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5, ENABLE);
	/* TIM5 Configuration*/
	/* Time Base Configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;//Auto reload
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Update);
	/* Disable the TIM5 Update event */
	TIM_UpdateDisableConfig(TIM5, ENABLE);
	/* ----------------------TIM5 Configuration as slave for the TIM4 ----------*/
	/* Select the TIM5 Input Trigger: TIM4 TRGO used as Input Trigger for TIM5*/
	TIM_SelectInputTrigger(TIM5, TIM_TS_ITR2);
	/* Use the External Clock as TIM5 Slave Mode */
	TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_External1);
	/* Enable the TIM5 Master Slave Mode */
	TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	/* TIM4 Configuration */
  	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM4, TIM_UpdateSource_Regular);
	/* ----------------------TIM4 Configuration as Master for the TIM5 -----------*/
  	/* Use the TIM4 Update event as TIM4 Trigger Output(TRGO) */
  	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);
  	/* Enable the TIM4 Master Slave Mode */
  	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	/* Start Timer */
	TIM_Cmd(TIM4, ENABLE);
  	TIM_Cmd(TIM5, ENABLE);
}

/**
  * @brief  Get microseconds.
  * @param  None
  * @retval micros
  */
uint32_t GetMicrosecond(void)
{
	uint32_t temp;
	temp = TIM5->CNT;
	temp <<= 16;
	temp += TIM4->CNT;
	return temp;
}

/*************************************** END OF FILE ***************************************/
