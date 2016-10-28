/**
  ******************************************************************************
  * @file    Delay.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    3-February-2015
  * @brief   Delay Function.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t Ms, Us;
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Delay Init.
  * @param  Load value.
  * @retval None
  */
void Delay_Init(uint32_t Value)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	SysTick->CTRL &= 0xFFFFFFFB;
	Us = Value / 8;
	Ms = Us * 1000;
}

/**
  * @brief  This function provides accurate delay (in milliseconds).
  * @param  Delay: specifies the delay time length, in milliseconds.
  * @note     This value should < 798 (24bit, 0xFFFFFF / ((168/8)*1000) = 798).
  * @retval None
  */
void Delay_ms(uint32_t Delay)
{
	uint32_t temp;
	SysTick->LOAD = Delay * Ms;//SysTick->LOAD is 24bit
	SysTick->VAL = 0x00;//clear counter.
	SysTick->CTRL = 0x01;//start count.
	do
	{
		temp = SysTick->CTRL;
	}
	while((temp & 0x01)&& !(temp & (1 << 16)));//wait
	SysTick->CTRL = 0x00;//close counter.
	SysTick->VAL = 0X00;//clear counter.
}

/**
  * @brief  This function provides accurate delay (in milliseconds, long time).
  * @param  Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay_lms(uint32_t Delay)
{
	uint16_t Repeat, Remain;
	Repeat = Delay / 500;
	Remain = Delay % 500;
	while(Repeat)
	{
		Delay_ms(500);
		Repeat --;
	}
	if(Remain)
		Delay_ms(Remain);
}

/**
  * @brief  This function provides accurate delay (in microseconds).
  * @param  Delay: specifies the delay time length, in microseconds.
  * @retval None
  */
void Delay_us(uint32_t Delay)
{
	uint32_t temp;
	SysTick->LOAD = Delay * Us;//SysTick->LOAD is 24bit
	SysTick->VAL = 0x00;//clear counter.
	SysTick->CTRL = 0x01;//start count.
	do
	{
		temp = SysTick->CTRL;
	}
	while((temp & 0x01)&& !(temp & (1 << 16)));//wait
	SysTick->CTRL = 0x00;//close counter.
	SysTick->VAL = 0X00;//clear counter.
}

/*************************************** END OF FILE ***************************************/
