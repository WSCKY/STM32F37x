/**
  ******************************************************************************
  * @file    LED.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    25-June-2015
  * @brief   LED Driver File.
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "LED.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  LED Init.
  * @param  Specifies the Led to be configured.
  * @retval None
  */
void LED_Init(uint8_t LEDs)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	if(LEDs & 0x01)
	{
		RCC_AHBPeriphClockCmd(BLE_LED_GPIO_CLK, ENABLE);
		GPIO_Init(BLE_LED_GPIO_PORT, &GPIO_InitStructure);
	}
	if(LEDs & 0x02)
	{
		RCC_AHBPeriphClockCmd(RED_LED_GPIO_CLK, ENABLE);
		GPIO_Init(RED_LED_GPIO_PORT, &GPIO_InitStructure);
	}
}

/*************************************** END OF FILE ***************************************/
