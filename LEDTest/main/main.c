#include "main.h"
#include <iostream>

void Delay(__IO uint32_t nCount);

int main(void)
{
	SCB->VTOR = FLASH_BASE | 0x5200;
	LED_Init(BLE_LED | RED_LED);
	BLE_LED_Off();
	RED_LED_Off();
	while(1)
	{
		BLE_LED_Toggle();
		Delay(0x1FFFFF);
//		BLE_LED_Toggle();
//		Delay(0x1FFFFF);
		RED_LED_Toggle();
		Delay(0x1FFFFF);
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
