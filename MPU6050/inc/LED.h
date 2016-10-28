/**
  ******************************************************************************
  * @file    LED.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    25-June-2015
  * @brief   Header for LED.c module.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported Definitions ------------------------------------------------------*/
#define BLE_LED					0x01
#define RED_LED					0x02

#define BLE_LED_GPIO_PORT		GPIOA
#define RED_LED_GPIO_PORT		GPIOD

#define BLE_LED_PIN				GPIO_Pin_8
#define RED_LED_PIN				GPIO_Pin_8

#define BLE_LED_GPIO_CLK		RCC_AHBPeriph_GPIOA
#define RED_LED_GPIO_CLK		RCC_AHBPeriph_GPIOD
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BLE_LED_On()			BLE_LED_GPIO_PORT->BRR = BLE_LED_PIN
#define RED_LED_On()			RED_LED_GPIO_PORT->BRR = RED_LED_PIN

#define BLE_LED_Off()			BLE_LED_GPIO_PORT->BSRR = BLE_LED_PIN
#define RED_LED_Off()			RED_LED_GPIO_PORT->BSRR = RED_LED_PIN

#define BLE_LED_Toggle()		BLE_LED_GPIO_PORT->ODR ^= BLE_LED_PIN
#define RED_LED_Toggle()		RED_LED_GPIO_PORT->ODR ^= RED_LED_PIN
/* Exported functions ------------------------------------------------------- */
void LED_Init(uint8_t LEDs);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */

/*************************************** END OF FILE ***************************************/
