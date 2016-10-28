/**
  ******************************************************************************
  * @file    Delay.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    3-February-2015
  * @brief   The header file for Delay.c.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"
	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported Definitions ------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Delay_Init(uint32_t Value);
void Delay_ms(uint32_t Delay);
	 void Delay_lms(uint32_t Delay);
void Delay_us(uint32_t Delay);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */

/*************************************** END OF FILE ***************************************/
