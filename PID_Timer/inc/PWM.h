/**
  ******************************************************************************
  * @file    PWM.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    16-June-2015
  * @brief   Header for PWM.c module.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM_H
#define __PWM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported Definitions ------------------------------------------------------*/
#define Default_CCR1							0
#define Default_CCR2							0
#define Default_CCR3							0
#define Default_CCR4							0
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void PWM_GeneraterInit(void);
void PWM_CCRSet(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3, uint16_t PWM4);
void PWM_DutyRateSet(uint16_t PWMRate1, uint16_t PWMRate2, uint16_t PWMRate3, uint16_t PWMRate4);

#endif /* __TIM1_PWM_H */

/*************************************** END OF FILE ***************************************/
