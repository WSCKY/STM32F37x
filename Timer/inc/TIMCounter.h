/**
  ******************************************************************************
  * @file    TIMCounter.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    16-June-2015
  * @brief   Header for TIMCounter.c module.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMCOUNTER_H
#define __TIMCOUNTER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TIMConfig(void);
uint32_t GetMicrosecond(void);

#endif /* __TIMCOUNTER_H */

/*************************************** END OF FILE ***************************************/
