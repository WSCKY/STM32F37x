/**
  ******************************************************************************
  * @file    ADC.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    19-June-2015
  * @brief   The header file for ADC.c.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

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
/* Exported variables --------------------------------------------------------*/
extern __IO uint16_t ADC1ConvertedValue;
/* Exported functions ------------------------------------------------------- */
void ADC1_CH4_DMA_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */

/*************************************** END OF FILE ***************************************/
