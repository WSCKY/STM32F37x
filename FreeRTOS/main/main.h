/**
  ******************************************************************************
  * @file    main.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    27-June-2015
  * @brief   Header for main.c module.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"

#include "LED.h"
#include "UART.h"

#include "cmsis_os.h"
#include "cpu_utils.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "StackMacros.h"
#include "queue.h"
#include "FreeRTOSConfig.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/******************************** END OF FILE *********************************/
