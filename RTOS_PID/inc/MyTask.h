/**
  ******************************************************************************
  * @file    MyTask.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    27-June-2015
  * @brief   Header for MyTask.c module.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYTASK_H
#define __MYTASK_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* System */
#include "stm32f37x.h"
#include "stm32f37x_conf.h"
/* RealTime Operation System */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "StackMacros.h"
#include "FreeRTOSConfig.h"
/* Hardware */
#include "LED.h"
#include "ADC.h"
#include "PWM.h"
#include "UART_IT.h"
#include "TIMCounter.h"
//#include "MPU6050.h"
#include "MPU6050_IIC.h"
/* Algorithm */
#include "IMU.h"
#include "MyPID.h"
/* Data Upload */
#include "DataMonitor.h"
#include "DataTransfer.h"
/* control */
#include "control.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern ANGLE ExpAngle;
extern float Throttle;
extern FlagStatus InitStatus;
/* Exported functions ------------------------------------------------------- */
void StartTask(void const * argument);

#ifdef __cplusplus
}
#endif

#endif /* MYTASK_H */

/******************************** END OF FILE *********************************/
