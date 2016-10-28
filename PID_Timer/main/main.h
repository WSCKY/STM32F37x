#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f37x.h"
#include "stm32f37x_conf.h"

#include "UART_IT.h"
#include "PWM.h"
#include "Delay.h"
#include "MPU6050_IIC.h"
#include "TIMCounter.h"
#include "TIM2_IT.h"
#include "ADC.h"
#include "LED.h"
#include "control.h"
#include "IMU.h"
#include "MyPID.h"

#include "DataMonitor.h"
#include "DataTransfer.h"

extern ANGLE ExpAngle;
extern float Throttle;
extern FlagStatus InitStatus;

#endif /* __MAIN_H */
