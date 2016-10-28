#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f37x.h"
#include "stm32f37x_conf.h"

#include "LED.h"
#include "UART_IT.h"
#include "PWM.h"
#include "Delay.h"
#include "MPU6050_IIC.h"
#include "TIMCounter.h"
#include "TIM2_IT.h"
#include "ADC.h"

#include "IMU.h"
#include "MyPID.h"

extern PID RollPID, PitchPID, YawPID;
extern float Throttle;
extern Axis A, G;
extern ANGLE Angle;
extern EulerPWM pidPWM;

#endif /* __MAIN_H */
