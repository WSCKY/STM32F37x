/**
  ******************************************************************************
  * @file    IMU.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    4-February-2015
  * @brief   Header for IMU.c module.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMU_H
#define __IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"
#include "TIMCounter.h"
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}ANGLE;

typedef struct
{
	float X;
	float Y;
	float Z;
}Axis;

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
}Quaternions;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void IMU_Init(void);
void IMUAccGyr(Axis *Acc, Axis *Gyr, ANGLE *Angle);
void IMUUpdate(Axis *Acc, Axis *Gyr, Axis *Mag, ANGLE *Angle);

#endif /* __IMU_H */
