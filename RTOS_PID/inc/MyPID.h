#ifndef __MYPID_H
#define __MYPID_H

#include "stm32f37x.h"
#include "stm32f37x_conf.h"
#include "TIMCounter.h"

typedef struct
{
	float pGain,
		  iGain,
	      dGain;
	float pTerm,
		  iTerm,
	      dTerm;
	float iMax,
		  iMin;
	float iState;
	float PreErr;
	uint32_t Now, Last;
}PID;

typedef struct
{
	float EulerRoll;
	float EulerPitch;
	float EulerYaw;
}EulerPWM;

typedef struct
{
	uint16_t PWM1;
	uint16_t PWM2;
	uint16_t PWM3;
	uint16_t PWM4;
}MotorPWM;

#define DEFAULT_PID_INTEGRATION_LIMIT	1000.0
#define PID_ROLL_INTEGRATION_LIMIT		1000.0
#define PID_PITCH_INTEGRATION_LIMIT		1000.0
#define PID_YAW_INTEGRATION_LIMIT		1000.0

#define PID_ROLL_GYR_INTEG_LIMIT		1000.0
#define PID_PITCH_GYR_INTEG_LIMIT		1000.0
#define PID_YAW_GYR_INTEG_LIMIT			1000.0

#define MOTOR_MIN_LIMIT					0
#define MOTOR_MAX_LIMIT					1000

#define ABS(x) 							((x > 0) ? x : (-x))

void PID_Init(PID *pPID, float Kp, float Ki, float Kd);
void PIDSetIntegralLimit(PID *pPID, float Limit);
void PIDSetGain(PID *pPID, float pGain, float iGain, float dGain);
float PIDUpdateT(PID *pPID, float Error, float Update_T);
float PIDUpdate(PID *pPID, float Error, float Gyro);
float GyrPIDUpdate(PID *pPID, float Error);

void MotorPWMCompute(MotorPWM *pMotor, float Thro, EulerPWM PIDGyrPWM);

#endif /* __MYPID_H */

/*************************************** END OF FILE ***************************************/
