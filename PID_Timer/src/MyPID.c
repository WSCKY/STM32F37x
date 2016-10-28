#include "MyPID.h"

/**
  * @brief  
  * @param  
  * @retval 
  */
void PID_Init(PID *pPID, float Kp, float Ki, float Kd)
{
	pPID->pGain = Kp;
	pPID->iGain = Ki;
	pPID->dGain = Kd;
	pPID->iMax = DEFAULT_PID_INTEGRATION_LIMIT;
	pPID->iMin = -DEFAULT_PID_INTEGRATION_LIMIT;
	pPID->iState = 0;
	pPID->Now = GetMicrosecond();
	pPID->Last = GetMicrosecond();
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void PIDSetIntegralLimit(PID *pPID, float Limit)
{
	pPID->iMax = ABS(Limit);
	pPID->iMin = -ABS(Limit);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void PIDSetGain(PID *pPID, float pGain, float iGain, float dGain)
{
	pPID->pGain = pGain;
	pPID->iGain = iGain;
	pPID->dGain = dGain;
}

/**
  * @brief  ...
  * @param  ...
  * @retval None
  */
float PIDUpdateT(PID *pPID, float Error, float Update_T)
{
	pPID->pTerm = pPID->pGain * Error;
	pPID->iState += Error * Update_T;
	if(pPID->iState > pPID->iMax)
		pPID->iState = pPID->iMax;
	if(pPID->iState < pPID->iMin)
		pPID->iState = pPID->iMin;
	pPID->iTerm = pPID->iGain * pPID->iState;
	pPID->dTerm = pPID->dGain * ((Error - pPID->PreErr) / Update_T);
	pPID->PreErr = Error;
	return (pPID->pTerm + pPID->iTerm + pPID->dTerm);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
float PIDUpdate(PID *pPID, float Error, float Gyro)
{
	float Update_T;

	pPID->Now = GetMicrosecond();//Update Time.
	if(pPID->Now >= pPID->Last)
		Update_T = ((float)(pPID->Now - pPID->Last) / 1000000.0f);
	else
		Update_T = ((float)(pPID->Now + (0xFFFFFFFF - pPID->Last)) / 1000000.0f);
	pPID->Last = pPID->Now;

	pPID->pTerm = pPID->pGain * Error;
	pPID->iState += Error * Update_T;
	if(pPID->iState > pPID->iMax)
		pPID->iState = pPID->iMax;
	if(pPID->iState < pPID->iMin)
		pPID->iState = pPID->iMin;
	pPID->iTerm = pPID->iGain * pPID->iState;
	pPID->dTerm = -pPID->dGain * Gyro;

	return (pPID->pTerm + pPID->iTerm + pPID->dTerm);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
float GyrPIDUpdate(PID *pPID, float Error)
{
	float Update_T;

	pPID->Now = GetMicrosecond();//Update Time.
	if(pPID->Now >= pPID->Last)
		Update_T = ((float)(pPID->Now - pPID->Last) / 1000000.0f);
	else
		Update_T = ((float)(pPID->Now + (0xFFFFFFFF - pPID->Last)) / 1000000.0f);
	pPID->Last = pPID->Now;

	pPID->pTerm = pPID->pGain * Error;
	pPID->iState += Error * Update_T;
	if(pPID->iState > pPID->iMax)
		pPID->iState = pPID->iMax;
	if(pPID->iState < pPID->iMin)
		pPID->iState = pPID->iMin;
	pPID->iTerm = pPID->iGain * pPID->iState;
	pPID->dTerm = pPID->dGain * ((Error - pPID->PreErr) / Update_T);
	pPID->PreErr = Error;
	return (pPID->pTerm + pPID->iTerm + pPID->dTerm);
}

/* ----------------------------------------- Motor --------------------------------------- */

void MotorPWMCompute(MotorPWM *pMotor, float Thro, EulerPWM PIDGyrPWM)
{
	float PWMValue;
	PWMValue = Thro - PIDGyrPWM.EulerPitch + PIDGyrPWM.EulerRoll - PIDGyrPWM.EulerYaw;
	if(PWMValue > MOTOR_MAX_LIMIT) PWMValue = MOTOR_MAX_LIMIT;
	if(PWMValue < MOTOR_MIN_LIMIT) PWMValue = MOTOR_MIN_LIMIT;
	pMotor->PWM1 = PWMValue;
	PWMValue = Thro + PIDGyrPWM.EulerPitch + PIDGyrPWM.EulerRoll + PIDGyrPWM.EulerYaw;
	if(PWMValue > MOTOR_MAX_LIMIT) PWMValue = MOTOR_MAX_LIMIT;
	if(PWMValue < MOTOR_MIN_LIMIT) PWMValue = MOTOR_MIN_LIMIT;
	pMotor->PWM2 = PWMValue;
	PWMValue = Thro + PIDGyrPWM.EulerPitch - PIDGyrPWM.EulerRoll - PIDGyrPWM.EulerYaw;
	if(PWMValue > MOTOR_MAX_LIMIT) PWMValue = MOTOR_MAX_LIMIT;
	if(PWMValue < MOTOR_MIN_LIMIT) PWMValue = MOTOR_MIN_LIMIT;
	pMotor->PWM3 = PWMValue;
	PWMValue = Thro - PIDGyrPWM.EulerPitch - PIDGyrPWM.EulerRoll + PIDGyrPWM.EulerYaw;
	if(PWMValue > MOTOR_MAX_LIMIT) PWMValue = MOTOR_MAX_LIMIT;
	if(PWMValue < MOTOR_MIN_LIMIT) PWMValue = MOTOR_MIN_LIMIT;
	pMotor->PWM4 = PWMValue;
}

/*************************************** END OF FILE ***************************************/
