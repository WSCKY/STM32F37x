/**
  ******************************************************************************
  * @file    IMU.c
  * @author  '^_^'
  * @modify  '^_^'
  * @version V0.0.1
  * @date    2-July-2015
  * @brief   IMU compute.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "IMU.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define KP 0.002f
#define KI 0.00f

#define Pitch_Offset 0.0f
#define Roll_Offset  -14.25f
#define Yaw_Offset   0.0f
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Quaternions Q0 = {1, 0, 0, 0}, Q1 = {1, 0, 0, 0};
float halfT;
uint32_t LastUpdate, Current;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  
  * @param  
  * @retval 
  */
void IMU_Init(void)
{
	LastUpdate = GetMicrosecond();
	Current = GetMicrosecond();
}

/**
  * @brief  IMU Update without Mag.
  * @param  
  * @retval None
  */
void IMUAccGyr(Axis *Acc, Axis *Gyr, ANGLE *Angle)
{
	float Norm = 0.0f;
	float Theta_2 = 0.0f;
	float Vx, Vy, Vz;
	float Ex, Ey, Ez;
	float ExInt, EyInt, EzInt;
	float q0q0 = Q0.q0 * Q0.q0;//q0*q0;
	float q0q1 = Q0.q0 * Q0.q1;//q0*q1;
	float q0q2 = Q0.q0 * Q0.q2;//q0*q2;
//	float q0q3 = Q0.q0 * Q0.q3;//q0*q3;
	float q1q1 = Q0.q1 * Q0.q1;//q1*q1;
//	float q1q2 = Q0.q1 * Q0.q2;//q1*q2;
	float q1q3 = Q0.q1 * Q0.q3;//q1*q3;
	float q2q2 = Q0.q2 * Q0.q2;//q2*q2;
	float q2q3 = Q0.q2 * Q0.q3;//q2*q3;
	float q3q3 = Q0.q3 * Q0.q3;//q3*q3;

	Current = GetMicrosecond();
	if(Current >= LastUpdate)
		halfT = ((float)(Current - LastUpdate) / 2000000.0f);
	else
		halfT = ((float)(Current + (0xFFFFFFFF - LastUpdate)) / 2000000.0f);
	LastUpdate = Current;

//	Norm = sqrt(Acc->X * Acc->X + Acc->Y * Acc->Y + Acc->Z * Acc->Z);
	arm_sqrt_f32((Acc->X * Acc->X + Acc->Y * Acc->Y + Acc->Z * Acc->Z), &Norm);
	Acc->X /= Norm; Acc->Y /= Norm; Acc->Z /= Norm;

	Vx = 2*(q1q3 - q0q2); Vy = 2*(q0q1 + q2q3); Vz = q0q0 - q1q1 - q2q2 + q3q3;

	Ex = (Acc->Y * Vz - Acc->Z * Vy);
	Ey = (Acc->Z * Vx - Acc->X * Vz);
	Ez = (Acc->X * Vy - Acc->Y * Vx);

	if(Ex != 0.0f && Ey != 0.0f && Ez != 0.0f)
	{
		ExInt += Ex * KI * halfT;
		EyInt += Ey * KI * halfT;
		EzInt += Ez * KI * halfT;
		Gyr->X += Ex * KP / halfT + ExInt;
		Gyr->Y += Ey * KP / halfT + EyInt;
		Gyr->Z += Ez * KP / halfT + EzInt;
	}

	Theta_2 = (2*halfT*Gyr->X)*(2*halfT*Gyr->X)+(2*halfT*Gyr->Y)*(2*halfT*Gyr->Y)+(2*halfT*Gyr->Z)*(2*halfT*Gyr->Z);

	Q1 = Q0;
	Q0.q0 = (1 - Theta_2/8) * Q1.q0 + (-Q1.q1 * Gyr->X - Q1.q2 * Gyr->Y - Q1.q3 * Gyr->Z) * halfT;
	Q0.q1 = (1 - Theta_2/8) * Q1.q1 + (Q1.q0 * Gyr->X + Q1.q2 * Gyr->Z - Q1.q3 * Gyr->Y) * halfT;
	Q0.q2 = (1 - Theta_2/8) * Q1.q2 + (Q1.q0 * Gyr->Y - Q1.q1 * Gyr->Z + Q1.q3 * Gyr->X) * halfT;
	Q0.q3 = (1 - Theta_2/8) * Q1.q3 + (Q1.q0 * Gyr->Z + Q1.q1 * Gyr->Y - Q1.q2 * Gyr->X) * halfT;

//	Norm = sqrt(Q0.q0 * Q0.q0 + Q0.q1 * Q0.q1 + Q0.q2 * Q0.q2 + Q0.q3 * Q0.q3);
	arm_sqrt_f32((Q0.q0 * Q0.q0 + Q0.q1 * Q0.q1 + Q0.q2 * Q0.q2 + Q0.q3 * Q0.q3), &Norm);
	Q0.q0 /= Norm; Q0.q1 /= Norm; Q0.q2 /= Norm; Q0.q3 /= Norm;

	Angle->Pitch = asin(-2 * Q0.q1 * Q0.q3 + 2 * Q0.q0 * Q0.q2)* 57.3f - Pitch_Offset;
	Angle->Roll = atan2(2 * Q0.q2 * Q0.q3 + 2 * Q0.q0 * Q0.q1, -2 * Q0.q1 * Q0.q1 - 2 * Q0.q2 * Q0.q2 + 1) * 57.3f - Roll_Offset;
	Angle->Yaw = atan2(2 * Q0.q1 * Q0.q2 + 2 * Q0.q0 * Q0.q3, -2 * Q0.q2 * Q0.q2 - 2 * Q0.q3 * Q0.q3 + 1) * 57.3f - Yaw_Offset;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void IMUUpdate(Axis *Acc, Axis *Gyr, Axis *Mag, ANGLE *Angle)
{
	float Norm = 0.0f;
	float Theta_2 = 0.0f;
	float Vx, Vy, Vz;
	float Ex, Ey, Ez;
	float Hx, Hy, Hz;
	float Bx, Bz;
	float Wx, Wy, Wz;
//	float Emx, Emy, Emz;
	float ExInt, EyInt, EzInt;
	float q0q0 = Q0.q0 * Q0.q0;//q0*q0;
	float q0q1 = Q0.q0 * Q0.q1;//q0*q1;
	float q0q2 = Q0.q0 * Q0.q2;//q0*q2;
	float q0q3 = Q0.q0 * Q0.q3;//q0*q3;
	float q1q1 = Q0.q1 * Q0.q1;//q1*q1;
	float q1q2 = Q0.q1 * Q0.q2;//q1*q2;
	float q1q3 = Q0.q1 * Q0.q3;//q1*q3;
	float q2q2 = Q0.q2 * Q0.q2;//q2*q2;
	float q2q3 = Q0.q2 * Q0.q3;//q2*q3;
	float q3q3 = Q0.q3 * Q0.q3;//q3*q3;

	Current = GetMicrosecond();
	if(Current >= LastUpdate)
		halfT = ((float)(Current - LastUpdate) / 2000000.0f);
	else
		halfT = ((float)(Current + (0xFFFFFFFF - LastUpdate)) / 2000000.0f);
	LastUpdate = Current;

//	Norm = sqrt(Acc->X * Acc->X + Acc->Y * Acc->Y + Acc->Z * Acc->Z);
	arm_sqrt_f32((Acc->X * Acc->X + Acc->Y * Acc->Y + Acc->Z * Acc->Z), &Norm);
	Acc->X /= Norm; Acc->Y /= Norm; Acc->Z /= Norm;
	
//	Norm = sqrt(Mag->X * Mag->X + Mag->Y * Mag->Y + Mag->Z * Mag->Z);
	arm_sqrt_f32((Mag->X * Mag->X + Mag->Y * Mag->Y + Mag->Z * Mag->Z), &Norm);
	Mag->X /= Norm; Mag->Y /= Norm; Mag->Z /= Norm;

	// compute reference direction of flux
	Hx = 2*Mag->X*(0.5f - q2q2 - q3q3) + 2*Mag->Y*(q1q2 - q0q3) + 2*Mag->Z*(q1q3 + q0q2);
	Hy = 2*Mag->X*(q1q2 + q0q3) + 2*Mag->Y*(0.5f - q1q1 - q3q3) + 2*Mag->Z*(q2q3 - q0q1);
	Hz = 2*Mag->X*(q1q3 - q0q2) + 2*Mag->Y*(q2q3 + q0q1) + 2*Mag->Z*(0.5f - q1q1 - q2q2);
//	Bx = sqrt((Hx*Hx) + (Hy*Hy));
	arm_sqrt_f32(((Hx*Hx) + (Hy*Hy)), &Bx);
	Bz = Hz;

	Vx = 2*(q1q3 - q0q2); Vy = 2*(q0q1 + q2q3); Vz = q0q0 - q1q1 - q2q2 + q3q3;
	Wx = 2*Bx*(0.5f - q2q2 - q3q3) + 2*Bz*(q1q3 - q0q2);
	Wy = 2*Bx*(q1q2 - q0q3) + 2*Bz*(q0q1 + q2q3);
	Wz = 2*Bx*(q0q2 + q1q3) + 2*Bz*(0.5f - q1q1 - q2q2);

//	Wx = 2*Hx*(0.5f - q2q2 - q3q3) + 2*Hy*(q1q2 + q0q3) + 2*Hz*(q1q3 - q0q2);
//	Wy = 2*Hx*(q1q2 - q0q3) + 2*Hy*(0.5f - q1q1 - q3q3) + 2*Hz*(q0q1 + q2q3);
//	Wz = 2*Hx*(q0q2 + q1q3) + 2*Hy*(q2q3 - q0q1) + 2*Hz*(0.5f - q1q1 - q2q2);

//	Wx = 1.34f*(0.5f - q2q2 - q3q3) + 1.14f*(q1q2 + q0q3) + 1.48f*(q1q3 - q0q2);
//	Wy = 1.34f*(q1q2 - q0q3) + 1.14f*(0.5f - q1q1 - q3q3) + 1.48f*(q0q1 + q2q3);
//	Wz = 1.34f*(q0q2 + q1q3) + 1.14f*(q2q3 - q0q1) + 1.48f*(0.5f - q1q1 - q2q2);

/* Error is sum of cross product between estimated direction and 
	measured direction of field vectors */
//	Emx = (My * Wz - Mz * Wy);
//	Emy = (Mz * Wx - Mx * Wz);
//	Emz = (Mx * Wy - My * Wx);
//	Ex = (Ay * Vz - Az * Vy) + Emx;
//	Ey = (Az * Vx - Ax * Vz) + Emy;
//	Ez = (Ax * Vy - Ay * Vx) + Emz;
	
	Ex = (Acc->Y * Vz - Acc->Z * Vy) + (Mag->Y * Wz - Mag->Z * Wy);
	Ey = (Acc->Z * Vx - Acc->X * Vz) + (Mag->Z * Wx - Mag->X * Wz);
	Ez = (Acc->X * Vy - Acc->Y * Vx) + (Mag->X * Wy - Mag->Y * Wx);

	if(Ex != 0.0f && Ey != 0.0f && Ez != 0.0f)
	{
		ExInt += Ex * KI * halfT;
		EyInt += Ey * KI * halfT;
		EzInt += Ez * KI * halfT;
		Gyr->X += Ex * KP / halfT + ExInt;
		Gyr->Y += Ey * KP / halfT + EyInt;
		Gyr->Z += Ez * KP / halfT + EzInt;
	}

	Theta_2 = (2*halfT*Gyr->X)*(2*halfT*Gyr->X)+(2*halfT*Gyr->Y)*(2*halfT*Gyr->Y)+(2*halfT*Gyr->Z)*(2*halfT*Gyr->Z);	

	Q1 = Q0;
	Q0.q0 = (1-Theta_2/8)*Q1.q0 + (-Q1.q1*Gyr->X - Q1.q2*Gyr->Y - Q1.q3*Gyr->Z)*halfT;
	Q0.q1 = (1-Theta_2/8)*Q1.q1 + (Q1.q0*Gyr->X + Q1.q2*Gyr->Z - Q1.q3*Gyr->Y)*halfT;
	Q0.q2 = (1-Theta_2/8)*Q1.q2 + (Q1.q0*Gyr->Y - Q1.q1*Gyr->Z + Q1.q3*Gyr->X)*halfT;
	Q0.q3 = (1-Theta_2/8)*Q1.q3 + (Q1.q0*Gyr->Z + Q1.q1*Gyr->Y - Q1.q2*Gyr->X)*halfT;

//	Norm = sqrt(Q0.q0 * Q0.q0 + Q0.q1 * Q0.q1 + Q0.q2 * Q0.q2 + Q0.q3 * Q0.q3);
	arm_sqrt_f32((Q0.q0 * Q0.q0 + Q0.q1 * Q0.q1 + Q0.q2 * Q0.q2 + Q0.q3 * Q0.q3), &Norm);
	Q0.q0 /= Norm; Q0.q1 /= Norm; Q0.q2 /= Norm; Q0.q3 /= Norm;

	Angle->Pitch = asin(-2 * Q0.q1 * Q0.q3 + 2 * Q0.q0 * Q0.q2)* 57.3 - Pitch_Offset;
	Angle->Roll = atan2(2 * Q0.q2 * Q0.q3 + 2 * Q0.q0 * Q0.q1, -2 * Q0.q1 * Q0.q1 - 2 * Q0.q2 * Q0.q2 + 1) * 57.3 - Roll_Offset;
	Angle->Yaw = atan2(2 * Q0.q1 * Q0.q2 + 2 * Q0.q0 * Q0.q3, -2 * Q0.q2 * Q0.q2 - 2 * Q0.q3 * Q0.q3 + 1) * 57.3 - Yaw_Offset;
}
