#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private definition --------------------------------------------------------*/
#define MOTOR_SWITCH				0

#define SLOW						200
#define FAST						20
/* Private macro -------------------------------------------------------------*/
#define AcceP(x)					((x * 0.0122f) * 0.098f)
#define GyroP(x)					(x * 0.061f)
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;//GPIO Init Structure
MPU6050_InitTypeDef MPU6050_InitStruct;//MPU6050 Init Structure

AcceDef AcceOffset, Acce;
GyroDef GyroOffset, Gyro;

Axis A, G;
ANGLE Angle, ExpAngle;
MotorPWM pMotor;
Quaternions Q;

//Angle PID
PID RollPID, PitchPID;//, YawPID;
float KpRoll = 5.0f, KiRoll = 0.1f, KdRoll = 0.135f;
float KpPitch = 5.0f, KiPitch = 0.1f, KdPitch = 0.135f;
//float KpYaw = 0, KiYaw = 0, KdYaw = 0;
//Rate PID
PID RollGyrPID, PitchGyrPID, YawGyrPID;
float KpGyrRoll = 1.2f, KiGyrRoll = 0.022f, KdGyrRoll = 0.08f;
float KpGyrPitch = 1.2f, KiGyrPitch = 0.022f, KdGyrPitch = 0.08f;
float KpGyrYaw = 2.0f, KiGyrYaw = 2.0f, KdGyrYaw = 0.0f;

EulerPWM pidPWM, pidGyrPWM;
float Throttle = 0;
FlagStatus BatteryFlag = RESET;//default: Low voltage
FlagStatus InitStatus = RESET, FirstComp = RESET;

float fData[8] = {0};
float Voltage = 0;
uint8_t VolCnt = 0;
uint8_t LEDFlag = 0, LEDDelay;
/*uint32_t First = 0, Second = 0;*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	InitStatus = RESET;
	FirstComp = RESET;
	/* LED Initialization */
	LED_Init(BLE_LED | RED_LED);
	BLE_LED_On(); RED_LED_Off();//Init Start.
	Delay_Init(SystemCoreClock/1000000);

	/* UART Init */
	UART_Init(115200);

	/* ADC1 Channel4 configuration */
	ADC1_CH4_DMA_Config();
	/* Start conversion */
	ADC_SoftwareStartConv(ADC1);

	/* Set MEMS Initializes */
	MPU6050_InitStruct.Sleep_Mode = MPU6050_PWR1_SLEEP_DISABLE;
	MPU6050_InitStruct.Clock_Mode = MPU6050_CLOCK_INTERNAL;
	MPU6050_InitStruct.Wake_FREQ = MPU6050_WAKE_FREQ_1P25;
	MPU6050_InitStruct.STDBY_Mode = MPU6050_PWR2_STBY_None;
	MPU6050_InitStruct.Gyro_FullScale = MPU6050_GYRO_FS_2000;
	MPU6050_InitStruct.Acce_FullScale = MPU6050_ACCEL_FS_4;
	MPU6050_InitStruct.HighPassFilter = MPU6050_DHPF_RESET;
	MPU6050_InitStruct.LowPassFilter = MPU6050_DLPF_BW_20;
	MPU6050_InitStruct.EXT_SYNC = MPU6050_EXT_SYNC_DISABLED;
	MPU6050_InitStruct.SMPLRT_DIV = 0x04;
	MPU6050_Init(&MPU6050_InitStruct);

	/* Read ID */
//	printf("Hello MPU6050! ID is %x \r\n", MPU6050_GetID());

	MPU6050_Correct(&AcceOffset, &GyroOffset);//Calibration.
	TIMConfig();//TIM4 & TIM5
	PWM_GeneraterInit();//TIM19
	PWM_CCRSet(0, 0, 0, 0);

	CalCtrlCurve();
/*----------------------- PID parameter init -----------------------*/
	PID_Init(&RollPID, KpRoll, KiRoll, KdRoll);
	PID_Init(&PitchPID, KpPitch, KiPitch, KdPitch);
//	PID_Init(&YawPID, KpYaw, KiYaw, KdYaw);
	PID_Init(&RollGyrPID, KpGyrRoll, KiGyrRoll, KdGyrRoll);
	PID_Init(&PitchGyrPID, KpGyrPitch, KiGyrPitch, KdGyrPitch);
	PID_Init(&YawGyrPID, KpGyrYaw, KiGyrYaw, KdGyrYaw);

	PIDSetIntegralLimit(&RollPID, PID_ROLL_INTEGRATION_LIMIT);
	PIDSetIntegralLimit(&PitchPID, PID_PITCH_INTEGRATION_LIMIT);
//	PIDSetIntegralLimit(&YawPID, PID_YAW_INTEGRATION_LIMIT);
	PIDSetIntegralLimit(&RollGyrPID, PID_ROLL_GYR_INTEG_LIMIT);
	PIDSetIntegralLimit(&PitchGyrPID, PID_PITCH_GYR_INTEG_LIMIT);
	PIDSetIntegralLimit(&YawGyrPID, PID_YAW_GYR_INTEG_LIMIT);
/*----------------------- PID parameter init -----------------------*/

//	IMU_Init();
	Delay_ms(500);
	TIM2_ITInit();//TIM2
	BLE_LED_Off();//Init OK.
	InitStatus = SET;
	while(1)
	{
		//running...
/*		First = GetMicrosecond();*/
//		printf("Hello YUNEEC!\r\n");
//		ReportPWM(&pMotor);
//		ReportAttitute(&Angle);
//		Send_IMU(&Acce, &Gyro, &Angle);
//		ReportHost(pMotor);


//pidGyrPWM.EulerPitch;pidGyrPWM.EulerRoll;pidGyrPWM.EulerYaw;
//A.X;A.Y;A.Z;G.X;G.Y;
		LEDFlag ++;
		fData[0] = pMotor.PWM1;
		fData[1] = pMotor.PWM2;
		fData[2] = pMotor.PWM3;
		fData[3] = pMotor.PWM4;
		fData[4] = Angle.Pitch;
		fData[5] = Angle.Roll;
		fData[6] = Angle.Yaw;
		fData[7] = G.Z;
		Upload8Float(fData);
		Delay_ms(5);
		if(BatteryFlag)
			LEDDelay = SLOW;
		else
			LEDDelay = FAST;
		if(LEDFlag >= LEDDelay)
		{
			LEDFlag = 0;
			RED_LED_Toggle();
		}
/*
		Second = GetMicrosecond();
		if(Second >= First)
			First = Second - First;
		else
			First = Second + (0xFFFFFFFF - First);
		printf("Time:%d\r\n", First);*/
	}
}

/*
This function handles TIM2 interrupt request. <per 1ms>
*/
void TIM2_IRQHandler(void)
{
	if(((TIM2->SR & TIM_IT_Update) != RESET) && ((TIM2->DIER & TIM_IT_Update) != RESET))
	{
		static uint8_t TIM2Flag = 0;
		TIM2Flag ++;

		MPU6050_GetAcceCal(&Acce, &AcceOffset);//Get IMU Data
		MPU6050_GetGyroCal(&Gyro, &GyroOffset);
		//Data Process
		A.X = AcceP(((float)Acce.X)); A.Y = AcceP(((float)Acce.Y)); A.Z = AcceP(((float)Acce.Z));
		G.X = GyroP(((float)Gyro.X)); G.Y = GyroP(((float)Gyro.Y)); G.Z = GyroP(((float)Gyro.Z));
		//Quaternion Update
		Q = IMUAccGyr(&A, &G, 0.0005);//TimeCnt / 2

		if(TIM2Flag % 5 == 0)
		{
			TIM2Flag = 0;
			/*----------------------- angle control -----------------------*/
			//Get Euler Angle.
			QuaternionsToEuler(&Q, &Angle);

			pidPWM.EulerRoll = PIDUpdateT(&RollPID, ExpAngle.Roll - Angle.Roll, 0.005);
			pidPWM.EulerPitch = PIDUpdateT(&PitchPID, ExpAngle.Pitch - Angle.Pitch, 0.005);
			/*----------------------- angle control -----------------------*/
			
			Voltage = Voltage * 0.9f + ADC1ConvertedValue * 0.1f;
			if(Voltage >= 3000)
			{
				VolCnt = 0;
				BatteryFlag = SET;
			}
			else if(VolCnt >= 200)
			{
				VolCnt = 200;
				BatteryFlag = RESET;
			}
			else
			{
				VolCnt ++;
				BatteryFlag = SET;
			}
		}

		/*----------------------- angular control -----------------------*/
		pidGyrPWM.EulerRoll = PIDUpdateT(&RollGyrPID, pidPWM.EulerRoll - G.X, 0.001);//TimeCnt
		pidGyrPWM.EulerPitch = PIDUpdateT(&PitchGyrPID, pidPWM.EulerPitch - G.Y, 0.001);//TimeCnt
		pidGyrPWM.EulerYaw = PIDUpdateT(&YawGyrPID, ExpAngle.Yaw - G.Z, 0.001);//pidPWM.EulerYawTimeCnt
		/*----------------------- angular control -----------------------*/

		//PWM calculate.
		MotorPWMCompute(&pMotor, Throttle, pidGyrPWM);
		if((Throttle == 0) || (FirstComp == RESET))// && (!BatteryFlag))
		{
			PWM_CCRSet(0, 0, 0, 0);
			RollPID.iState = 0;
			PitchPID.iState = 0;
			RollGyrPID.iState = 0;
			PitchGyrPID.iState = 0;
			YawGyrPID.iState = 0;
		}
#if MOTOR_SWITCH
		else
		{
			PWM_CCRSet(pMotor.PWM1, pMotor.PWM2, pMotor.PWM3, pMotor.PWM4);
		}
#endif

		if(FirstComp == RESET)
		{
			FirstComp = SET;
			Q0.q0 = 1; Q0.q1 = 0; Q0.q2 = 0; Q0.q3 = 0;
		}

		TIM2->SR = (uint16_t)~TIM_IT_Update;
	}
}

/************************************ END OF FILE ******************************************/
