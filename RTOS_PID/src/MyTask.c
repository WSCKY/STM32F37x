/**
  ******************************************************************************
  * @file    MyTask.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    27-June-2015
  * @brief   System routine.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MyTask.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_SWITCH		0
/* Private macros ------------------------------------------------------------*/
#define AcceP(x)					((x * 0.0122f) * 0.098f)
#define GyroP(x)					(x * 0.061f)
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;//GPIO Init Structure
MPU6050_InitTypeDef MPU6050_InitStruct;//MPU6050 Init Structure
AcceDef AcceOffset, Acce;
GyroDef GyroOffset, Gyro;

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

Axis A, G;
ANGLE Angle, ExpAngle;
Quaternions Q;
MotorPWM pMotor;

EulerPWM pidPWM, pidGyrPWM;
float Throttle = 0;
FlagStatus BatteryFlag = RESET;//default: Low voltage
FlagStatus InitStatus = RESET, FirstComp = RESET;

/* RTOS Timer Handle */
osTimerId PIDTimer;

uint32_t TimeStart = 0, TimeEnd = 0;
float TimeCnt = 0;
uint32_t TA = 0, TB = 0;
float Cnt = 0;
/* Private function prototypes -----------------------------------------------*/
static void BSP_Init(void);
static void RatePIDTask(void const * argument);
static void MyUARTTask(void const * argument);
static void MyADCTask(void const * argument);
static void CPUUsage(void const * argument);
static void PIDTimerCallback(void const *n);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  BSP Init.
  * @param  None
  * @retval None
  */
static void BSP_Init(void)
{
	/* LED Init */
	LED_Init(BLE_LED | RED_LED);
	BLE_LED_On(); RED_LED_Off();//Init Start.

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

	CalCtrlCurve();//control curve calculation
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
	/* UART Init */
	UART_Init(115200);

	osDelay(500);

	BLE_LED_Off();//Init OK.
}

/**
  * @brief  Start task.
  * @param  None
  * @retval None
  */
void StartTask(void const * argument)
{
	InitStatus = RESET;
	FirstComp = RESET;
	BSP_Init();

	/* Rate PID Thread */
	osThreadDef(RatePID_Thread, RatePIDTask, osPriorityRealtime, 0, configMINIMAL_STACK_SIZE * 10);/* Highest Priority */
	osThreadCreate (osThread(RatePID_Thread), NULL);
	/* UART Thread */
	osThreadDef(UART_Thread, MyUARTTask, osPriorityHigh, 0, configMINIMAL_STACK_SIZE);/* High Priority */
	osThreadCreate (osThread(UART_Thread), NULL);
	/* ADC Thread */
	osThreadDef(ADC_Thread, MyADCTask, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE);/* Above Normal Priority */
	osThreadCreate (osThread(ADC_Thread), NULL);
	/* CPU Usage Thread */
	osThreadDef(Usage_Thread, CPUUsage, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);/* Normal(default) Priority */
	osThreadCreate (osThread(Usage_Thread), NULL);

	/* Create Timer(Angle PID) */
	osTimerDef(PIDTimer, PIDTimerCallback);/* STACK DEPTH = configMINIMAL_STACK_SIZE * 10 */
	PIDTimer = osTimerCreate(osTimer(PIDTimer), osTimerPeriodic, (void *)0);
	/* Start the Timer */
	osTimerStart(PIDTimer, 5);

	InitStatus = SET;

	for(;;)
	{
		//running...
		/* Toggles the Red LED */
		RED_LED_Toggle();
		if(BatteryFlag)
			osDelay(1000);/* Insert delay 1000 ms(slowly) */
		else
			osDelay(100);/* Insert delay 100 ms(fast) */
	}
}

static void RatePIDTask(void const * argument)
{
	while(1)
	{
		TimeStart = GetMicrosecond();
		if(TimeStart >= TimeEnd)
			TimeCnt = ((float)(TimeStart - TimeEnd) / 1000000.0f);
		else
			TimeCnt = ((float)(TimeStart + (0xFFFFFFFF - TimeEnd)) / 1000000.0f);
		TimeEnd = TimeStart;

		MPU6050_GetAcceCal(&Acce, &AcceOffset);//Get IMU Data
		MPU6050_GetGyroCal(&Gyro, &GyroOffset);
		//Data Process
		A.X = AcceP(((float)Acce.X)); A.Y = AcceP(((float)Acce.Y)); A.Z = AcceP(((float)Acce.Z));
		G.X = GyroP(((float)Gyro.X)); G.Y = GyroP(((float)Gyro.Y)); G.Z = GyroP(((float)Gyro.Z));
		//Quaternion Update
		Q = IMUAccGyr(&A, &G, TimeCnt / 2.0f);
/*----------------------- angular control -----------------------*/
		pidGyrPWM.EulerRoll = PIDUpdateT(&RollGyrPID, pidPWM.EulerRoll - G.X, TimeCnt);
		pidGyrPWM.EulerPitch = PIDUpdateT(&PitchGyrPID, pidPWM.EulerPitch - G.Y, TimeCnt);
		pidGyrPWM.EulerYaw = PIDUpdateT(&YawGyrPID, ExpAngle.Yaw - G.Z, TimeCnt);//pidPWM.EulerYaw
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
//			RollPID.iState = 0;
//			PitchPID.iState = 0;
//			RollGyrPID.iState = 0;
//			PitchGyrPID.iState = 0;
//			YawGyrPID.iState = 0;
		}
		osDelay(1);/* Delay 1ms */
	}
}

/* per 5ms */
static void PIDTimerCallback(void const *n)
{
	TA = GetMicrosecond();
	if(TA >= TB)
		Cnt = ((float)(TA - TB) / 1000000.0f);
	else
		Cnt = ((float)(TA + (0xFFFFFFFF - TB)) / 1000000.0f);
	TB = TA;
/*----------------------- angle control -----------------------*/
	//Get Euler Angle.
	QuaternionsToEuler(&Q, &Angle);

//	pidPWM.EulerRoll = PIDUpdate(&RollPID, ExpAngle.Roll - Angle.Roll, G.X);//
//	pidPWM.EulerPitch = PIDUpdate(&PitchPID, ExpAngle.Pitch - Angle.Pitch, G.Y);//
//	pidPWM.EulerYaw = PIDUpdate(&YawPID, ExpAngle.Yaw - Angle.Yaw, G.Z);
	pidPWM.EulerRoll = PIDUpdateT(&RollPID, ExpAngle.Roll - Angle.Roll, Cnt);
	pidPWM.EulerPitch = PIDUpdateT(&PitchPID, ExpAngle.Pitch - Angle.Pitch, Cnt);
//	pidPWM.EulerYaw = PIDUpdateT(&YawPID, ExpAngle.Yaw - Angle.Yaw, Cnt);
/*----------------------- angle control -----------------------*/
}

/**
  * @brief  Data upload by UART.
  * @param  None
  * @retval None
  */
static void MyUARTTask(void const * argument)
{
	float fData[8] = {0};
	while(1)
	{
//		printf("Hello YUNEEC!\r\n");
//		ReportPWM(&pMotor);
//		ReportAttitute(&Angle);
//		Send_IMU(&Acce, &Gyro, &Angle);
//		ReportHost(pMotor);
//
//
//pidGyrPWM.EulerPitch;pidGyrPWM.EulerRoll;pidGyrPWM.EulerYaw;
//A.X;A.Y;A.Z;G.X;G.Y;
		fData[0] = pMotor.PWM1;
		fData[1] = pMotor.PWM2;
		fData[2] = pMotor.PWM3;
		fData[3] = pMotor.PWM4;
		fData[4] = Angle.Pitch;
		fData[5] = Angle.Roll;
		fData[6] = Angle.Yaw;
		fData[7] = G.Z;
		Upload8Float(fData);
		osDelay(5);
	}
}

static void CPUUsage(void const * argument)
{
	while(1)
	{
		osGetCPUUsage();
		osDelay(1000);
	}
}

/**
  * @brief  Check Battery Voltage.
  * @param  None
  * @retval None
  */
static void MyADCTask(void const * argument)
{
	float Voltage = 0;
	uint8_t VolCnt = 0;
	while(1)
	{
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
		osDelay(10);
	}
}

/******************************** END OF FILE *********************************/
