//
//                              _oo0oo_
//                             o8888888o
//                             88" . "88
//                             (| -_- |)
//                             0\  =  /0
//                           ___/`___'\___
//                         .' \\|     |// '.
//                        / \\|||  :  |||// \
//                       / _||||| -:- |||||- \
//                      |   | \\\  -  /// |   |
//                      | \_|  ''\---/''  |_/ |
//                      \  .-\__  '-'  ___/-. /
//                    ___'. .'  /--.--\  `. .'___
//                 ."" '<  `.___\_<|>_/___.' >' "".
//                | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//                \  \ `_.   \_ __\ /__ _/   .-` /  /
//            =====`-.____`.___ \_____/___.-`___.-'=====
//                              `=---='
//         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//                     ·ð×æ±£ÓÓ           ÓÀÎÞBUG
//

#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define AcceP(x)					((x * 0.0244) * 0.098)
#define GyroP(x)					((x * 0.061) * 0.0174)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;
MPU6050_InitTypeDef MPU6050_InitStruct;

AcceDef AcceOffset, Acce;
GyroDef GyroOffset, Gyro;

Axis A, G;
ANGLE Angle;
MotorPWM pMotor;

PID RollPID, PitchPID, YawPID;
float KpRoll = 8, KiRoll = 0.0f, KdRoll = 0.0f;
float KpPitch = 8, KiPitch = 0.0f, KdPitch = 0.0f;
float KpYaw = 0, KiYaw = 0, KdYaw = 0;

PID RollGyrPID, PitchGyrPID, YawGyrPID;
float KpGyrRoll = 0.5, KiGyrRoll = 0, KdGyrRoll = 0.0f;
float KpGyrPitch = 0.5, KiGyrPitch = 0, KdGyrPitch = 0.0f;
float KpGyrYaw = 0, KiGyrYaw = 0, KdGyrYaw = 0;

EulerPWM pidPWM, pidGyrPWM;
float Throttle = 0;
/* Private function prototypes -----------------------------------------------*/
void ReportNiming(uint8_t Fun, uint8_t *Buf, uint8_t Leng);
void Send_Data(AcceDef *Acce, GyroDef *Gyro);
void Send_IMU(AcceDef *Acce, GyroDef *Gyro, ANGLE *Angle);
void ReportHost(MotorPWM pPWM);
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	Delay_Init(72);
	/* LED GPIO Pin Configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOD, ENABLE);
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIOD->ODR ^= GPIO_Pin_8;
	GPIOA->ODR ^= GPIO_Pin_8;

	/* USART1 Configuration */
	UART_Init(9600);

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
	MPU6050_InitStruct.Acce_FullScale = MPU6050_ACCEL_FS_8;
	MPU6050_InitStruct.HighPassFilter = MPU6050_DHPF_5;
	MPU6050_InitStruct.LowPassFilter = MPU6050_DLPF_BW_98;
	MPU6050_InitStruct.EXT_SYNC = MPU6050_EXT_SYNC_DISABLED;
	MPU6050_InitStruct.SMPLRT_DIV = 0x01;
	MPU6050_Init(&MPU6050_InitStruct);

//	/* Read ID */
//	printf("Hello MPU6050! ID is %x \r\n", MPU6050_GetID());

	MPU6050_Correct(&AcceOffset, &GyroOffset);
	TIMConfig();
	PWM_GeneraterInit();
	PWM_CCRSet(0, 0, 0, 0);
	PID_Init(&RollPID, KpRoll, KiRoll, KdRoll);
	PID_Init(&PitchPID, KpPitch, KiPitch, KdPitch);
	PID_Init(&YawPID, KpYaw, KiYaw, KdYaw);
	PID_Init(&RollGyrPID, KpGyrRoll, KiGyrRoll, KdGyrRoll);
	PID_Init(&PitchGyrPID, KpGyrPitch, KiGyrPitch, KdGyrPitch);
	PID_Init(&YawGyrPID, KpGyrYaw, KiGyrYaw, KdGyrYaw);

	PIDSetIntegralLimit(&RollPID, PID_ROLL_INTEGRATION_LIMIT);
	PIDSetIntegralLimit(&PitchPID, PID_PITCH_INTEGRATION_LIMIT);
	PIDSetIntegralLimit(&YawPID, PID_YAW_INTEGRATION_LIMIT);
	PIDSetIntegralLimit(&RollGyrPID, PID_ROLL_GYR_INTEG_LIMIT);
	PIDSetIntegralLimit(&PitchGyrPID, PID_PITCH_GRY_INTEG_LIMIT);
	PIDSetIntegralLimit(&YawGyrPID, PID_YAW_GYR_INTEG_LIMIT);

	MPU6050_GetAcceCal(&Acce, &AcceOffset);//Get IMU Data
	MPU6050_GetGyroCal(&Gyro, &GyroOffset);
	//Data Process
	A.X = AcceP(Acce.X); A.Y = AcceP(Acce.Y); A.Z = AcceP(Acce.Z);
	G.X = GyroP(Gyro.X); G.Y = GyroP(Gyro.Y); G.Z = GyroP(Gyro.Z);

	TIM2_ITInit();
	IMU_Init();
//	GPIOD->ODR ^= GPIO_Pin_8;
	while(1)
	{
		MPU6050_GetAcceCal(&Acce, &AcceOffset);//Get IMU Data
		MPU6050_GetGyroCal(&Gyro, &GyroOffset);
		//Data Process
		A.X = AcceP(Acce.X); A.Y = AcceP(Acce.Y); A.Z = AcceP(Acce.Z);
		G.X = GyroP(Gyro.X); G.Y = GyroP(Gyro.Y); G.Z = GyroP(Gyro.Z);

//angular control
		pidGyrPWM.EulerRoll = GyrPIDUpdate(&RollGyrPID, pidPWM.EulerRoll - G.X);
		pidGyrPWM.EulerPitch = GyrPIDUpdate(&PitchGyrPID, pidPWM.EulerPitch - G.Y);
		pidGyrPWM.EulerYaw = GyrPIDUpdate(&YawGyrPID, pidPWM.EulerYaw - G.Z);
//angular control

		MotorPWMCompute(&pMotor, Throttle, pidGyrPWM);
//		ReportHost(pMotor);
		if((Throttle != 0))// && ((double)ADC1ConvertedValue * 3/0xFFF >= 2.39)
			PWM_CCRSet(pMotor.PWM1, pMotor.PWM2, pMotor.PWM3, pMotor.PWM4);
		else
			PWM_CCRSet(0, 0, 0, 0);
//		Send_Data(&Acce, &Gyro);
//		Send_IMU(&Acce, &Gyro, &Angle);
	}
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void ReportHost(MotorPWM pPWM)
{
	uint8_t Data[35], Index;
	uint32_t x;
	float pwm;
	Data[0] = 0x88;
	Data[1] = 0x77;
	pwm = (float)pPWM.PWM1;
	x = *(uint32_t *)&pwm;
	Data[2] = (x >> 0) & 0xFF; Data[3] = (x >> 8) & 0xFF; Data[4] = (x >> 16) & 0xFF; Data[5] = (x >> 24) & 0xFF;
	pwm = (float)pPWM.PWM2;
	x = *(uint32_t *)&pwm;
	Data[6] = (x >> 0) & 0xFF; Data[7] = (x >> 8) & 0xFF; Data[8] = (x >> 16) & 0xFF; Data[9] = (x >> 24) & 0xFF;
	pwm = (float)pPWM.PWM3;
	x = *(uint32_t *)&pwm;
	Data[10] = (x >> 0) & 0xFF; Data[11] = (x >> 8) & 0xFF; Data[12] = (x >> 16) & 0xFF; Data[13] = (x >> 24) & 0xFF;
	pwm = (float)pPWM.PWM4;
	x = *(uint32_t *)&pwm;
	Data[14] = (x >> 0) & 0xFF; Data[15] = (x >> 8) & 0xFF; Data[16] = (x >> 16) & 0xFF; Data[17] = (x >> 24) & 0xFF;
	pwm = (float)Angle.Pitch;
	x = *(uint32_t *)&pwm;
	Data[18] = (x >> 0) & 0xFF; Data[19] = (x >> 8) & 0xFF; Data[20] = (x >> 16) & 0xFF; Data[21] = (x >> 24) & 0xFF;
	pwm = (float)Angle.Roll;
	x = *(uint32_t *)&pwm;
	Data[22] = (x >> 0) & 0xFF; Data[23] = (x >> 8) & 0xFF; Data[24] = (x >> 16) & 0xFF; Data[25] = (x >> 24) & 0xFF;
	Data[34] = 0x99;
	for(Index = 0; Index < 35; Index ++)
	{
		USART_SendData(USART1, Data[Index]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ReportNiming(uint8_t Fun, uint8_t *Buf, uint8_t Leng)
{
	uint8_t Send[32], i;
	if(Leng > 28)
		return;
	Send[Leng + 3] = 0;
	Send[0] = 0x88;
	Send[1] = Fun;
	Send[2] = Leng;
	for(i = 0; i < Leng; i ++)
		Send[i + 3] = Buf[i];
	for(i = 0; i < Leng + 3; i ++)
		Send[Leng + 3] += Send[i];
	for(i = 0; i < Leng + 4; i ++)
	{
		USART_SendData(USART1, Send[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void Send_Data(AcceDef *Acce, GyroDef *Gyro)
{
	uint8_t Buf[12];
	Buf[0] = (Acce->X >> 8) & 0xFF; Buf[1] = (Acce->X) & 0xFF;
	Buf[2] = (Acce->Y >> 8) & 0xFF; Buf[3] = (Acce->Y) & 0xFF;
	Buf[4] = (Acce->Z >> 8) & 0xFF; Buf[5] = (Acce->Z) & 0xFF;
	Buf[6] = (Gyro->X >> 8) & 0xFF; Buf[7] = (Gyro->X) & 0xFF;
	Buf[8] = (Gyro->Y >> 8) & 0xFF; Buf[9] = (Gyro->Y) & 0xFF;
	Buf[10] = (Gyro->Z >> 8) & 0xFF; Buf[11] = (Gyro->Z) & 0xFF;
	ReportNiming(0xA1, Buf, 12);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void Send_IMU(AcceDef *Acce, GyroDef *Gyro, ANGLE *Angle)
{
	uint8_t Buf[28];//, i;
	int16_t Roll = 100 * Angle->Roll, Pitch = 100 * Angle->Pitch, Yaw = 10 * Angle->Yaw;
//	for(i = 0; i < 28; i ++) Buf[i] = 0;//flush
	Buf[0] = (Acce->X >> 8) & 0xFF; Buf[1] = (Acce->X) & 0xFF;
	Buf[2] = (Acce->Y >> 8) & 0xFF; Buf[3] = (Acce->Y) & 0xFF;
	Buf[4] = (Acce->Z >> 8) & 0xFF; Buf[5] = (Acce->Z) & 0xFF;
	Buf[6] = (Gyro->X >> 8) & 0xFF; Buf[7] = (Gyro->X) & 0xFF;
	Buf[8] = (Gyro->Y >> 8) & 0xFF; Buf[9] = (Gyro->Y) & 0xFF;
	Buf[10] = (Gyro->Z >> 8) & 0xFF; Buf[11] = (Gyro->Z) & 0xFF;
//	Buf[12] = (Magn->MagX >> 8) & 0xFF; Buf[13] = (Magn->MagX) & 0xFF;
//	Buf[14] = (Magn->MagY >> 8) & 0xFF; Buf[15] = (Magn->MagY) & 0xFF;
//	Buf[16] = (Magn->MagZ >> 8) & 0xFF; Buf[17] = (Magn->MagZ) & 0xFF;
	Buf[18] = (Roll >> 8) & 0xFF; Buf[19] = (Roll) & 0xFF;
	Buf[20] = (Pitch >> 8) & 0xFF; Buf[21] = (Pitch) & 0xFF;
	Buf[22] = (Yaw >> 8) & 0xFF; Buf[23] = (Yaw) & 0xFF;
	ReportNiming(0xAF, Buf, 28);
}
