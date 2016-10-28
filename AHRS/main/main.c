#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define AcceP(x)					((x * 0.0244f) * 0.098f)
#define GyroP(x)					((x * 0.061f) * 0.0174f)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;
MPU6050_InitTypeDef MPU6050_InitStruct;

AcceDef AcceOffset, Acce;
GyroDef GyroOffset, Gyro;

Axis A, G;
ANGLE Angle;
//Quaternions Q;

/* Private function prototypes -----------------------------------------------*/
void ReportNiming(uint8_t Fun, uint8_t *Buf, uint8_t Leng);
void Send_Data(AcceDef *Acce, GyroDef *Gyro);
void Send_IMU(AcceDef *Acce, GyroDef *Gyro, ANGLE *Angle);
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	Delay_Init(72);
	LED_Init(BLE_LED | RED_LED);
	BLE_LED_Off();
	RED_LED_On();
	
	/* USART1 Configuration */
	UART_Init(115200);
	
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
	IMU_Init();
	RED_LED_Off();
	BLE_LED_On();
	while(1)
	{
//		BLE_LED_Toggle();
		MPU6050_GetAcceCal(&Acce, &AcceOffset);
		MPU6050_GetGyroCal(&Gyro, &GyroOffset);
		A.X = AcceP(Acce.X); A.Y = AcceP(Acce.Y); A.Z = AcceP(Acce.Z);
		G.X = GyroP(Gyro.X); G.Y = GyroP(Gyro.Y); G.Z = GyroP(Gyro.Z);
		IMUAccGyr(&A, &G, &Angle);

//		Send_Data(&Acce, &Gyro);
		Send_IMU(&Acce, &Gyro, &Angle);
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
	uint8_t Buf[28], i;
	int16_t Roll = 100 * Angle->Roll, Pitch = 100 * Angle->Pitch, Yaw = 10 * Angle->Yaw;
	for(i = 0; i < 28; i ++) Buf[i] = 0;//flush
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
