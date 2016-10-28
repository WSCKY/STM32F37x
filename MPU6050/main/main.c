#include "main.h"

GPIO_InitTypeDef  GPIO_InitStructure;
MPU6050_InitTypeDef MPU6050_InitStruct;

AcceDef Acce, AcceOffset;
GyroDef Gyro, GyroOffset;

int main(void)
{
	Delay_Init(72);
	LED_Init(BLE_LED | RED_LED);
	BLE_LED_Off(); RED_LED_Off();

	/* USART1 Configuration */
	UART_Init(115200);
	BLE_LED_On();
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

	/* Read ID */
	printf("Hello MPU6050! ID is %x \r\n", MPU6050_GetID());

	printf("Start Calibration...\r\n");
	MPU6050_Correct(&AcceOffset, &GyroOffset);
	printf("Calibrate OK!----->\r\n");
	BLE_LED_Off();
	while(1)
	{
		BLE_LED_Toggle();
		Delay_ms(500);

//		MPU6050_GetAcce(&Acce);
//		MPU6050_GetGyro(&Gyro);
//		printf("GyroX=%2.2frad/s\nAcceX=%2.2fm/s^2 \r\n", \
//		(Gyro.X - GyroOffset.X) * 0.061 * 0.0174, \
//		((Acce.X - AcceOffset.X) * 0.0244) * 0.098);

//		printf("GyroY=%2.2frad/s\nAcceY=%2.2fm/s^2 \r\n", \
//		(Gyro.Y - GyroOffset.Y) * 0.061 * 0.0174, \
//		((Acce.Y - AcceOffset.Y) * 0.0244) * 0.098);

//		printf("GyroZ=%2.2frad/s\nAcceZ=%2.2fm/s^2 \r\n", \
//		(Gyro.Z - GyroOffset.Z) * 0.061 * 0.0174, \
//		((Acce.Z - AcceOffset.Z) * 0.0244) * 0.098);
//		RED_LED_Toggle();
//		Delay_ms(500);
	}
}
