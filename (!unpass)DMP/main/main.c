#include "main.h"

GPIO_InitTypeDef  GPIO_InitStructure;
MPU6050_InitTypeDef MPU6050_InitStruct;

uint8_t ErrorCode = 0;

AcceDef AcceOffset;
GyroDef GyroOffset;

float pitch, roll, yaw;

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

	/* Read ID */
	printf("Hello MPU6050! ID is %x \r\n", MPU6050_GetID());

	MPU6050_Correct(&AcceOffset, &GyroOffset);

	while(mpu_dmp_init())
	{
		printf("MPU6050 DMP Init Error!\r\n");
		Delay_ms(500);
	}
	printf("MPU6050 DMP Init OK!\r\n");
	while(1)
	{
		GPIOA->ODR ^= GPIO_Pin_8;//Toggle LED
		Delay_ms(500);
		printf("New Data : \r\n");
		if((ErrorCode = mpu_dmp_get_data(&pitch, &roll, &yaw)) == 0)
		{
			printf("------------------------------\r\n");
			printf("pitch: %2.2f\r\n", pitch);
			printf("roll : %2.2f\r\n", roll);
			printf("yaw  : %2.2f\r\n", yaw);
			printf("------------------------------\r\n");
		}
		else
			printf("<! Error code : %d\r\n", ErrorCode);
		GPIOD->ODR ^= GPIO_Pin_8;
		Delay_ms(500);
	}
}
