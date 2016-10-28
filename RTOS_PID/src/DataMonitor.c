#include "DataMonitor.h"

#define BYTE0(x)		(*(uint8_t *)(&x))
#define BYTE1(x)		(*((uint8_t *)(&x) + 1))
#define BYTE2(x)		(*((uint8_t *)(&x) + 2))
#define BYTE3(x)		(*((uint8_t *)(&x) + 3))

/**
  * @brief  
  * @param  
  * @retval 
  */
void Upload8Float(float *fData)
{
	uint8_t Index;
	while((USART1->ISR & 0x40) == 0);
	USART1->TDR = 0x88;
	while((USART1->ISR & 0x40) == 0);
	USART1->TDR = 0x77;
	for(Index = 0; Index < 8; Index ++)
	{
		while((USART1->ISR & 0x40) == 0);
		USART1->TDR = BYTE0(*(fData + Index));
		while((USART1->ISR & 0x40) == 0);
		USART1->TDR = BYTE1(*(fData + Index));
		while((USART1->ISR & 0x40) == 0);
		USART1->TDR = BYTE2(*(fData + Index));
		while((USART1->ISR & 0x40) == 0);
		USART1->TDR = BYTE3(*(fData + Index));
	}
	while((USART1->ISR & 0x40) == 0);
	USART1->TDR = 0x99;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void ReportHost(MotorPWM pPWM)
{
	uint8_t i = 0;
	volatile float data[8]={pPWM.PWM1, pPWM.PWM2, pPWM.PWM3, pPWM.PWM4, 0, 0, 0, 0};
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, 0x88);

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, 0x77);

	for(i = 0; i < 8; i ++)
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, ((char*)(data + i))[0]);

		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, ((char*)(data + i))[1]);

		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, ((char*)(data + i))[2]);

		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, ((char*)(data + i))[3]);
	}

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, 0x99);
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

void SendSystemParam(uint16_t Throt, ANGLE *Angle, MotorPWM *PWM, float Volt)
{
	uint8_t Buf[28];
	uint16_t V;
	int16_t Roll = 100 * Angle->Roll, Pitch = 100 * Angle->Pitch, Yaw = 10 * Angle->Yaw;
	PWM->PWM1 /= 10; PWM->PWM2 /= 10; PWM->PWM3 /= 10; PWM->PWM4 /= 10;
	V = Volt * 100;
	Buf[0] = (Throt >> 8) & 0xFF; Buf[1] = Throt & 0xFF;
	Buf[2] = (Yaw >> 8) & 0xFF; Buf[3] = (Yaw) & 0xFF;
	Buf[4] = (Roll >> 8) & 0xFF; Buf[5] = (Roll) & 0xFF;
	Buf[6] = (Pitch >> 8) & 0xFF; Buf[7] = (Pitch) & 0xFF;
	Buf[18] = (PWM->PWM3 >> 8) & 0xFF; Buf[19] = (PWM->PWM3) & 0xFF;
	Buf[20] = (PWM->PWM4 >> 8) & 0xFF; Buf[21] = (PWM->PWM4) & 0xFF;
	Buf[22] = (PWM->PWM1 >> 8) & 0xFF; Buf[23] = (PWM->PWM1) & 0xFF;
	Buf[24] = (PWM->PWM2 >> 8) & 0xFF; Buf[25] = (PWM->PWM2) & 0xFF;
	Buf[26] = (V >> 8) & 0xFF; Buf[27] = (V) & 0xFF;
	ReportNiming(0xAE, Buf, 28);
}

/************************************* END OF FILE ****************************************/
