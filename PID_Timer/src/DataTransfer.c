#include "DataTransfer.h"

#define BYTE0(x)		(*(uint8_t *)(&x))
#define BYTE1(x)		(*((uint8_t *)(&x) + 1))
#define BYTE2(x)		(*((uint8_t *)(&x) + 2))
#define BYTE3(x)		(*((uint8_t *)(&x) + 3))

uint8_t data_to_send[50];

void ReportPWM(MotorPWM *PWM)
{
	uint8_t _cnt = 0;
	data_to_send[_cnt ++] = 0xAA;
	data_to_send[_cnt ++] = 0xAA;
	data_to_send[_cnt ++] = 0x06;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = BYTE1(PWM->PWM1);
	data_to_send[_cnt ++] = BYTE0(PWM->PWM1);
	data_to_send[_cnt ++] = BYTE1(PWM->PWM2);
	data_to_send[_cnt ++] = BYTE0(PWM->PWM2);
	data_to_send[_cnt ++] = BYTE1(PWM->PWM3);
	data_to_send[_cnt ++] = BYTE0(PWM->PWM3);
	data_to_send[_cnt ++] = BYTE1(PWM->PWM4);
	data_to_send[_cnt ++] = BYTE0(PWM->PWM4);
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	
	data_to_send[3] = _cnt - 4;
	
	uint8_t sum = 0;
	for(uint8_t i = 0; i < _cnt; i ++)
		sum += data_to_send[i];
	
	data_to_send[_cnt ++] = sum;

	UART_SendString(data_to_send, _cnt);
}

void ReportAttitute(ANGLE *Angle)
{
	u8 _cnt = 0;
	data_to_send[_cnt ++] = 0xAA;
	data_to_send[_cnt ++] = 0xAA;
	data_to_send[_cnt ++] = 0x01;
	data_to_send[_cnt ++] = 0;
	vs16 _temp;
	_temp = (int)(Angle->Roll * 100);
	data_to_send[_cnt ++] = BYTE1(_temp);
	data_to_send[_cnt ++] = BYTE0(_temp);
	_temp = (int)(Angle->Pitch * 100);
	data_to_send[_cnt ++] = BYTE1(_temp);
	data_to_send[_cnt ++] = BYTE0(_temp);
	_temp = (int)(Angle->Yaw * 100);
	data_to_send[_cnt ++] = BYTE1(_temp);
	data_to_send[_cnt ++] = BYTE0(_temp);

	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
	data_to_send[_cnt ++] = 0;
		
	data_to_send[_cnt ++] = 0xA0;
	
	data_to_send[3] = _cnt - 4;
	
	uint8_t sum = 0;
	for(uint8_t i = 0; i < _cnt; i ++)
		sum += data_to_send[i];
	data_to_send[_cnt ++] = sum;

	UART_SendString(data_to_send, _cnt);
}
