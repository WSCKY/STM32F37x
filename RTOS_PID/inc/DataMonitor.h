#ifndef __DATAMONITOR_H
#define __DATAMONITOR_H

#include "main.h"

void ReportNiming(uint8_t Fun, uint8_t *Buf, uint8_t Leng);
void Send_Data(AcceDef *Acce, GyroDef *Gyro);
void Send_IMU(AcceDef *Acce, GyroDef *Gyro, ANGLE *Angle);
void ReportHost(MotorPWM pPWM);
void SendSystemParam(uint16_t Throt, ANGLE *Angle, MotorPWM *PWM, float Volt);
void Upload8Float(float *fData);

#endif /* DATAMONITOR_H */
