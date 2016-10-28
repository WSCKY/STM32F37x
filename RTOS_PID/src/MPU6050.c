/******************************************************************************
  * @file    MPU6050.c
  * @author  '^_^'
  * @version V1.0.0
  * @date    21-December-2013
  * @brief   This file includes the driver for MPU6050 senser.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MPU6050.h"
/* Private Defines -----------------------------------------------------------*/
#define SCL_H()								GPIO_SetBits(MPU6050_I2C_SCL_GPIO_PORT, MPU6050_I2C_SCL_PIN)
#define SCL_L()								GPIO_ResetBits(MPU6050_I2C_SCL_GPIO_PORT, MPU6050_I2C_SCL_PIN)

#define SDA_H()								GPIO_SetBits(MPU6050_I2C_SDA_GPIO_PORT, MPU6050_I2C_SDA_PIN)
#define SDA_L()								GPIO_ResetBits(MPU6050_I2C_SDA_GPIO_PORT, MPU6050_I2C_SDA_PIN)

#define SCL_Status()						GPIO_ReadInputDataBit(MPU6050_I2C_SCL_GPIO_PORT, MPU6050_I2C_SCL_PIN)
#define SDA_Status()						GPIO_ReadInputDataBit(MPU6050_I2C_SDA_GPIO_PORT, MPU6050_I2C_SDA_PIN)
/* Private Macros ------------------------------------------------------------*/
#define ABS(x)					((x > 0.0f)? (x) : (-x))
/* Private FunctionPrototypes ------------------------------------------------*/
static void MPU6050_LowLevel_Init(void);
static bool I2C_START(void);
static void I2C_STOP(void);
static void I2C_Ack(void);
static void I2C_NoAck(void);
static bool I2C_WaitAck(void);
static void I2C_SendByte(uint8_t SendByte);
static uint8_t I2C_ReadByte(void);
static void I2C_delay(void);

/**
  * @brief  Set MPU6050 Initialization.
  * @param  MPU6050_Config_Struct: pointer to a MPU6050_Config_TypeDef structure 
  *         that contains the configuration setting for the MPU6050.
  * @retval None
  */
void MPU6050_Init(MPU6050_InitTypeDef *MPU6050_InitStruct)
{
	/* Configure the low level interface ---------------------------------------*/
	MPU6050_LowLevel_Init();
	MEMS_Delay(30);
	/* Reset The MPU6050 Device */	
	MPU6050_Reset(RESET);
	MEMS_Delay(30);
	MPU6050_Reset(SET);

	/* Configure MEMS*/
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, \
							MPU6050_InitStruct->Sleep_Mode | \
							MPU6050_InitStruct->Clock_Mode);
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_2, \
							MPU6050_InitStruct->Wake_FREQ | \
							MPU6050_InitStruct->STDBY_Mode);
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, \
							MPU6050_InitStruct->Gyro_FullScale);
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, \
							MPU6050_InitStruct->Acce_FullScale | \
							MPU6050_InitStruct->HighPassFilter);
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, \
							MPU6050_InitStruct->SMPLRT_DIV);
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_CONFIG , \
							MPU6050_InitStruct->LowPassFilter | \
							MPU6050_InitStruct->EXT_SYNC);
}

/**
  * @brief  configure INT Pin
  * @param  MPU6050_INT_Struct: pointer to a MPU6050_INT_TypeDef structure 
  *         that contains the configuration setting for the MPU6050 INT Pin.
  * @retval None
  */
void MPU6050_INT_Init(MPU6050_INTTypeDef *MPU6050_INTStruct)
{
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, \
							MPU6050_INTStruct->INT_LEVEL | \
							MPU6050_INTStruct->INT_OPEN | \
							MPU6050_INTStruct->LATCH_INT_EN | \
							MPU6050_INTStruct->INT_RD_CLEAR);
	I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_INT_ENABLE, \
							MPU6050_INTStruct->DATA_RDY_EN);
}

void MPU6050_Reset(FlagStatus Status)
{
	if(Status == RESET)
		I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
	else
		I2C_WriteDeviceRegister(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_SET);
}

/**
  * @brief  Get MPU6050 ID.
  * @param  None.
  * @retval MPU6050's ID Value.
  */
uint8_t MPU6050_GetID(void)
{
	return(I2C_ReadDeviceRegister(MPU6050_ADDR, MPU6050_RA_WHO_AM_I) & 0x7e);
}

/**
  * @brief  Read the X axis acceleration
  * @param  None
  * @retval acceleration
  */
int16_t MPU6050_GetAcceX(void)
{
	return(I2C_ReadDataBuffer(MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H));
}

/**
  * @brief  Read the Y axis acceleration
  * @param  None
  * @retval acceleration
  */
int16_t MPU6050_GetAcceY(void)
{
	return(I2C_ReadDataBuffer(MPU6050_ADDR, MPU6050_RA_ACCEL_YOUT_H));
}

/**
  * @brief  Read the Z axis acceleration
  * @param  None
  * @retval acceleration
  */
int16_t MPU6050_GetAcceZ(void)
{
	return(I2C_ReadDataBuffer(MPU6050_ADDR, MPU6050_RA_ACCEL_ZOUT_H));
}

/**
  * @brief  Get acceleration.
  * @param  Acce: pointer to a acceleration structure.
  * @retval None
  */
void MPU6050_GetAcce(AcceDef *Acce)
{
	I2C_ReadIMUData(MPU6050_RA_ACCEL_XOUT_H, (IMUDataDef *)Acce);
}

/**
  * @brief  acceleration calibration.
  * @param  
  * @retval None
  */
void MPU6050_GetAcceCal(AcceDef *Acce, AcceDef *AcceOff)
{
	I2C_ReadIMUData(MPU6050_RA_ACCEL_XOUT_H, (IMUDataDef *)Acce);
	Acce->X -= AcceOff->X;
	Acce->Y -= AcceOff->Y;
	Acce->Z -= AcceOff->Z;
}

/**
  * @brief  Read the X axis angular velocity
  * @param  None
  * @retval gyroscope
  */
int16_t MPU6050_GetGyroX(void)
{
	return(I2C_ReadDataBuffer(MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H));
}

/**
  * @brief  Read the Y axis angular velocity
  * @param  None
  * @retval gyroscope
  */
int16_t MPU6050_GetGyroY(void)
{
	return(I2C_ReadDataBuffer(MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_H));
}

/**
  * @brief  Read the Z axis angular velocity
  * @param  None
  * @retval gyroscope
  */
int16_t MPU6050_GetGyroZ(void)
{
	return(I2C_ReadDataBuffer(MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_H));
}

/**
  * @brief  Get angular velocity.
  * @param  Gyro: pointer to a gyroscope structure.
  * @retval None
  */
void MPU6050_GetGyro(GyroDef *Gyro)
{
	I2C_ReadIMUData(MPU6050_RA_GYRO_XOUT_H, (IMUDataDef *)Gyro);
}

/**
  * @brief  angular velocity calibration.
  * @param  
  * @retval None
  */
void MPU6050_GetGyroCal(GyroDef *Gyro, GyroDef *GyroOff)
{
	I2C_ReadIMUData(MPU6050_RA_GYRO_XOUT_H, (IMUDataDef *)Gyro);
	Gyro->X -= GyroOff->X;
	Gyro->Y -= GyroOff->Y;
	Gyro->Z -= GyroOff->Z;
}

/**
  * @brief  Parameter calibration
  * @param  pointer to accelerometer calibration structure
  *			pointer to gyroscope calibration structure
  * @retval None
  */
void MPU6050_Correct(AcceDef *AcceOffset, GyroDef *GyroOffset)
{
	int16_t gyrox = 0, gyroy = 0, gyroz = 0, accex = 0, accey = 0, accez = 0;
	IMUDataDef DataGyr = {0}, OldDataGyr = {0}, DataAcc = {0}, OldDataAcc = {0};
	uint16_t Turb = 0;
	uint8_t index, PeaceGyr, PeaceAcc;
	do
	{
		I2C_ReadIMUData(MPU6050_RA_GYRO_XOUT_H, &DataGyr);
		I2C_ReadIMUData(MPU6050_RA_ACCEL_XOUT_H, &DataAcc);
		Turb =	ABS(DataGyr.X - OldDataGyr.X) + \
				ABS(DataGyr.Y - OldDataGyr.Y) + \
				ABS(DataGyr.Z - OldDataGyr.Z);
		if(Turb < 50)
			PeaceGyr ++;
		else
			PeaceGyr = 0;
		Turb =	ABS(DataAcc.X - OldDataAcc.X) + \
				ABS(DataAcc.Y - OldDataAcc.Y) + \
				ABS(DataAcc.Z - OldDataAcc.Z);
		if(Turb < 50)
			PeaceAcc ++;
		else
			PeaceAcc = 0;
		OldDataGyr = DataGyr;
		OldDataAcc = DataAcc;
		MEMS_Delay(5);
	}while((PeaceGyr <= 100));// || (PeaceAcc < 100));

	for(index = 0; index < 20; index ++)
	{
		I2C_ReadIMUData(MPU6050_RA_GYRO_XOUT_H, (IMUDataDef *)GyroOffset);
		I2C_ReadIMUData(MPU6050_RA_ACCEL_XOUT_H, (IMUDataDef *)AcceOffset);
		gyrox += GyroOffset->X;
		gyroy += GyroOffset->Y;
		gyroz += GyroOffset->Z;

		accex += AcceOffset->X;
		accey += AcceOffset->Y;
		accez += AcceOffset->Z;
		MEMS_Delay(1);
	}
	GyroOffset->X = gyrox / 20;
	GyroOffset->Y = gyroy / 20;
	GyroOffset->Z = gyroz / 20;
	AcceOffset->X = accex / 20;
	AcceOffset->Y = accey / 20;
	AcceOffset->Z = accez / 20;
}

/**
  * @brief  Writes a value in a register of the device through I2C.
  * @param  DeviceAddr: The address of the MPU6050, could be : MPU6050_ADDR
  * @param  RegisterAddr: The target register address
  * @param  RegisterValue: The target register value to be written.
  * @retval TRUE: if all operations are OK. Other value if error.
  */
bool I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue)
{
	if(!I2C_START()) return FALSE;
	/* Transmit the slave address and enable writing operation */
    I2C_SendByte(DeviceAddr);
	if(!I2C_WaitAck())
	{
		/* Send STOP Condition */  
		I2C_STOP();
		return FALSE;
	}
	/* Transmit the first address for r/w operations */
	I2C_SendByte(RegisterAddr);
	I2C_WaitAck();	
	I2C_SendByte(RegisterValue);
	I2C_WaitAck();
	/* Send STOP Condition */  
	I2C_STOP();
	return TRUE;
}

/**
  * @brief  Writes a value in a register of the device through I2C.
  * @param  DeviceAddr: The address of the MPU6050, could be : MPU6050_ADDR
  * @param  RegisterAddr: The target register address
  * @param  RegisterValue: The target register value to be written.
  * @retval TRUE: if all operations are OK. Other value if error.
  */
uint8_t I2C_WriteDeviceRegisterLength(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Length, uint8_t *RegisterValue)
{
	uint8_t Index;
	if(!I2C_START()) return 1;
	/* Transmit the slave address and enable writing operation */
    I2C_SendByte((DeviceAddr << 1) |0);
	if(!I2C_WaitAck())
	{
		/* Send STOP Condition */  
		I2C_STOP();
		return 1;
	}
	/* Transmit the first address for r/w operations */
	I2C_SendByte(RegisterAddr);
	I2C_WaitAck();
	for(Index = 0; Index < Length; Index ++)
	{
		I2C_SendByte(RegisterValue[Index]);
		if(I2C_WaitAck() == FALSE)
		{
			I2C_STOP();
			return 1;
		}
	}
	/* Send STOP Condition */  
	I2C_STOP();
	return 0;
}

/**
  * @brief  Reads a register of the device through I2C.
  * @param  DeviceAddr: The address of the device, could be : MPU6050_ADDR.
  * @param  RegisterAddr: The target register address.
  * @retval The value of the read register.
  */
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
	uint8_t REG_data;
	if(!I2C_START()) return FALSE;
	/* Send device address for write */
	I2C_SendByte(DeviceAddr);
	if(!I2C_WaitAck())
	{
		/* Send STOP Condition */
		I2C_STOP();
		return FALSE;
	}
	/* Send MPU6050 address for read */
	I2C_SendByte(RegisterAddr);
	I2C_WaitAck();
	I2C_START();
	I2C_SendByte(DeviceAddr + 1);
	I2C_WaitAck();
	/* Read data from MPU6050 */
	REG_data = I2C_ReadByte();
	I2C_NoAck();
	/* Send STOP Condition */
	I2C_STOP();
	return REG_data;
}

/**
  * @brief  Reads a register of the device through I2C.
  * @param  DeviceAddr: The address of the device, could be : MPU6050_ADDR.
  * @param  RegisterAddr: The target register address.
  * @retval The value of the read register.
  */
uint8_t I2C_ReadDeviceRegisterLength(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Length, uint8_t *RegisterValue)
{
	if(!I2C_START()) return 1;
	/* Send device address for write */
	I2C_SendByte((DeviceAddr << 1) | 0);
	if(!I2C_WaitAck())
	{
		/* Send STOP Condition */
		I2C_STOP();
		return 1;
	}
	/* Send MPU6050 address for read */
	I2C_SendByte(RegisterAddr);
	I2C_WaitAck();
	I2C_START();
	I2C_SendByte((DeviceAddr << 1) | 1);
	I2C_WaitAck();
	while(Length)
	{
		/* Read data from MPU6050 */
		*RegisterValue = I2C_ReadByte();
		if(Length == 1)
			I2C_NoAck();
		else
			I2C_Ack();
		Length --;
		RegisterValue ++;
	}
	/* Send STOP Condition */
	I2C_STOP();
	return 0;
}

/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : MPU6050_ADDR
  * @param  RegisterAddr: The target register address
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).  
  */
int16_t I2C_ReadDataBuffer(uint8_t DeviceAddr, uint32_t RegisterAddr)
{ 
	uint8_t tmp= 0;
	uint8_t MPU_BufferRX[2] = {0x00, 0x00};  
	
	MPU_BufferRX[0] = I2C_ReadDeviceRegister(DeviceAddr, RegisterAddr);
	MPU_BufferRX[1] = I2C_ReadDeviceRegister(DeviceAddr, RegisterAddr + 1);

	/* Reorganize received data */
	tmp = MPU_BufferRX[0];
	MPU_BufferRX[0] = MPU_BufferRX[1];
	MPU_BufferRX[1] = tmp;
	/* return a pointer to the MPU_Buffer */
	return *(int16_t *)MPU_BufferRX; 
}

/**
  * @brief  Get IMU data from MPU6050.(acceleration or gyroscope)
  * @param  RegisterAddr: MPU6050_RA_GYRO_XOUT_H or MPU6050_RA_ACCEL_XOUT_H.
  * @param  IMUData: pointer to IMU data structure.
  * @retval None
  */
void I2C_ReadIMUData(uint32_t RegisterAddr, IMUDataDef *IMUData)
{
	uint8_t IMUBuffer[6];
	I2C_ReadDeviceRegisterLength(MPU6050_ADDR, RegisterAddr, 6, IMUBuffer);
	((uint8_t *)&(IMUData->X))[0] = IMUBuffer[1];
	((uint8_t *)&(IMUData->X))[1] = IMUBuffer[0];
	((uint8_t *)&(IMUData->Y))[0] = IMUBuffer[3];
	((uint8_t *)&(IMUData->Y))[1] = IMUBuffer[2];
	((uint8_t *)&(IMUData->Z))[0] = IMUBuffer[5];
	((uint8_t *)&(IMUData->Z))[1] = IMUBuffer[4];
}

/**
  * @brief  Master Start Simulation IIC Communication.
  * @param  None.
  * @retval Start Status.
  */
static bool I2C_START(void)
{
	SDA_H();
	SCL_H();
	I2C_delay();
	if(!SDA_Status())return FALSE;
	SDA_L();
	I2C_delay();
	if(SDA_Status()) return FALSE;
	SDA_L();
	I2C_delay();
	return TRUE;
}

/**
  * @brief  Master Stop Simulation IIC Communication.
  * @param  None.
  * @retval None.
  */
static void I2C_STOP(void)
{
	SCL_L();
	I2C_delay();
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SDA_H();
	I2C_delay();
}

/**
  * @brief  Master Send Acknowledge Single.
  * @param  None.
  * @retval None.
  */
static void I2C_Ack(void)
{
	SCL_L();
	I2C_delay();
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SCL_L();
	I2C_delay();
}

/**
  * @brief  Master Send No Acknowledge Single.
  * @param  None.
  * @retval None.
  */
static void I2C_NoAck(void)
{
	SCL_L();
	I2C_delay();
	SDA_H();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SCL_L();
	I2C_delay();
}

/**
  * @brief  Master Reserive Slave Acknowledge Single.
  * @param  None.
  * @retval Ack Resualt.
  */
static bool I2C_WaitAck(void)
{
	SCL_L();
	I2C_delay();
	SDA_H();
	I2C_delay();
	SCL_H();
	I2C_delay();
	if(SDA_Status())
	{
		SCL_L();
		I2C_delay();
		return FALSE;
	}
	SCL_L();
	I2C_delay();
	return TRUE;
}

/**
  * @brief  Master Send a Byte to Slave.
  * @param  SendByte: the value to be written.
  * @retval None.
  */
static void I2C_SendByte(uint8_t SendByte)
{
	uint8_t index = 8;
	while(index --)
	{
		SCL_L();
		I2C_delay();
		if(SendByte&0x80) SDA_H();
		else SDA_L();
		SendByte <<= 1;
		I2C_delay();
		SCL_H();
		I2C_delay();
	}
	SCL_L();
}

/**
  * @brief  Master Reserive a Byte From Slave.
  * @param  None.
  * @retval the value read from Slave.
  */
static uint8_t I2C_ReadByte(void)
{
	uint8_t index = 8, ReceiveByte = 0;
	SDA_H();
	while(index--)
	{
		ReceiveByte<<=1;
		SCL_L();
		I2C_delay();
		SCL_H();
		I2C_delay();
		if(SDA_Status())
		{
			ReceiveByte |= 0x01;
		}
	}
	SCL_L();
	return ReceiveByte;
}

/**
  * @brief  Initializes the low level interface used to drive the MPU6050.
  * @param  None.
  * @retval None.
  */
static void MPU6050_LowLevel_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable SDA, SCL GPIO clocks */
	RCC_AHBPeriphClockCmd(MPU6050_I2C_SDA_GPIO_CLK | \
						   MPU6050_I2C_SCL_GPIO_CLK | \
						   MPU6050_I2C_INT_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* I2C SDA pin configuration */
	GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
	GPIO_Init(MPU6050_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* I2C SCL pin configuration */
	GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
	GPIO_Init(MPU6050_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	/* Configure GPIO PINs to detect Interrupts */
	GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_INT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(MPU6050_I2C_INT_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Inserts a delay time.
  * @param  None.
  * @retval None
  */
static void I2C_delay(void)
{
	uint16_t times = DELAY_TIME;
	while(times)
		times --;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
