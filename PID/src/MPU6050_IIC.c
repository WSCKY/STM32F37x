/******************************************************************************
  * @file    MPU6050_IIC.c
  * @author  '^_^'
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file includes the driver for MPU6050 senser.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MPU6050_IIC.h"
/* Private Defines -----------------------------------------------------------*/
/* Private Macros ------------------------------------------------------------*/
#define ABS(x)					((x > 0)? x : (-x))
/* Private FunctionPrototypes ------------------------------------------------*/
static void MPU6050_LowLevel_Init(void);

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
	Delay_ms(30);
	/* Reset The MPU6050 Device */	
	MPU6050_Reset(RESET);
	Delay_ms(30);
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
	IMUDataDef DataGyr = {0}, DataAcc = {0}, OldDataGyr = {0}, OldDataAcc = {0};
	uint16_t Turb = 0;
	uint8_t index, PeaceGyr, PeaceAcc;
	do
	{
		I2C_ReadIMUData(MPU6050_RA_GYRO_XOUT_H, &DataGyr);
		Turb =	ABS(DataGyr.X - OldDataGyr.X) + \
				ABS(DataGyr.Y - OldDataGyr.Y) + \
				ABS(DataGyr.Z - OldDataGyr.Z);
		if(Turb < 50)
			PeaceGyr ++;
		else
			PeaceGyr = 0;
		I2C_ReadIMUData(MPU6050_RA_ACCEL_XOUT_H, &DataAcc);
		Turb =	ABS(DataAcc.X - OldDataAcc.X) + \
				ABS(DataAcc.Y - OldDataAcc.Y) + \
				ABS(DataAcc.Z - OldDataAcc.Z);
		if(Turb < 50)
			PeaceAcc ++;
		else
			PeaceAcc = 0;
		OldDataGyr = DataGyr;
		OldDataAcc = DataAcc;
		Delay_ms(5);
	}while((PeaceGyr < 100) || (PeaceAcc < 100));

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
		Delay_ms(1);
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
  * @retval None
  */
void I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue)
{
	/* Test on BUSY Flag */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_BUSY) != RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TXIS) == RESET);

	/* Send Register address */
	I2C_SendData(MPU6050_I2C, RegisterAddr);

	/* Wait until TCR flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TCR) == RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

	/* Wait until TXIS flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TXIS) == RESET);

	/* Write data to TXDR */
	I2C_SendData(MPU6050_I2C, RegisterValue);

	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 0, I2C_AutoEnd_Mode, I2C_Generate_Stop);

	/* Wait until STOPF flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_STOPF) == RESET);

	/* Clear STOPF flag */
	I2C_ClearFlag(MPU6050_I2C, I2C_ICR_STOPCF);
}

/**
  * @brief  Writes a value in a register of the device through I2C.
  * @param  DeviceAddr: The address of the MPU6050, could be : MPU6050_ADDR
  * @param  RegisterAddr: The target register address
  * @param  RegisterValue: The target register value to be written.
  * @retval None
  */
void I2C_WriteDeviceRegisterLength(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t NumByteToRead, uint8_t *RegisterValue)
{
	/* Test on BUSY Flag */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_BUSY) != RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TXIS) == RESET);

	/* Send Register address */
	I2C_SendData(MPU6050_I2C, RegisterAddr);

	/* Wait until TCR flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TCR) == RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_No_StartStop);

	while(NumByteToRead)
	{
		/* Wait until TXIS flag is set */
		while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TXIS) == RESET);

		/* Write data to TXDR */
		I2C_SendData(MPU6050_I2C, *RegisterValue);
		/* Point to the next location where the byte read will be saved */
		RegisterValue ++;
		/* Decrement the read bytes counter */
		NumByteToRead --;
	}

	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 0, I2C_AutoEnd_Mode, I2C_Generate_Stop);

	/* Wait until STOPF flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_STOPF) == RESET);

	/* Clear STOPF flag */
	I2C_ClearFlag(MPU6050_I2C, I2C_ICR_STOPCF);
}

/**
  * @brief  Reads a register of the device through I2C.
  * @param  DeviceAddr: The address of the device, could be : MPU6050_ADDR.
  * @param  RegisterAddr: The target register address.
  * @retval The value of the read register.
  */
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
	uint8_t Data = 0;
	/* Test on BUSY Flag */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_BUSY) != RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TXIS) == RESET);

	/* Send Register address */
	I2C_SendData(MPU6050_I2C, RegisterAddr);

	/* Wait until TC flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TC) == RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	/* Wait until RXNE flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_RXNE) == RESET);

	/* Read data from RXDR */
	Data = I2C_ReceiveData(MPU6050_I2C);

	/* Wait until STOPF flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_STOPF) == RESET);

	/* Clear STOPF flag */
	I2C_ClearFlag(MPU6050_I2C, I2C_ICR_STOPCF);

	/* If all operations OK */
	return Data;
}

/**
  * @brief  Reads a register of the device through I2C.
  * @param  DeviceAddr: The address of the device, could be : MPU6050_ADDR.
  * @param  RegisterAddr: The target register address.
  * @retval None
  */
void I2C_ReadDeviceRegisterLength(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t NumByteToRead, uint8_t *RegisterValue)
{
	/* Test on BUSY Flag */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_BUSY) != RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TXIS) == RESET);

	/* Send Register address */
	I2C_SendData(MPU6050_I2C, RegisterAddr);

	/* Wait until TC flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_TC) == RESET);

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU6050_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	/* Wait until all data are received */
	while(NumByteToRead)
	{   
		/* Wait until RXNE flag is set */
		while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_RXNE) == RESET);

		/* Read data from RXDR */
		*RegisterValue = I2C_ReceiveData(MPU6050_I2C);
		/* Point to the next location where the byte read will be saved */
		RegisterValue ++;
		/* Decrement the read bytes counter */
		NumByteToRead --;
	} 

	/* Wait until STOPF flag is set */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_ISR_STOPF) == RESET);

	/* Clear STOPF flag */
	I2C_ClearFlag(MPU6050_I2C, I2C_ICR_STOPCF);

	/* If all operations OK */
}

/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : MPU6050_ADDR
  * @param  RegisterAddr: The target register address
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).  
  */
int16_t I2C_ReadDataBuffer(uint8_t DeviceAddr, uint32_t RegisterAddr)
{ 
	uint8_t tmp = 0;
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
	((uint8_t *)(&(IMUData->X)))[0] = IMUBuffer[1];
	((uint8_t *)(&(IMUData->X)))[1] = IMUBuffer[0];
	((uint8_t *)(&(IMUData->Y)))[0] = IMUBuffer[3];
	((uint8_t *)(&(IMUData->Y)))[1] = IMUBuffer[2];
	((uint8_t *)(&(IMUData->Z)))[0] = IMUBuffer[5];
	((uint8_t *)(&(IMUData->Z)))[1] = IMUBuffer[4];
}

/**
  * @brief  Initializes the low level interface used to drive the MPU6050.
  * @param  None.
  * @retval None.
  */
static void MPU6050_LowLevel_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	RCC_AHBPeriphClockCmd(MPU6050_I2C_SDA_GPIO_CLK | \
						  MPU6050_I2C_SCL_GPIO_CLK | \
						  MPU6050_I2C_INT_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(MPU6050_I2C_CLK, ENABLE);

	GPIO_PinAFConfig(MPU6050_I2C_SCL_GPIO_PORT, MPU6050_I2C_SCL_SOURCE, GPIO_AF_4);
	GPIO_PinAFConfig(MPU6050_I2C_SDA_GPIO_PORT, MPU6050_I2C_SDA_SOURCE, GPIO_AF_4);
	/* I2C SDA,SCL pin configuration */
	GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN | MPU6050_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(MPU6050_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Disable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_Timing = 0x00902025;/* 100K */
	I2C_InitStructure.I2C_Timing = 0x00200404;/* 380K */
//	I2C_InitStructure.I2C_Timing = 0x20200404;/* 200K */

	I2C_Init(MPU6050_I2C, &I2C_InitStructure);
	I2C_Cmd(MPU6050_I2C, ENABLE);
}

/*************************************** END OF FILE ***************************************/
