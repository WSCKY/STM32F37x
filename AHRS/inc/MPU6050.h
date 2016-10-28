/**
  ******************************************************************************
  * @file    MPU6050.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    3-February-2015
  * @brief   Header file for MPU6050.c
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H
#define __MPU6050_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"
#include "Delay.h"

/** 
  * @brief  MPU6050 DMA Direction  
  */ 
typedef enum
{
	MPU6050_DMA_TX = 0,
	MPU6050_DMA_RX = 1
}MPU6050_DMADirection_TypeDef;

typedef enum
{
	FALSE = 0,
	TRUE = !FALSE
}bool;

/**
  * @brief  MPU6050 Struct
  */
typedef struct
{
	uint8_t Sleep_Mode;
	uint8_t Clock_Mode;
	uint8_t Wake_FREQ;
	uint8_t STDBY_Mode;
	uint8_t Gyro_FullScale;
	uint8_t Acce_FullScale;
	uint8_t HighPassFilter;
	uint8_t LowPassFilter;
	uint8_t EXT_SYNC;
	uint8_t SMPLRT_DIV;
}MPU6050_InitTypeDef;

/**
  * @brief  MPU6050 INT Struct
  */
typedef struct
{
	uint8_t INT_LEVEL;
	uint8_t INT_OPEN;
	uint8_t LATCH_INT_EN;
	uint8_t INT_RD_CLEAR;
	uint8_t DATA_RDY_EN;
}MPU6050_INTTypeDef;

typedef struct
{
	int16_t AcceX;
	int16_t AcceY;
	int16_t AcceZ;
}AcceDef;

typedef struct
{
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
}GyroDef;

/**
  * @brief  MPU6050 I2C Interface pins
  */
#define MPU6050_I2C_SDA_PIN					GPIO_Pin_7					/* PF.07 */
#define MPU6050_I2C_SDA_GPIO_PORT			GPIOF						/* GPIOF */
#define MPU6050_I2C_SDA_GPIO_CLK			RCC_AHBPeriph_GPIOF
#define MPU6050_I2C_SDA_SOURCE				GPIO_PinSource7

#define MPU6050_I2C_SCL_PIN					GPIO_Pin_6					/* PF.06 */
#define MPU6050_I2C_SCL_GPIO_PORT			GPIOF						/* GPIOF */
#define MPU6050_I2C_SCL_GPIO_CLK			RCC_AHBPeriph_GPIOF
#define MPU6050_I2C_SCL_SOURCE				GPIO_PinSource6

#define MPU6050_I2C_INT_PIN					GPIO_Pin_0					/* PB.00 */
#define MPU6050_I2C_INT_GPIO_PORT			GPIOB						/* GPIOB */
#define MPU6050_I2C_INT_GPIO_CLK			RCC_AHBPeriph_GPIOB
#define MPU6050_I2C_INT_EXTI_LINE			EXTI_Line0
#define MPU6050_I2C_INT_EXTI_PORT_SOURCE	EXTI_PortSourceGPIOB
#define MPU6050_I2C_INT_EXTI_PIN_SOURCE		EXTI_PinSource0
#define MPU6050_I2C_INT_EXTI_IRQn			EXTI0_IRQn

#define CorCount							200
#define MPU6050_ADDR						0xD0
#define DELAY_TIME							20

/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/

/**
  *The register map for the MPU-60X0 is listed below.
  */

/**
  *Note: Register Names ending in _H and _L contain the high and low bytes, respectively,
  *of an internal register value.
  *In the detailed register tables that follow, register names are in capital letters,
  *while register values are in capital letters and italicized. For example, the ACCEL_XOUT_H
  *register (Register 59) contains the 8 most significant bits, ACCEL_XOUT[15:8], of the 16-bit
  *X-Axis accelerometer measurement, ACCEL_XOUT.
  *The reset value is 0x00 for all registers other than the registers below :
  *		Register 107: 0x40.
  *		Register 117: 0x68.
  */
#define MPU6050_RA_XG_OFFS_TC				0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC				0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC				0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN				0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN				0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN				0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H				0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC				0x07
#define MPU6050_RA_YA_OFFS_H				0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC				0x09
#define MPU6050_RA_ZA_OFFS_H				0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC				0x0B
#define MPU6050_RA_XG_OFFS_USRH				0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL				0x14
#define MPU6050_RA_YG_OFFS_USRH				0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL				0x16
#define MPU6050_RA_ZG_OFFS_USRH				0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL				0x18

/************************************************************************************************
*Register 25(R19h) 每 Sample Rate Divider
*Type: Read/Write
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|Register (Hex)|Register (Decimal)| Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|		1A	    |        26        |					SMPLRT_DIV[7:0]					   |
*+--------------+------------------+-------------------------------------------------------+
*Description:
*	This register specifies the divider from the gyroscope output rate used to generate the Sample Rate for the MPU-60X0.
*	The sensor register output, FIFO output, DMP sampling, Motion detection, Zero Motion detection, and Free Fall detection are all based on the Sample Rate.
*	The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:
*	Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
*	where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled (see Register 26).
*Note: The accelerometer output rate is 1kHz. This means that for a Sample Rate greater than 1kHz, the same accelerometer sample may be output to the FIFO, DMP, and sensor registers more than once.
*	For a diagram of the gyroscope and accelerometer signal paths, see Section 8 of the MPU-6000/MPU-6050 Product Specification document.
*Parameters:
*	SMPLRT_DIV	8-bit unsigned value. The Sample Rate is determined by dividing the gyroscope output rate by this value.
************************************************************************************************/
#define MPU6050_RA_SMPLRT_DIV				0x19
/************************************************************************************************
*Register 26(R1Ah) 每 Configuration
*Type: Read/Write
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|Register (Hex)|Register (Decimal)| Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|		1A	    |        26        |   -  |   -  |  EXT_SYNC_SET[2:0] |    DLPF_CFG[2:0]   |
*+--------------+------------------+------+------+--------------------+--------------------+
*Description:
*	This register configures the external Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.
*	An external signal connected to the FSYNC pin can be sampled by configuring EXT_SYNC_SET.
*	Signal changes to the FSYNC pin are latched so that short strobes may be captured. The latched FSYNC signal will be sampled at the Sampling Rate, as defined in register 25. After sampling, the latch will reset to the current FSYNC signal state.
*	The sampled value will be reported in place of the least significant bit in a sensor data register determined by the value of EXT_SYNC_SET according to the following table.
*	EXT_SYNC_SET-----FSYNC Bit Location
*	      0            Input disabled
*	      1            TEMP_OUT_L[0]
*	      2            GYRO_XOUT_L[0]
*	      3            GYRO_YOUT_L[0]
*	      4            GYRO_ZOUT_L[0]
*	      5            ACCEL_XOUT_L[0]
*	      6            ACCEL_YOUT_L[0]
*	      7            ACCEL_ZOUT_L[0]
*	The DLPF is configured by DLPF_CFG. The accelerometer and gyroscope are filtered according to the value of DLPF_CFG as shown in the table below.
*	+--------+-------------------------+----------------------------------+
*	|DLPF_CFG|Accelerometer (Fs = 1kHz)|			  Gyroscope			  |
*	+--------+--------------+----------+--------------+----------+--------+
*	|        |Bandwidth (Hz)|Delay (ms)|Bandwidth (Hz)|Delay (ms)|Fs (kHz)|
*	+--------+--------------+----------+--------------+----------+--------+
*	|	0	 |		260		|	 0     |	 256 	  |   0.98   |	  8   |
*	+--------+--------------+----------+--------------+----------+--------+
*	|	1	 |		184		|	 2.0   |	 188 	  |    1.9   |	  1   |
*	+--------+--------------+----------+--------------+----------+--------+
*	|	2	 |		94		|	 3.0   |	  98 	  |    2.8   |	  1   |
*	+--------+--------------+----------+--------------+----------+--------+
*	|	3	 |		44		|	 4.9   |	  42 	  |    4.8   |	  1   |
*	+--------+--------------+----------+--------------+----------+--------+
*	|	4	 |		21		|	 8.5   |	  20 	  |    8.3   |	  1   |
*	+--------+--------------+----------+--------------+----------+--------+
*	|	5	 |		10		|	 13.8  |	  10 	  |    13.4  |	  1   |
*	+--------+--------------+----------+--------------+----------+--------+
*	|	6	 |		5		|	 19.0  |	  5 	  |    18.6  |	  1   |
*	+--------+--------------+----------+--------------+----------+--------+
*	|	7	 |			Reserved	   |		  Reserved		 |	  8   |
*	+--------+-------------------------+-------------------------+--------+
*	Bit 7 and bit 6 are reserved.
*Parameters:
*	EXT_SYNC_SET 3-bit unsigned value. Configures the FSYNC pin sampling.
*	DLPF_CFG 3-bit unsigned value. Configures the DLPF setting.
************************************************************************************************/
#define MPU6050_RA_CONFIG					0x1A
/************************************************************************************************
*Register 27(R1Bh) 每 Gyroscope Configuration
*Type: Read/Write
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|Register (Hex)|Register (Decimal)| Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|		1B	    |        27        | XG_ST| YG_ST| ZG_ST| FS_SEL[1:0] |   -  |	-   |   -  |
*+--------------+------------------+------+------+------+-------------+------+------+------+
*Description:
*	This register is used to trigger gyroscope self-test and configure the gyroscopes＊ full scale range.
*	Gyroscope self-test permits users to test the mechanical and electrical portions of the gyroscope. The self-test for each gyroscope axis can be activated by controlling the XG_ST, YG_ST, and ZG_ST bits of this register. Self-test for each axis may be performed independently or all at the same time.
*	When self-test is activated, the on-board electronics will actuate the appropriate sensor. This actuation will move the sensor＊s proof masses over a distance equivalent to a pre-defined Coriolis force. This proof mass displacement results in a change in the sensor output, which is reflected in the output signal. The output signal is used to observe the self-test response.
*	The self-test response is defined as follows:
*	Self-test response = Sensor output with self-test enabled 每 Sensor output without self-test enabled
*	The self-test limits for each gyroscope axis is provided in the electrical characteristics tables of the MPU-6000/MPU-6050 Product Specification document. When the value of the self-test response is within the min/max limits of the product specification, the part has passed self test. When the self-test response exceeds the min/max values specified in the document, the part is deemed to have failed self-test.
*	FS_SEL selects the full scale range of the gyroscope outputs according to the following table.
*	FS_SEL-----Full Scale Range
*	  0			   ㊣ 250 ～/s
*	  1			   ㊣ 500 ～/s
*	  2			  ㊣ 1000 ～/s
*	  3			  ㊣ 2000 ～/s
*	Bits 2 through 0 are reserved.
*Parameters:
*	XG_ST	Setting this bit causes the X axis gyroscope to perform self test.
*	YG_ST	Setting this bit causes the Y axis gyroscope to perform self test.
*	ZG_ST	Setting this bit causes the Z axis gyroscope to perform self test.
*	FS_SEL	2-bit unsigned value. Selects the full scale range of gyroscopes.
************************************************************************************************/
#define MPU6050_RA_GYRO_CONFIG				0x1B
/************************************************************************************************
*Register 28(R1Ch) 每 Accelerometer Configuration
*Type: Read/Write
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|Register (Hex)|Register (Decimal)| Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|		1C	    |        28        | XA_ST| YA_ST| ZA_ST| AFS_SEL[1:0]|   ACCEL_HPF[2:0]   |
*+--------------+------------------+------+------+------+-------------+--------------------+
*Description:
*	This register is used to trigger accelerometer self test and configure the accelerometer full scale range. This register also configures the Digital High Pass Filter (DHPF).
*	Accelerometer self-test permits users to test the mechanical and electrical portions of the accelerometer. The self-test for each accelerometer axis can be activated by controlling the XA_ST, YA_ST, and ZA_ST bits of this register. Self-test for each axis may be performed independently or all at the same time.
*	When self-test is activated, the on-board electronics will actuate the appropriate sensor. This actuation simulates an external force. The actuated sensor, in turn, will produce a corresponding output signal. The output signal is used to observe the self-test response.
*	The self-test response is defined as follows:
*	Self-test response = Sensor output with self-test enabled - Sensor output without self-test enabled
*	The self-test limits for each accelerometer axis is provided in the electrical characteristics tables of the MPU-6000/MPU-6050 Product Specification document. When the value of the self-test response is within the min/max limits of the product specification, the part has passed self test. When the self-test response exceeds the min/max values specified in the document, the part is deemed to have failed self-test.
*	AFS_SEL	selects the full scale range of the accelerometer outputs according to the following table.
*	AFS_SEL-----Full Scale Range
*	   0			  ㊣2g
*	   1			  ㊣4g
*	   2			  ㊣8g
*	   3			  ㊣16g
*	ACCEL_HPF	configures the DHPF available in the path leading to motion detectors (Free Fall, Motion threshold, and Zero Motion). The high pass filter output is not available to the data registers (see Figure in Section 8 of the MPU-6000/MPU-6050 Product Specification document).
*	The high pass filter has three modes:
*	Reset: The filter output settles to zero within one sample. This effectively disables the high pass filter. This mode may be toggled to quickly settle the filter.
*	On: The high pass filter will pass signals above the cut off frequency.
*	Hold: When triggered, the filter holds the present sample. The filter output will be the difference between the input sample and the held sample.
*	ACCEL_HPF-----Filter Mode-----Cut-off Frequency
*		0			Reset				 None
*		1			 On					 5Hz
*		2			 On					 2.5Hz
*		3			 On					 1.25Hz
*		4			 On					 0.63Hz
*		7			Hold				 None
*Parameters:
*	XA_ST			When set to 1, the X- Axis accelerometer performs self test.
*	YA_ST			When set to 1, the Y- Axis accelerometer performs self test.
*	ZA_ST			When set to 1, the Z- Axis accelerometer performs self test.
*	ACCEL_FS_SEL	2-bit unsigned value. Selects the full scale range of accelerometers.
*	ACCEL_HPF		3-bit unsigned value. Selects the Digital High Pass Filter configuration.
************************************************************************************************/
#define MPU6050_RA_ACCEL_CONFIG				0x1C

#define MPU6050_RA_FF_THR					0x1D
#define MPU6050_RA_FF_DUR					0x1E
#define MPU6050_RA_MOT_THR					0x1F
#define MPU6050_RA_MOT_DUR					0x20
#define MPU6050_RA_ZRMOT_THR				0x21
#define MPU6050_RA_ZRMOT_DUR				0x22
#define MPU6050_RA_FIFO_EN					0x23
#define MPU6050_RA_I2C_MST_CTRL				0x24
#define MPU6050_RA_I2C_SLV0_ADDR			0x25
#define MPU6050_RA_I2C_SLV0_REG				0x26
#define MPU6050_RA_I2C_SLV0_CTRL			0x27
#define MPU6050_RA_I2C_SLV1_ADDR			0x28
#define MPU6050_RA_I2C_SLV1_REG				0x29
#define MPU6050_RA_I2C_SLV1_CTRL			0x2A
#define MPU6050_RA_I2C_SLV2_ADDR			0x2B
#define MPU6050_RA_I2C_SLV2_REG				0x2C
#define MPU6050_RA_I2C_SLV2_CTRL			0x2D
#define MPU6050_RA_I2C_SLV3_ADDR			0x2E
#define MPU6050_RA_I2C_SLV3_REG				0x2F
#define MPU6050_RA_I2C_SLV3_CTRL			0x30
#define MPU6050_RA_I2C_SLV4_ADDR			0x31
#define MPU6050_RA_I2C_SLV4_REG				0x32
#define MPU6050_RA_I2C_SLV4_DO				0x33
#define MPU6050_RA_I2C_SLV4_CTRL			0x34
#define MPU6050_RA_I2C_SLV4_DI				0x35
#define MPU6050_RA_I2C_MST_STATUS			0x36
/************************************************************************************************
*Register 55(R37h) 每 INT Pin / Bypass Enable Configuration
*Type: Read/Write
*+--------------+------------------+---------+--------+------------+------------+---------------+------------+-------------+---------+
*|Register (Hex)|Register (Decimal)|   Bit7  |  Bit6  |    Bit5    |    Bit4    |      Bit3     |    Bit2    |     Bit1    |   Bit0  |
*+--------------+------------------+---------+--------+------------+------------+---------------+------------+-------------+---------+
*|		1C	    |        28        |INT_LEVEL|INT_OPEN|LATCH_INT_EN|INT_RD_CLEAR|FSYNC_INT_LEVEL|FSYNC_INT_EN|I2C_BYPASS_EN|CLKOUT_EN|
*+--------------+------------------+---------+--------+------------+------------+---------------+------------+-------------+---------+
*Description:
*	This register configures the behavior of the interrupt signals at the INT pins. This register is also used to enable the FSYNC Pin to be used as an interrupt to the host application processor, as well as to enable Bypass Mode on the I2C Master. This bit also enables the clock output.
*	FSYNC_INT_EN enables the FSYNC pin to be used as an interrupt to the host application processor. A transition to the active level specified in FSYNC_INT_LEVEL will trigger an interrupt. The status of this interrupt is read from the PASS_THROUGH bit in the I2C Master Status Register (Register 54).
*	When I2C_BYPASS_EN is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN.
*	For further information regarding Bypass Mode, please refer to Section 7.11 and 7.13 of the MPU-6000/MPU-6050 Product Specification document.
*Parameters:
*	INT_LEVEL		When this bit is equal to 0, the logic level for the INT pin is active high.
*					When this bit is equal to 1, the logic level for the INT pin is active low.
*	INT_OPEN		When this bit is equal to 0, the INT pin is configured as push-pull.
*					When this bit is equal to 1, the INT pin is configured as open drain.
*	LATCH_INT_EN	When this bit is equal to 0, the INT pin emits a 50us long pulse.
*					When this bit is equal to 1, the INT pin is held high until the interrupt is cleared.
*	INT_RD_CLEAR	When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58)
*					When this bit is equal to 1, interrupt status bits are cleared on any read operation.
*	FSYNC_INT_LEVEL	When this bit is equal to 0, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active high.
*					When this bit is equal to 1, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active low.
*	FSYNC_INT_EN	When equal to 0, this bit disables the FSYNC pin from causing an interrupt to the host processor.
*					When equal to 1, this bit enables the FSYNC pin to be used as an interrupt to the host processor.
*	I2C_BYPASS_EN	When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0.
*					When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106 bit[5]).
*	CLKOUT_EN		When this bit is equal to 1, a reference clock output is provided at the CLKOUT pin.
*					When this bit is equal to 0, the clock output is disabled.
*					For further information regarding CLKOUT, please refer to the MPU-60X0 Product Specification document.
************************************************************************************************/
#define MPU6050_RA_INT_PIN_CFG				0x37
/************************************************************************************************
*Register 56(R38h) 每 Interrupt Enable
*Type: Read/Write
*+--------------+------------------+------+------+-------+-------------+--------------+------+------+------------+
*|Register (Hex)|Register (Decimal)| Bit7 | Bit6 |  Bit5 |     Bit4    |     Bit3     | Bit2 | Bit1 |    Bit0    |
*+--------------+------------------+------+------+-------+-------------+--------------+------+------+------------+
*|		38	    |        56        | FF_EN|MOT_EN|ZMOT_EN|FIFO_OFLOW_EN|I2C_MST_INT_EN|   -  |   -  |DATA _RDY_EN|
*+--------------+------------------+------+------+-------+-------------+--------------+------+------+------------+
*Description:
*	This register enables interrupt generation by interrupt sources.
*	For information regarding Free Fall detection, Motion detection, and Zero Motion detection, please refer to Registers 29 to 34
*	For information regarding the interrupt status for of each interrupt generation source, please refer to Register 58. Further information regarding I2C Master interrupt generation can be found in Register 54.
*	Bits 2 and 1 are reserved.
*Parameters:
*	FF_EN			When set to 1, this bit enables Free Fall detection to generate an interrupt.
*	MOT_EN			When set to 1, this bit enables Motion detection to generate an interrupt.
*	ZMOT_EN			When set to 1, this bit enables Zero Motion detection to generate an interrupt.
*	FIFO_OFLOW_EN	When set to 1, this bit enables a FIFO buffer overflow to generate an interrupt.
*	I2C_MST_INT_EN	When set to 1, this bit enables any of the I2C Master interrupt sources to generate an interrupt.
*	DATA_RDY_EN		When set to 1, this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed.
************************************************************************************************/
#define MPU6050_RA_INT_ENABLE				0x38

#define MPU6050_RA_DMP_INT_STATUS			0x39
#define MPU6050_RA_INT_STATUS				0x3A
/************************************************************************************************
*Registers 59 to 64(R3Bh - R40h) 每 Accelerometer Measurements
*Type: Read Only
*Description:
*	These registers store the most recent accelerometer measurements.
*	Accelerometer measurements are written to these registers at the Sample Rate as defined in Register 25.
*	The accelerometer measurement registers, along with the temperature measurement registers, gyroscope measurement registers, and external sensor data registers, are composed of two sets of registers: an internal register set and a user-facing read register set.
*	The data within the accelerometer sensors＊ internal register set is always updated at the Sample Rate. Meanwhile, the user-facing read register set duplicates the internal register set＊s data values whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.
*	Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS (Register 28). For each full scale setting, the accelerometers＊ sensitivity per LSB in ACCEL_xOUT is shown in the table below.
*	AFS_SEL-----Full Scale Range-----LSB Sensitivity
*		0			  ㊣2g              16384 LSB/mg 
*		1			  ㊣4g			    8192 LSB/mg
*		2			  ㊣8g				4096 LSB/mg  
*		3			  ㊣16g				2048 LSB/mg  
*Parameters:
*	ACCEL_XOUT	16-bit 2＊s complement value.
*				Stores the most recent X axis accelerometer measurement.
*	ACCEL_YOUT	16-bit 2＊s complement value.
*				Stores the most recent Y axis accelerometer measurement.
*	ACCEL_ZOUT	16-bit 2＊s complement value.
*				Stores the most recent Z axis accelerometer measurement.
************************************************************************************************/
#define MPU6050_RA_ACCEL_XOUT_H				0x3B
#define MPU6050_RA_ACCEL_XOUT_L				0x3C
#define MPU6050_RA_ACCEL_YOUT_H				0x3D
#define MPU6050_RA_ACCEL_YOUT_L				0x3E
#define MPU6050_RA_ACCEL_ZOUT_H				0x3F
#define MPU6050_RA_ACCEL_ZOUT_L				0x40
/************************************************************************************************
*Registers 65 and 66(R41h,R42h) 每 Temperature Measurement
*Type: Read Only
*Description:
*	These registers store the most recent temperature sensor measurement.
*	Temperature measurements are written to these registers at the Sample Rate as defined in Register 25.
*	These temperature measurement registers, along with the accelerometer measurement registers, gyroscope measurement registers, and external sensor data registers, are composed of two sets of registers: an internal register set and a user-facing read register set.
*	The data within the temperature sensor＊s internal register set is always updated at the Sample Rate. Meanwhile, the user-facing read register set duplicates the internal register set＊s data values whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.
*	The scale factor and offset for the temperature sensor are found in the Electrical Specifications table (Section 6.4 of the MPU-6000/MPU-6050 Product Specification document).
*Parameters:
*	TEMP_OUT	16-bit signed value.
*				Stores the most recent temperature sensor measurement.
************************************************************************************************/
#define MPU6050_RA_TEMP_OUT_H				0x41
#define MPU6050_RA_TEMP_OUT_L				0x42
/************************************************************************************************
*Registers 67 to 72(R43h - R48h) 每 Gyroscope Measurements
*Type: Read Only
*Description:
*	These registers store the most recent gyroscope measurements.
*	Gyroscope measurements are written to these registers at the Sample Rate as defined in Register 25.
*	These gyroscope measurement registers, along with the accelerometer measurement registers, temperature measurement registers, and external sensor data registers, are composed of two sets of registers: an internal register set and a user-facing read register set.
*	The data within the gyroscope sensors＊ internal register set is always updated at the Sample Rate. Meanwhile, the user-facing read register set duplicates the internal register set＊s data values whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.
*	Each 16-bit gyroscope measurement has a full scale defined in FS_SEL (Register 27). For each full scale setting, the gyroscopes＊ sensitivity per LSB in GYRO_xOUT is shown in the table below:
*	FS_SEL-----Full Scale Range-----LSB Sensitivity
*     0	  		㊣ 250 ～/s	   	   	  131 LSB/～/s
*	  1			㊣ 500 ～/s   		 65.5 LSB/～/s  
*	  2		   ㊣ 1000 ～/s  	         32.8 LSB/～/s
*	  3		   ㊣ 2000 ～/s            16.4 LSB/～/s 
*Parameters:
*	GYRO_XOUT	16-bit 2＊s complement value.
*				Stores the most recent X axis gyroscope measurement.
*	GYRO_YOUT	16-bit 2＊s complement value.
*				Stores the most recent Y axis gyroscope measurement.
*	GYRO_ZOUT	16-bit 2＊s complement value.
*				Stores the most recent Z axis gyroscope measurement.
************************************************************************************************/
#define MPU6050_RA_GYRO_XOUT_H				0x43
#define MPU6050_RA_GYRO_XOUT_L				0x44
#define MPU6050_RA_GYRO_YOUT_H				0x45
#define MPU6050_RA_GYRO_YOUT_L				0x46
#define MPU6050_RA_GYRO_ZOUT_H				0x47
#define MPU6050_RA_GYRO_ZOUT_L				0x48
/************************************************************************************************
*Registers 73 to 96(R49h - R60h) 每 External Sensor Data
*Type: Read Only
*Description:
*	These registers store data read from external sensors by the Slave 0, 1, 2, and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in I2C_SLV4_DI (Register 53).
*	External sensor data is written to these registers at the Sample Rate as defined in Register 25. This access rate can be reduced by using the Slave Delay Enable registers (Register 103).
*	External sensor data registers, along with the gyroscope measurement registers, accelerometer measurement registers, and temperature measurement registers, are composed of two sets of registers: an internal register set and a user-facing read register set.
*	The data within the external sensors＊ internal register set is always updated at the Sample Rate (or the reduced access rate) whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.
*	Data is placed in these external sensor data registers according to I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39, 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as defined in Register 25) or delayed rate (if specified in Register 52 and 103). During each Sample cycle, slave reads are performed in order of Slave number. If all slaves are enabled with more than zero bytes to be read, the order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
*	Each enabled slave will have EXT_SENS_DATA registers associated with it by number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may change the higher numbered slaves＊ associated registers. Furthermore, if fewer total bytes are being read from the external sensors as a result of such a change, then the data remaining in the registers which no longer have an associated slave device (i.e. high numbered registers) will remain in these previously allocated registers unless reset.
*	If the sum of the read lengths of all SLVx transactions exceed the number of available EXT_SENS_DATA registers , the excess bytes will be dropped. There are 24 EXT_SENS_DATA registers and hence the total read lengths between all the slaves cannot be greater than 24 or some bytes will be lost.
*Note: Slave 4＊s behavior is distinct from that of Slaves 0-3. For further information regarding the characteristics of Slave 4, please refer to Registers 49 to 53.
*Example:
*	Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read, (I2C_SLV1_EN=1 and I2C_SLV1_LEN = 2). In such a situation, EXT_SENS_DATA _00 through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05 will be associated with Slave 1.
*	If Slave 2 is enabled as well, registers starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
*	If Slave 2 is disabled while Slave 3 is enabled in this same situation, then registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3 instead.
*Register Allocation for Dynamic Disable vs. Normal Disable
*	If a slave is disabled at any time, the space initially allocated to the slave in the EXT_SENS_DATA register, will remain associated with that slave. This is to avoid dynamic adjustment of the register allocation.
*	The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106) .
*	This above is also true if one of the slaves gets NACKed and stops functioning.
************************************************************************************************/
#define MPU6050_RA_EXT_SENS_DATA_00			0x49
#define MPU6050_RA_EXT_SENS_DATA_01			0x4A
#define MPU6050_RA_EXT_SENS_DATA_02			0x4B
#define MPU6050_RA_EXT_SENS_DATA_03			0x4C
#define MPU6050_RA_EXT_SENS_DATA_04			0x4D
#define MPU6050_RA_EXT_SENS_DATA_05			0x4E
#define MPU6050_RA_EXT_SENS_DATA_06			0x4F
#define MPU6050_RA_EXT_SENS_DATA_07			0x50
#define MPU6050_RA_EXT_SENS_DATA_08			0x51
#define MPU6050_RA_EXT_SENS_DATA_09			0x52
#define MPU6050_RA_EXT_SENS_DATA_10			0x53
#define MPU6050_RA_EXT_SENS_DATA_11			0x54
#define MPU6050_RA_EXT_SENS_DATA_12			0x55
#define MPU6050_RA_EXT_SENS_DATA_13			0x56
#define MPU6050_RA_EXT_SENS_DATA_14			0x57
#define MPU6050_RA_EXT_SENS_DATA_15			0x58
#define MPU6050_RA_EXT_SENS_DATA_16			0x59
#define MPU6050_RA_EXT_SENS_DATA_17			0x5A
#define MPU6050_RA_EXT_SENS_DATA_18			0x5B
#define MPU6050_RA_EXT_SENS_DATA_19			0x5C
#define MPU6050_RA_EXT_SENS_DATA_20			0x5D
#define MPU6050_RA_EXT_SENS_DATA_21			0x5E
#define MPU6050_RA_EXT_SENS_DATA_22			0x5F
#define MPU6050_RA_EXT_SENS_DATA_23			0x60

#define MPU6050_RA_MOT_DETECT_STATUS		0x61
#define MPU6050_RA_I2C_SLV0_DO				0x63
#define MPU6050_RA_I2C_SLV1_DO				0x64
#define MPU6050_RA_I2C_SLV2_DO				0x65
#define MPU6050_RA_I2C_SLV3_DO				0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL		0x67
#define MPU6050_RA_SIGNAL_PATH_RESET		0x68
/************************************************************************************************
*Register 105 每 Motion Detection Control
*Type: Read/Write
*+--------------+------------------+------+------+---------+---------+------+------+------+-------+
*|Register (Hex)|Register (Decimal)| Bit7 | Bit6 |   Bit5  |   Bit4  | Bit3 | Bit2 | Bit1 |  Bit0 |
*+--------------+------------------+------+------+---------+---------+------+------+------+-------+
*|		1C	    |        28        |   -  |   -  |ACCEL_ON_DELAY[1:0]|FF_COUNT[1:0]|MOT_COUNT[1:0]|
*+--------------+------------------+------+------+-------------------+-------------+--------------+
*Description:
*	This register is used to add delay to the accelerometer power on time. It is also used to configure the Free Fall and Motion detection decrement rate.
*	The accelerometer data path provides samples to the sensor registers, Motion detection, Zero Motion detection, and Free Fall detection modules. The signal path contains filters which must be flushed on wake-up with new samples before the detection modules begin operations. The default wake-up delay, of 4ms can be lengthened by up to 3ms. This additional delay is specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select any value above zero unless instructed otherwise by InvenSense. Please refer to Section 8 of the MPU-6000/MPU-6050 Product Specification document for further information regarding the detection modules.
*	Detection is registered by the Free Fall detection module or the Motion detection module after accelerometer measurements meet their respective threshold conditions over a specified number of samples. When the threshold conditions are met, the corresponding detection counter increments by 1. The user may control the rate at which the detection counter decrements when the threshold condition is not met by configuring FF_COUNT and MOT_COUNT. The decrement rate can be set according to the following table:
*	FF_COUNT or MOT_COUNT-----Counter Decrement
*			 0						Reset
*			 1						  1
*			 2						  2
*			 3						  4
*	When FF_COUNT or MOT_COUNT are configured to 0 (reset), any non-qualifying sample will reset the corresponding counter to 0. For further information on Free Fall detection and Motion detection, please refer to Registers 29 to 32.
*	Bits 7 and 6 are reserved.
*Parameters:
*	ACCEL_ON_DELAY	2-bit unsigned value. Specifies the additional power-on delay applied to accelerometer data path modules.
*					Unit of 1 LSB = 1 ms.
*	FF_COUNT		2-bit unsigned value. Configures the Free Fall detection counter decrement rate.
*	MOT_COUNT		2-bit unsigned value. Configures the Motion detection counter decrement rate.
************************************************************************************************/
#define MPU6050_RA_MOT_DETECT_CTRL			0x69
#define MPU6050_RA_USER_CTRL				0x6A
/************************************************************************************************
*Register 107(R6Bh) 每 Power Management 1
*Type: Read/Write
*+--------------+------------------+------------+------+------+------+--------+------+------+------+
*|Register (Hex)|Register (Decimal)|    Bit7    | Bit6 | Bit5 | Bit4 |  Bit3  | Bit2 | Bit1 | Bit0 |
*+--------------+------------------+------------+------+------+------+--------+------+------+------+
*|		6B	    |        107       |DEVICE_RESET| SLEEP| CYCLE|   -  |TEMP_DIS|     CLKSEL[2:0]    |
*+--------------+------------------+------------+------+------+------+--------+--------------------+
*Description:
*	This register allows the user to configure the power mode and clock source. It also provides a bit for resetting the entire device, and a bit for disabling the temperature sensor.
*	By setting SLEEP to 1, the MPU-60X0 can be put into low power sleep mode. When CYCLE is set to 1 while SLEEP is disabled, the MPU-60X0 will be put into Cycle Mode. In Cycle Mode, the device cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL (register 108). To configure the wake frequency, use LP_WAKE_CTRL within the Power Management 2 register (Register 108).
*	An internal 8MHz oscillator, gyroscope based clock, or external sources can be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator or an external source is chosen as the clock source, the MPU-60X0 can operate in low power modes with the gyroscopes disabled.
*	Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source) as the clock reference for improved stability. The clock source can be selected according to the following table.
*		CLKSEL--------------------Clock Source
*         0              Internal 8MHz oscillator
*         1				 PLL with X axis gyroscope reference
*         2              PLL with Y axis gyroscope reference
*         3              PLL with Z axis gyroscope reference
*         4              PLL with external 32.768kHz reference
*         5              PLL with external 19.2MHz reference
*         6              Reserved
*         7              Stops the clock and keeps the timing generator in reset
*	For further information regarding the MPU-60X0 clock source, please refer to the MPU-6000/MPU-6050 Product Specification document.
*	Bit 4 is reserved.
*Parameters:
*	DEVICE_RESET	When set to 1, this bit resets all internal registers to their default values.
*					The bit automatically clears to 0 once the reset is done.
*					The default values for each register can be found in Section 3.
*	SLEEP			When set to 1, this bit puts the MPU-60X0 into sleep mode.
*	CYCLE			When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL (register 108).
*	TEMP_DIS		When set to 1, this bit disables the temperature sensor.
*	CLKSEL			3-bit unsigned value. Specifies the clock source of the device.
************************************************************************************************/
#define MPU6050_RA_PWR_MGMT_1				0x6B
/************************************************************************************************
*Register 108(R6Ch) 每 Power Management 2
*Type: Read/Write
*+--------------+------------------+--------+--------+-------+-------+-------+-------+-------+-------+
*|Register (Hex)|Register (Decimal)|  Bit7  |  Bit6  |  Bit5 |  Bit4 |  Bit3 |  Bit2 |  Bit1 |  Bit0 |
*+--------------+------------------+--------+--------+-------+-------+-------+-------+-------+-------+
*|		6C	    |        108       |LP_WAKE_CTRL[1:0]|STBY_XA|STBY_YA|STBY_ZA|STBY_XG|STBY_YG|STBY_ZG|
*+--------------+------------------+-----------------+-------+-------+-------+-------+-------+-------+
*Description:
*	This register allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode. This register also allows the user to put individual axes of the accelerometer and gyroscope into standby mode.
*	The MPU-9150 can be put into Accerlerometer Only Low Power Mode by setting PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode, the device will power off all devices except for the primary I2C interface, waking only the accelerometer at fixed intervals to take a single measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL as shown below.
*	LP_WAKE_CTRL-----Wake-up Frequency
*		  0				   1.25 Hz
*		  1				    2.5 Hz
*		  2					  5 Hz
*		  3					 10 Hz 
*	For further information regarding the MPU-9150＊s power modes, please refer to Register 107.
*	The user can put individual accelerometer and gyroscopes axes into standby mode by using this register. If the device is using a gyroscope axis as the clock source and this axis is put into standby mode, the clock source will automatically be changed to the internal 8MHz oscillator.
*Parameters:
*	LP_WAKE_CTRL	2-bit unsigned value.
*					Specifies the frequency of wake-ups during Accelerometer Only Low Power Mode.
*	STBY_XA			When set to 1, this bit puts the X axis accelerometer into standby mode.
*	STBY_YA			When set to 1, this bit puts the Y axis accelerometer into standby mode.
*	STBY_ZA			When set to 1, this bit puts the Z axis accelerometer into standby mode.
*	STBY_XG			When set to 1, this bit puts the X axis gyroscope into standby mode.
*	STBY_YG			When set to 1, this bit puts the Y axis gyroscope into standby mode.
*	STBY_ZG			When set to 1, this bit puts the Z axis gyroscope into standby mode.
************************************************************************************************/
#define MPU6050_RA_PWR_MGMT_2				0x6C
#define MPU6050_RA_BANK_SEL					0x6D
#define MPU6050_RA_MEM_START_ADDR			0x6E
#define MPU6050_RA_MEM_R_W					0x6F
#define MPU6050_RA_DMP_CFG_1				0x70
#define MPU6050_RA_DMP_CFG_2				0x71
#define MPU6050_RA_FIFO_COUNTH				0x72
#define MPU6050_RA_FIFO_COUNTL				0x73
#define MPU6050_RA_FIFO_R_W					0x74
/************************************************************************************************
*Register 117(R75h) 每 Who Am I
*Type: Read Only
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|Register (Hex)|Register (Decimal)| Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
*+--------------+------------------+------+------+------+------+------+------+------+------+
*|		75	    |        117       |   -  |              WHO_AM_I[6:1]              |   -  |
*+--------------+------------------+------+------+------+-------------+--------------------+
*Description:
*	This register is used to verify the identity of the device. The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0＊s 7-bit I2C address. The least significant bit of the MPU-60X0＊s I2C address is determined by the value of the AD0 pin. The value of the AD0 pin is not reflected in this register.
*	The default value of the register is 0x68.
*	Bits 0 and 7 are reserved. (Hard coded to 0)
*Parameters:
*	WHO_AM_I	Contains the 6-bit I2C address of the MPU-60X0.
*				The Power-On-Reset value of Bit6:Bit1 is 110 100.
************************************************************************************************/
#define MPU6050_RA_WHO_AM_I					0x75

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define MPU6050_RA_DATA_START_ADDR			0x3B

#define MPU6050_TC_PWR_MODE_BIT				7
#define MPU6050_TC_OFFSET_BIT				6
#define MPU6050_TC_OFFSET_LENGTH			6
#define MPU6050_TC_OTP_BNK_VLD_BIT			0

#define MPU6050_VDDIO_LEVEL_VLOGIC			0
#define MPU6050_VDDIO_LEVEL_VDD				1

#define MPU6050_CFG_EXT_SYNC_SET_BIT		5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH		3
#define MPU6050_CFG_DLPF_CFG_BIT			2
#define MPU6050_CFG_DLPF_CFG_LENGTH			3
/*--------------------Configuration (CONFIG)--------------------*/
#define MPU6050_EXT_SYNC_DISABLED			(0x00<<3)
#define MPU6050_EXT_SYNC_TEMP_OUT_L			(0x01<<3)
#define MPU6050_EXT_SYNC_GYRO_XOUT_L		(0x02<<3)
#define MPU6050_EXT_SYNC_GYRO_YOUT_L		(0x03<<3)
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L		(0x04<<3)
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L		(0x05<<3)
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L		(0x06<<3)
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L		(0x07<<3)
#define MPU6050_DLPF_BW_256					0x00
#define MPU6050_DLPF_BW_188					0x01
#define MPU6050_DLPF_BW_98					0x02
#define MPU6050_DLPF_BW_42					0x03
#define MPU6050_DLPF_BW_20					0x04
#define MPU6050_DLPF_BW_10					0x05
#define MPU6050_DLPF_BW_5					0x06

#define MPU6050_GCONFIG_FS_SEL_BIT			4
#define MPU6050_GCONFIG_FS_SEL_LENGTH		2
/*--------------------Gyroscope Configuration (GYRO_CONFIG)--------------------*/
#define MPU6050_GYRO_FS_250					(0x00<<3)
#define MPU6050_GYRO_FS_500					(0x01<<3)
#define MPU6050_GYRO_FS_1000				(0x02<<3)
#define MPU6050_GYRO_FS_2000				(0x03<<3)

#define MPU6050_ACONFIG_XA_ST_BIT			7
#define MPU6050_ACONFIG_YA_ST_BIT			6
#define MPU6050_ACONFIG_ZA_ST_BIT			5
#define MPU6050_ACONFIG_AFS_SEL_BIT			4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH		2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT		2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH	3
/*--------------------Accelerometer Configuration (ACCEL_CONFIG)--------------------*/
#define MPU6050_ACCEL_FS_2					(0x00<<3)
#define MPU6050_ACCEL_FS_4					(0x01<<3)
#define MPU6050_ACCEL_FS_8					(0x02<<3)
#define MPU6050_ACCEL_FS_16					(0x03<<3)
#define MPU6050_DHPF_RESET					0x00
#define MPU6050_DHPF_5						0x01
#define MPU6050_DHPF_2P5					0x02
#define MPU6050_DHPF_1P25					0x03
#define MPU6050_DHPF_0P63					0x04
#define MPU6050_DHPF_HOLD					0x07

#define MPU6050_TEMP_FIFO_EN_BIT			7
#define MPU6050_XG_FIFO_EN_BIT				6
#define MPU6050_YG_FIFO_EN_BIT				5
#define MPU6050_ZG_FIFO_EN_BIT				4
#define MPU6050_ACCEL_FIFO_EN_BIT			3
#define MPU6050_SLV2_FIFO_EN_BIT			2
#define MPU6050_SLV1_FIFO_EN_BIT			1
#define MPU6050_SLV0_FIFO_EN_BIT			0

#define MPU6050_MULT_MST_EN_BIT				7
#define MPU6050_WAIT_FOR_ES_BIT				6
#define MPU6050_SLV_3_FIFO_EN_BIT			5
#define MPU6050_I2C_MST_P_NSR_BIT			4
#define MPU6050_I2C_MST_CLK_BIT				3
#define MPU6050_I2C_MST_CLK_LENGTH			4

#define MPU6050_CLOCK_DIV_348				0x0
#define MPU6050_CLOCK_DIV_333				0x1
#define MPU6050_CLOCK_DIV_320				0x2
#define MPU6050_CLOCK_DIV_308				0x3
#define MPU6050_CLOCK_DIV_296				0x4
#define MPU6050_CLOCK_DIV_286				0x5
#define MPU6050_CLOCK_DIV_276				0x6
#define MPU6050_CLOCK_DIV_267				0x7
#define MPU6050_CLOCK_DIV_258				0x8
#define MPU6050_CLOCK_DIV_500				0x9
#define MPU6050_CLOCK_DIV_471				0xA
#define MPU6050_CLOCK_DIV_444				0xB
#define MPU6050_CLOCK_DIV_421				0xC
#define MPU6050_CLOCK_DIV_400				0xD
#define MPU6050_CLOCK_DIV_381				0xE
#define MPU6050_CLOCK_DIV_364				0xF

#define MPU6050_I2C_SLV_RW_BIT				7
#define MPU6050_I2C_SLV_ADDR_BIT			6
#define MPU6050_I2C_SLV_ADDR_LENGTH			7
#define MPU6050_I2C_SLV_EN_BIT				7
#define MPU6050_I2C_SLV_BYTE_SW_BIT			6
#define MPU6050_I2C_SLV_REG_DIS_BIT			5
#define MPU6050_I2C_SLV_GRP_BIT				4
#define MPU6050_I2C_SLV_LEN_BIT				3
#define MPU6050_I2C_SLV_LEN_LENGTH			4

#define MPU6050_I2C_SLV4_RW_BIT				7
#define MPU6050_I2C_SLV4_ADDR_BIT			6
#define MPU6050_I2C_SLV4_ADDR_LENGTH		7
#define MPU6050_I2C_SLV4_EN_BIT				7
#define MPU6050_I2C_SLV4_INT_EN_BIT			6
#define MPU6050_I2C_SLV4_REG_DIS_BIT		5
#define MPU6050_I2C_SLV4_MST_DLY_BIT		4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH		5

#define MPU6050_MST_PASS_THROUGH_BIT		7
#define MPU6050_MST_I2C_SLV4_DONE_BIT		6
#define MPU6050_MST_I2C_LOST_ARB_BIT		5
#define MPU6050_MST_I2C_SLV4_NACK_BIT		4
#define MPU6050_MST_I2C_SLV3_NACK_BIT		3
#define MPU6050_MST_I2C_SLV2_NACK_BIT		2
#define MPU6050_MST_I2C_SLV1_NACK_BIT		1
#define MPU6050_MST_I2C_SLV0_NACK_BIT		0

#define MPU6050_INTCFG_INT_LEVEL_BIT		7
#define MPU6050_INTCFG_INT_OPEN_BIT			6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT		5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT		4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT	3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT		2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT	1
#define MPU6050_INTCFG_CLKOUT_EN_BIT		0
/*--------------------INT Pin / Bypass Enable Configuration (INT_PIN_CFG)--------------------*/
#define MPU6050_INTMODE_ACTIVEHIGH			(0x00<<7)
#define MPU6050_INTMODE_ACTIVELOW			(0x01<<7)
#define MPU6050_INTDRV_PUSHPULL				(0x00<<6)
#define MPU6050_INTDRV_OPENDRAIN			(0x01<<6)
#define MPU6050_INTLATCH_50USPULSE			(0x00<<5)
#define MPU6050_INTLATCH_WAITCLEAR			(0x01<<5)
#define MPU6050_INTCLEAR_STATUSREAD			(0x00<<4)
#define MPU6050_INTCLEAR_ANYREAD			(0x01<<4)

/*--------------------Interrupt Enable (INT_ENABLE)--------------------*/
#define MPU6050_INTERRUPT_DATA_RDY_ENABLE	0x01
#define MPU6050_INTERRUPT_DATA_RDY_DISABLE	0x00

#define MPU6050_INTERRUPT_FF_BIT			7
#define MPU6050_INTERRUPT_MOT_BIT			6
#define MPU6050_INTERRUPT_ZMOT_BIT			5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT	4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT	3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT	2
#define MPU6050_INTERRUPT_DMP_INT_BIT		1
#define MPU6050_INTERRUPT_DATA_RDY_BIT		0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT				5
#define MPU6050_DMPINT_4_BIT				4
#define MPU6050_DMPINT_3_BIT				3
#define MPU6050_DMPINT_2_BIT				2
#define MPU6050_DMPINT_1_BIT				1
#define MPU6050_DMPINT_0_BIT				0

#define MPU6050_MOTION_MOT_XNEG_BIT			7
#define MPU6050_MOTION_MOT_XPOS_BIT			6
#define MPU6050_MOTION_MOT_YNEG_BIT			5
#define MPU6050_MOTION_MOT_YPOS_BIT			4
#define MPU6050_MOTION_MOT_ZNEG_BIT			3
#define MPU6050_MOTION_MOT_ZPOS_BIT			2
#define MPU6050_MOTION_MOT_ZRMOT_BIT		0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT	7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT	4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT	3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT	2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT	1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT	0

#define MPU6050_PATHRESET_GYRO_RESET_BIT	2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT	1
#define MPU6050_PATHRESET_TEMP_RESET_BIT	0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT		5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH	2
#define MPU6050_DETECT_FF_COUNT_BIT				3
#define MPU6050_DETECT_FF_COUNT_LENGTH			2
#define MPU6050_DETECT_MOT_COUNT_BIT			1
#define MPU6050_DETECT_MOT_COUNT_LENGTH			2
/*--------------------Motion Detection Control (MOT_DETECT_CTRL)--------------------*/
#define MPU6050_DETECT_FF_COUNT_RESET		(0x00<<2)
#define MPU6050_DETECT_FF_COUNT_1			(0x01<<2)
#define MPU6050_DETECT_FF_COUNT_2			(0x02<<2)
#define MPU6050_DETECT_FF_COUNT_4			(0x03<<2)
#define MPU6050_DETECT_MOT_COUNT_RESET		0x00
#define MPU6050_DETECT_MOT_COUNT_1			0x01
#define MPU6050_DETECT_MOT_COUNT_2			0x02
#define MPU6050_DETECT_MOT_COUNT_4			0x03

#define MPU6050_USERCTRL_DMP_EN_BIT			7
#define MPU6050_USERCTRL_FIFO_EN_BIT		6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT		5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT		4
#define MPU6050_USERCTRL_DMP_RESET_BIT		3
#define MPU6050_USERCTRL_FIFO_RESET_BIT		2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT	1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT	0

#define MPU6050_PWR1_DEVICE_RESET_BIT		7
#define MPU6050_PWR1_SLEEP_BIT				6
#define MPU6050_PWR1_CYCLE_BIT				5
#define MPU6050_PWR1_TEMP_DIS_BIT			3
#define MPU6050_PWR1_CLKSEL_BIT				2
#define MPU6050_PWR1_CLKSEL_LENGTH			3
/*--------------------Power Management 1 (PWR_MGMT_1)--------------------*/
#define MPU6050_PWR1_DEVICE_RESET			(0x01<<7)
#define MPU6050_PWR1_DEVICE_SET				(0x00<<7)
#define MPU6050_PWR1_SLEEP_ENABLE			(0x01<<6)
#define MPU6050_PWR1_SLEEP_DISABLE			(0x00<<6)
#define MPU6050_CLOCK_INTERNAL				0x00
#define MPU6050_CLOCK_PLL_XGYRO				0x01
#define MPU6050_CLOCK_PLL_YGYRO				0x02
#define MPU6050_CLOCK_PLL_ZGYRO				0x03
#define MPU6050_CLOCK_PLL_EXT32K			0x04
#define MPU6050_CLOCK_PLL_EXT19M			0x05
#define MPU6050_CLOCK_KEEP_RESET			0x07
/*--------------------Power Management 2 (PWR_MGMT_2)--------------------*/
#define MPU6050_PWR2_LP_WAKE_CTRL_BIT		7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH	2
#define MPU6050_PWR2_STBY_XA				(0x01<<5)
#define MPU6050_PWR2_STBY_YA				(0x01<<4)
#define MPU6050_PWR2_STBY_ZA				(0x01<<3)
#define MPU6050_PWR2_STBY_XG				(0x01<<2)
#define MPU6050_PWR2_STBY_YG				(0x01<<1)
#define MPU6050_PWR2_STBY_ZG				0x01
#define MPU6050_PWR2_STBY_None				0x00
#define MPU6050_WAKE_FREQ_1P25				(0x00<<6)
#define MPU6050_WAKE_FREQ_2P5				(0x01<<6)
#define MPU6050_WAKE_FREQ_5					(0x02<<6)
#define MPU6050_WAKE_FREQ_10				(0x03<<6)

#define MPU6050_BANKSEL_PRFTCH_EN_BIT		6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT	5
#define MPU6050_BANKSEL_MEM_SEL_BIT			4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH		5

#define MPU6050_WHO_AM_I_BIT				6
#define MPU6050_WHO_AM_I_LENGTH				6

void MPU6050_Init(MPU6050_InitTypeDef *MPU6050_InitStruct);
void MPU6050_INT_Init(MPU6050_INTTypeDef *MPU6050_INTStruct);
void MPU6050_Reset(FlagStatus Status);
uint8_t MPU6050_GetID(void);
int16_t MPU6050_GetAcceX(void);
int16_t MPU6050_GetAcceY(void);
int16_t MPU6050_GetAcceZ(void);
void MPU6050_GetAcce(AcceDef *Acce, AcceDef *AcceOffset);
int16_t MPU6050_GetGyroX(void);
int16_t MPU6050_GetGyroY(void);
int16_t MPU6050_GetGyroZ(void);
void MPU6050_GetGyro(GyroDef *Gyro, GyroDef *GyroOffset);
void MPU6050_Correct(AcceDef *AcceOffset, GyroDef *GyroOffset);
bool I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue);
uint8_t I2C_WriteDeviceRegisterLength(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Length, uint8_t *RegisterValue);
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr);
uint8_t I2C_ReadDeviceRegisterLength(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Length, uint8_t *RegisterValue);
int16_t I2C_ReadDataBuffer(uint8_t DeviceAddr, uint32_t RegisterAddr);

#endif /* __MPU6050_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
