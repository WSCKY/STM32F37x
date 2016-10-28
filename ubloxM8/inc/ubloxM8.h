/**
  ******************************************************************************
  * @file    ubloxM8.h 
  * @author  '^_^'
  * @version V0.0.0
  * @date    20-July-2015
  * @brief   Header for ubloxM8.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UBLOXM8_H
#define __UBLOXM8_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"

#include "Delay.h"
#include "UART_IT.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t SYNCHAR1, SYNCHAR2;
	uint8_t CLASS, ID;
	uint16_t LENGTH;
	uint8_t* pPayload;
	uint8_t CK_A, CK_B;
}UBXStrcutureDef;

typedef struct
{
	uint8_t inProtocol;
	uint8_t outProtocol;
	uint32_t Baudrate;
	uint8_t CharLen;
	uint8_t Parity;
	uint8_t StopBits;
}UBXProtoUARTConfigDef;

typedef enum {Reset = 0, Set = !RESET} UBXFlag;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define ubloxUARTSend					UARTSendByte
#define ubloxRecDataHandler				MyUART_IRQHandler
#define ubloxDelay						Delay_ms
/* Exported Variables --------------------------------------------------------*/
extern UBXStrcutureDef UBXDataRec;
extern UBXFlag DataOK;
/* Exported definitions ------------------------------------------------------*/
/* ----- ublox UBX Protocol synchronous character ----- */
#define UBX_SYNCHAR1					0xB5
#define UBX_SYNCHAR2					0x62

/* --------- ublox UBX Protocol Message Class --------- */
/**
A class is a grouping of messages which are related to each other. 
The following table lists all the current message classes.
*/
/* Navigation Results: Position, Speed, Time, Acceleration, Heading, DOP, SVs used */
#define UBX_CLASS_NAV					0x01
/* Receiver Manager Messages: Satellite Status, RTC Status */
#define UBX_CLASS_RXM					0x02
/* Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice */
#define UBX_CLASS_INF					0x04
/* Ack/Nack Messages: as replies to CFG Input Messages */
#define UBX_CLASS_ACK					0x05
/* Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc. */
#define UBX_CLASS_CFG					0x06
/* Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc. */
#define UBX_CLASS_UPD					0x09
/* Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status */
#define UBX_CLASS_MON					0x0A
/* AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input */
#define UBX_CLASS_AID					0x0B
/* Timing Messages: Time Pulse Output, Timemark Results */
#define UBX_CLASS_TIM					0x0D
/* Multi-GNSS Assistance: Assistance data for various GNSS */
#define UBX_CLASS_MGA					0x13
/* Logging Messages: Log creation, deletion, info and retrieval */
#define UBX_CLASS_LOG					0x21

/* ---------- ublox UBX Protocol Message ID ----------- */
/* Class: UBX_CLASS_ACK(0x05) */
#define UBX_ACK_ACK						0x01
#define UBX_ACK_NAK						0x00
/* Class: UBX_CLASS_AID(0x0B) */
#define UBX_AID_ALM						0x30
#define UBX_AID_AOP						0x33
#define UBX_AID_EPH						0x31
#define UBX_AID_HUI						0x02
#define UBX_AID_INI						0x01
/* Class: UBX_CLASS_CFG(0x06) */
#define UBX_CFG_ANT						0x13
#define UBX_CFG_CFG						0x09
#define UBX_CFG_DAT						0x06
#define UBX_CFG_DOSC					0x61
#define UBX_CFG_ESRC					0x60
#define UBX_CFG_GNSS					0x3E
#define UBX_CFG_INF						0x02
#define UBX_CFG_ITFM					0x39
#define UBX_CFG_LOGFILTER				0x47
#define UBX_CFG_MSG						0x01
#define UBX_CFG_NAV5					0x24
#define UBX_CFG_NAVX5					0x23
#define UBX_CFG_NMEA					0x17
#define UBX_CFG_ODO						0x1E
#define UBX_CFG_PM2						0x3B
#define UBX_CFG_PRT						0x00
#define UBX_CFG_PWR						0x57
#define UBX_CFG_RATE					0x08
#define UBX_CFG_RINV					0x34
#define UBX_CFG_RST						0x04
#define UBX_CFG_RXM						0x11
#define UBX_CFG_SBAS					0x16
#define UBX_CFG_SMGR					0x62
#define UBX_CFG_TMODE2					0x3D
#define UBX_CFG_TP5						0x31
#define UBX_CFG_TXSLOT					0x53
#define UBX_CFG_USB						0x1B
/* Class: UBX_CLASS_INF(0x04) */
#define UBX_INF_DEBUG					0x04
#define UBX_INF_ERROR					0x00
#define UBX_INF_NOTICE					0x02
#define UBX_INF_TEST					0x03
#define UBX_INF_WARNING					0x01
/* Class: UBX_CLASS_LOG(0x21) */
#define UBX_LOG_CREATE					0x07
#define UBX_LOG_ERASE					0x03
#define UBX_LOG_FINDTIME				0x0E
#define UBX_LOG_INFO					0x08
#define UBX_LOG_RETRIEVEPOSEXTRA		0x0F
#define UBX_LOG_RETRIEVEPOS				0x0B
#define UBX_LOG_RETRIEVESTRING			0x0D
#define UBX_LOG_RETRIEVE				0x09
#define UBX_LOG_STRING					0x04
/* Class: UBX_CLASS_MGA(0x13) */
#define UBX_MGA_ACK						0x60
#define UBX_MGA_ANO						0x20
#define UBX_MGA_DBD						0x80
#define UBX_MGA_FLASH					0x21
#define UBX_MGA_GLO						0x06
#define UBX_MGA_GPS						0x00
#define UBX_MGA_INI						0x40
#define UBX_MGA_QZSS					0x05
/* Class: UBX_CLASS_MON(0x0A) */
#define UBX_MON_GNSS					0x28
#define UBX_MON_HW2						0x0B
#define UBX_MON_HW						0x09
#define UBX_MON_IO						0x02
#define UBX_MON_MSGPP					0x06
#define UBX_MON_PATCH					0x27
#define UBX_MON_RXBUF					0x07
#define UBX_MON_RXR						0x21
#define UBX_MON_SMGR					0x2E
#define UBX_MON_TXBUF					0x08
#define UBX_MON_VER						0x04
/* Class: UBX_CLASS_NAV(0x01) */
#define UBX_NAV_AOPSTATUS				0x60
#define UBX_NAV_CLOCK					0x22
#define UBX_NAV_DGPS					0x31
#define UBX_NAV_DOP						0x04
#define UBX_NAV_ODO						0x09
#define UBX_NAV_ORB						0x34
#define UBX_NAV_POSECEF					0x01
#define UBX_NAV_POSLLH					0x02
#define UBX_NAV_PVT						0x07
#define UBX_NAV_RESETODO				0x10
#define UBX_NAV_SAT						0x35
#define UBX_NAV_SBAS					0x32
#define UBX_NAV_SOL						0x06
#define UBX_NAV_STATUS					0x03
#define UBX_NAV_SVINFO					0x30
#define UBX_NAV_TIMEBDS					0x24
#define UBX_NAV_TIMEGLO					0x23
#define UBX_NAV_TIMEGPS					0x20
#define UBX_NAV_TIMEUTC					0x21
#define UBX_NAV_VELECEF					0x11
#define UBX_NAV_VELNED					0x12
/* Class: UBX_CLASS_RXM(0x02) */
#define UBX_RXM_PMREQ					0x41
#define UBX_RXM_RAWX					0x15
#define UBX_RXM_SFRBX					0x13
#define UBX_RXM_SVSI					0x20
/* Class: UBX_CLASS_TIM(0x0D) */
#define UBX_TIM_DOSC					0x11
#define UBX_TIM_FCHG					0x16
#define UBX_TIM_HOC						0x17
#define UBX_TIM_SMEAS					0x13
#define UBX_TIM_SVIN					0x04
#define UBX_TIM_TM2						0x03
#define UBX_TIM_TOS						0x12
#define UBX_TIM_TP						0x01
#define UBX_TIM_VCOCAL					0x15
#define UBX_TIM_VRFY					0x06
/* Class: UBX_CLASS_UPD(0x09) */
#define UBX_UPD_SOS						0x14

/* ----------------- GNSS Identifiers ----------------- */
#define GNSS_GPS						0x00
#define GNSS_SBAS						0x01
#define GNSS_Galileo					0x02
#define GNSS_BeiDou						0x03
#define GNSS_IMES						0x04
#define GNSS_QZSS						0x05
#define GNSS_GLONASS					0x06

/* -------------- Port Number assignment -------------- */
#define Port_DDC_I2C					0x00
#define Port_UART1						0x01
#define Port_USB						0x03
#define Port_SPI						0x04

/******************************************************************************/
/* Bitfield mode */
#define UARTCharLen_5bit				(0x00 << 6)/* not supported */
#define UARTCharLen_6bit				(0x01 << 6)/* not supported */
#define UARTCharLen_7bit				(0x02 << 6)/* supported only with parity */
#define UARTCharLen_8bit				(0x03 << 6)

#define UARTParity_Even					(0x00 << 1)
#define UARTParity_Odd					(0x01 << 1)
#define UARTParity_No					(0x04 << 1)

#define UARTStopBits_1					(0x00 << 4)
#define UARTStopBits_1_5				(0x01 << 4)
#define UARTStopBits_2					(0x02 << 4)
#define UARTStopBits_0_5				(0x03 << 4)
/* Bitfield inProtoMask */
#define UBX_Protoc						(0x01 << 0)
#define NMEA_Protoc						(0x01 << 1)
#define RTCM_Protoc						(0x01 << 2)
/* Exported functions ------------------------------------------------------- */
UBXFlag ubloxUBXProtoUARTConfig(UBXProtoUARTConfigDef* ConfigStructure);
void SendUBXStruct(UBXStrcutureDef* UBXStructure);
UBXFlag ChkSumVerify(UBXStrcutureDef* UBXStructure);
void CompChkSumVal(UBXStrcutureDef* UBXStructure);

#endif /* __UBLOXM8_H */

/******************************** END OF FILE *********************************/
