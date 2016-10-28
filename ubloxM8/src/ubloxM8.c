/**
  ******************************************************************************
  * @file    ubloxM8.c 
  * @author  '^_^'
  * @version V0.0.0
  * @date    20-July-2015
  * @brief   Operate ubloxM8 moudle by uart in UBX mode.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ubloxM8.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t RecBuffer[120];
UBXStrcutureDef UBXDataRec = {0xB5, 0x62, 0x0, 0x0, 0x0, RecBuffer, 0x0, 0x0};
static uint8_t PrivateBuffer[20] = {0};
UBXStrcutureDef UBXStructure = {0xB5, 0x62, 0x0, 0x0, 0x0, PrivateBuffer, 0x0, 0x0};
UBXFlag DataOK = Reset;

uint8_t Char;
uint8_t Step = 0;
uint8_t LenH, LenL;
static uint16_t LenIndex = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  configure UBX protocol & UART.
  * @param  pointer to a Config Structure.
  * @retval Config status.
  */
UBXFlag ubloxUBXProtoUARTConfig(UBXProtoUARTConfigDef* ConfigStructure)
{
	uint8_t TestCnt = 5, Index = 0;
	uint32_t Baudrate = ConfigStructure->Baudrate;
	for(Index = 0; Index < 20; Index ++)
		PrivateBuffer[Index] = 0x0;
	PrivateBuffer[0] = Port_UART1;
	PrivateBuffer[4] = ConfigStructure->CharLen;
	PrivateBuffer[5] = ConfigStructure->Parity | ConfigStructure->StopBits;
	PrivateBuffer[8] = Baudrate & 0xFF;
	PrivateBuffer[9] = (Baudrate >> 8) & 0xFF;
	PrivateBuffer[10] = (Baudrate >> 16) & 0xFF;
	PrivateBuffer[11] = (Baudrate >> 24) & 0xFF;
	PrivateBuffer[12] = ConfigStructure->inProtocol;
	PrivateBuffer[14] = ConfigStructure->outProtocol;
	UBXStructure.SYNCHAR1 = UBX_SYNCHAR1;
	UBXStructure.SYNCHAR2 = UBX_SYNCHAR2;
	UBXStructure.CLASS    = UBX_CLASS_CFG;
	UBXStructure.ID       = UBX_CFG_PRT;
	UBXStructure.LENGTH   = 20;
	UBXStructure.pPayload = PrivateBuffer;
	CompChkSumVal(&UBXStructure);

	SendUBXStruct(&UBXStructure);
	Index = 0;
	while(TestCnt)
	{
		if(DataOK == Set)
		{
			DataOK = Reset;
			TestCnt --;
			if(ChkSumVerify(&UBXDataRec))//checksum ok.
			{
				if((UBXDataRec.CLASS == UBX_CLASS_ACK) && (UBXDataRec.ID == UBX_ACK_ACK) && (UBXDataRec.pPayload[0] == 0x06) && (UBXDataRec.pPayload[1] == 0x00))
				{
					return Set;
				}
			}
			else
				SendUBXStruct(&UBXStructure);
		}
		ubloxDelay(1);
		Index ++;
		if(Index == 200)
		{
			Index = 0;
			TestCnt --;
			SendUBXStruct(&UBXStructure);
		}
	}
	return Reset;
}

/**
  * @brief  Send UBX package.
  * @param  pointer to a UBX Structure.
  * @retval None
  */
void SendUBXStruct(UBXStrcutureDef* UBXStructure)
{
	uint16_t Length = 0;
	ubloxUARTSend(UBXStructure->SYNCHAR1);
	ubloxUARTSend(UBXStructure->SYNCHAR2);
	ubloxUARTSend(UBXStructure->CLASS);
	ubloxUARTSend(UBXStructure->ID);
	ubloxUARTSend((UBXStructure->LENGTH) & 0xFF);
	ubloxUARTSend((UBXStructure->LENGTH >> 8) &0xFF);
	for(Length = 0; Length < UBXStructure->LENGTH; Length ++)
		ubloxUARTSend(*(UBXStructure->pPayload + Length));
	ubloxUARTSend(UBXStructure->CK_A);
	ubloxUARTSend(UBXStructure->CK_B);
}

/**
  * @brief  Verify UBX package.
  * @param  pointer to a UBX Structure.
  * @retval check status.
  */
UBXFlag ChkSumVerify(UBXStrcutureDef* UBXStructure)
{
	uint8_t Chk_A = 0, Chk_B = 0;
	uint8_t Cnt = 0;
	uint16_t Leng = UBXStructure->LENGTH;

	Chk_A += UBXStructure->CLASS; Chk_B += Chk_A;
	Chk_A += UBXStructure->ID; Chk_B += Chk_A;
	Chk_A += Leng & 0xFF; Chk_B += Chk_A;
	Chk_A += (Leng >> 8) & 0xFF; Chk_B += Chk_A;

	for(Cnt = 0; Cnt < (UBXStructure->LENGTH); Cnt ++)
	{
		Chk_A += (UBXStructure->pPayload)[Cnt];
		Chk_B += Chk_A;
	}
	if((Chk_A == UBXStructure->CK_A) && (Chk_B == UBXStructure->CK_B))
		return Set;
	else
		return Reset;
}

/**
  * @brief  Compute Checksum value.
  * @param  pointer to a UBX Structure.
  * @retval None
  */
void CompChkSumVal(UBXStrcutureDef* UBXStructure)
{
	uint8_t Chk_A = 0, Chk_B = 0;
	uint8_t Cnt = 0;
	uint16_t Leng = UBXStructure->LENGTH;
/* The checksum is calculated over the packet, 
	starting and including the CLASS field, up until, but excluding, the Checksum Field */
/* The checksum algorithm used is the 8-Bit Fletcher Algorithm, which is used in the TCP standard */
	Chk_A += UBXStructure->CLASS; Chk_B += Chk_A;
	Chk_A += UBXStructure->ID; Chk_B += Chk_A;
	Chk_A += Leng & 0xFF; Chk_B += Chk_A;
	Chk_A += (Leng >> 8) & 0xFF; Chk_B += Chk_A;

	for(Cnt = 0; Cnt < (UBXStructure->LENGTH); Cnt ++)
	{
		Chk_A += (UBXStructure->pPayload)[Cnt];
		Chk_B += Chk_A;
	}
	UBXStructure->CK_A = Chk_A;
	UBXStructure->CK_B = Chk_B;
}

/**
  * @brief  This function handles UART1 interrupt request.
  * @param  None
  * @retval None
  */
void ubloxRecDataHandler(void)
{
	if(USART_GetITStatus(MyUART, USART_IT_RXNE) != RESET)
	{
		Char = USART_ReceiveData(MyUART);
		switch(Step)
		{
			case 0:
				if(Char == UBX_SYNCHAR1)
				{
					UBXDataRec.SYNCHAR1 = Char;
					Step ++;
				}
			break;
			case 1:
				if(Char == UBX_SYNCHAR2)
				{
					UBXDataRec.SYNCHAR2 = Char;
					Step ++;
				}
				else
					Step = 0;
			break;
			case 2:
				UBXDataRec.CLASS = Char;
				Step ++;
			break;
			case 3:
				UBXDataRec.ID = Char;
				Step ++;
			break;
			case 4:
				LenL = Char;
				Step ++;
			break;
			case 5:
				LenH = Char;
				UBXDataRec.LENGTH = (((uint16_t)LenH) << 8 | LenL);
				LenIndex = 0;
				Step ++;
			break;
			case 6:
				(UBXDataRec.pPayload)[LenIndex] = Char;
				LenIndex ++;
				if(LenIndex == UBXDataRec.LENGTH)
					Step ++;
			break;
			case 7:
				UBXDataRec.CK_A = Char;
				Step ++;
			break;
			case 8:
				UBXDataRec.CK_B = Char;
				Step = 0;
				DataOK = Set;
			break;
			default:
				Step = 0;
				LenIndex = 0;
				DataOK = Reset;
			break;
		}
	}
}

/******************************** END OF FILE *********************************/
