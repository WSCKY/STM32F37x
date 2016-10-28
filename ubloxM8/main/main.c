#include "main.h"

UBXProtoUARTConfigDef ConfigStructure;
uint8_t EnaPVT[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};

int32_t Coord = 0x00000000;

/*****test variables*****/
FlagStatus PVTEnable = RESET;
uint8_t PackIndex = 0;
uint8_t NumOfSatel = 0;
float Lon = 0.0f, Lat = 0.0f, Height = 0.0f, hMSL = 0.0f;
uint16_t Year = 0;
uint8_t Month = 0, Day = 0, Hour = 0, Min = 0, Sec = 0;
uint8_t Fix = 0;

void Error_Handler(void);

int main(void)
{	
	Delay_Init(SystemCoreClock / 1000000);
	LED_Init(BLE_LED | RED_LED);
	BLE_LED_Off(); RED_LED_On();
	/* USART1 Configuration */
	UART_Init(9600);
	Delay_ms(500);

	/* Config UBX Protocol */
	ConfigStructure.inProtocol = UBX_Protoc;
	ConfigStructure.outProtocol = UBX_Protoc;
	ConfigStructure.Baudrate = 9600;
	ConfigStructure.CharLen = UARTCharLen_8bit;
	ConfigStructure.Parity = UARTParity_No;
	ConfigStructure.StopBits = UARTStopBits_1;
	if(!ubloxUBXProtoUARTConfig(&ConfigStructure))
	{
		Error_Handler();
	}
	UART_SendString(EnaPVT, 11);

	while(1)
	{
		if(DataOK == Set)
		{
			DataOK = Reset;
			BLE_LED_Toggle();
			if(ChkSumVerify(&UBXDataRec))//checksum ok.
			{
				if((UBXDataRec.CLASS == UBX_CLASS_NAV) && (UBXDataRec.ID == UBX_NAV_PVT))//NAV_PVT
				{
					PackIndex ++;
					NumOfSatel =  UBXDataRec.pPayload[23];
					Coord = ((int32_t)UBXDataRec.pPayload[27] << 24) | ((int32_t)UBXDataRec.pPayload[26] << 16) | \
							((int32_t)UBXDataRec.pPayload[25] << 8) | ((int32_t)UBXDataRec.pPayload[24]);
					Lon = (float)Coord / 10000000;
					Coord = ((int32_t)UBXDataRec.pPayload[31] << 24) | ((int32_t)UBXDataRec.pPayload[30] << 16) | \
							((int32_t)UBXDataRec.pPayload[29] << 8) | ((int32_t)UBXDataRec.pPayload[28]);
					Lat = (float)Coord / 10000000;
					Coord = ((int32_t)UBXDataRec.pPayload[35] << 24) | ((int32_t)UBXDataRec.pPayload[34] << 16) | \
							((int32_t)UBXDataRec.pPayload[33] << 8) | ((int32_t)UBXDataRec.pPayload[32]);
					Height = (float)Coord / 1000;
					Coord = ((int32_t)UBXDataRec.pPayload[39] << 24) | ((int32_t)UBXDataRec.pPayload[38] << 16) | \
							((int32_t)UBXDataRec.pPayload[37] << 8) | ((int32_t)UBXDataRec.pPayload[36]);
					hMSL = (float)Coord / 1000;
					Year = ((uint16_t)UBXDataRec.pPayload[5] << 8) | UBXDataRec.pPayload[4];
					Month = UBXDataRec.pPayload[6];
					Day = UBXDataRec.pPayload[7];
					Hour = UBXDataRec.pPayload[8];
					Min = UBXDataRec.pPayload[9];
					Sec = UBXDataRec.pPayload[10];
					Fix = UBXDataRec.pPayload[20];
				}
				else if((UBXDataRec.CLASS == UBX_CLASS_ACK) && (UBXDataRec.ID == UBX_ACK_ACK))//get an ack package
				{
					if((UBXDataRec.pPayload[0] == 0x06) && (UBXDataRec.pPayload[1] == 0x01))
						PVTEnable = SET;
				}
			}
		}
	}
}

void Error_Handler(void)
{
	while(1)
	{
		RED_LED_Toggle();
		Delay_ms(100);
	}
}
