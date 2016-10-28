/*
@date   22-July-2015
@modify '^_^'
*/
#include "main.h"

UBXProtoUARTConfigDef ConfigStructure;
uint8_t EnaPOSLLH[27] = {
0xB5, 0x62, 0x01, 0x06, 0x00, 0x00, 0x07, 0x16, 0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47
};
//uint8_t EnaPVT[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
uint8_t EnaTIMEUTC[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x01, 0x2D, 0x85};
//uint8_t SetRate[22] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x02, 0x00, 0x01, 0x00, 0x7B, 0x16, 0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
int32_t Coord = 0x00000000;

/*****test variables*****/
FlagStatus POSLLHSta = RESET, TIMEUTCSta = RESET;
uint8_t PackIndex = 0;
float Lon = 0.0f, Lat = 0.0f, Height = 0.0f, hMSL = 0.0f;
uint16_t Year = 0;
uint8_t Month = 0, Day = 0, Hour = 0, Min = 0, Sec = 0;

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
	ConfigStructure.Baudrate = 115200;
	ConfigStructure.CharLen = UARTCharLen_8bit;
	ConfigStructure.Parity = UARTParity_No;
	ConfigStructure.StopBits = UARTStopBits_1;
	if(!ubloxUBXProtoUARTConfig(&ConfigStructure))
	{
		Error_Handler();
	}
	UART_SendString(EnaPOSLLH, 27);

	while(1)
	{
		if(DataOK == Set)
		{
			DataOK = Reset;
			BLE_LED_Toggle();
			if(ChkSumVerify(&UBXDataRec))//checksum ok.
			{
				if((UBXDataRec.CLASS == UBX_CLASS_NAV) && (UBXDataRec.ID == UBX_NAV_POSLLH))//NAV_POSLLH
				{
					PackIndex ++;
					Coord = ((int32_t)UBXDataRec.pPayload[7] << 24) | ((int32_t)UBXDataRec.pPayload[6] << 16) | \
							((int32_t)UBXDataRec.pPayload[5] << 8) | ((int32_t)UBXDataRec.pPayload[4]);
					Lon = (float)Coord / 10000000;
					Coord = ((int32_t)UBXDataRec.pPayload[11] << 24) | ((int32_t)UBXDataRec.pPayload[10] << 16) | \
							((int32_t)UBXDataRec.pPayload[9] << 8) | ((int32_t)UBXDataRec.pPayload[8]);
					Lat = (float)Coord / 10000000;
					Coord = ((int32_t)UBXDataRec.pPayload[15] << 24) | ((int32_t)UBXDataRec.pPayload[14] << 16) | \
							((int32_t)UBXDataRec.pPayload[13] << 8) | ((int32_t)UBXDataRec.pPayload[12]);
					Height = (float)Coord / 1000;
					Coord = ((int32_t)UBXDataRec.pPayload[19] << 24) | ((int32_t)UBXDataRec.pPayload[18] << 16) | \
							((int32_t)UBXDataRec.pPayload[17] << 8) | ((int32_t)UBXDataRec.pPayload[16]);
					hMSL = (float)Coord / 1000;//Height above mean sea level
				}
				else if((UBXDataRec.CLASS == UBX_CLASS_NAV) && (UBXDataRec.ID == UBX_NAV_TIMEUTC))
				{
					Year = ((uint16_t)UBXDataRec.pPayload[13] << 8) | UBXDataRec.pPayload[12];
					Month = UBXDataRec.pPayload[14];
					Day = UBXDataRec.pPayload[15];
					Hour = UBXDataRec.pPayload[16];
					Min = UBXDataRec.pPayload[17];
					Sec = UBXDataRec.pPayload[18];
				}
				else if((UBXDataRec.CLASS == UBX_CLASS_ACK) && (UBXDataRec.ID == UBX_ACK_ACK))//get an ack package
				{
					if((UBXDataRec.pPayload[0] == 0x06) && (UBXDataRec.pPayload[1] == 0x01) && (POSLLHSta == RESET))
					{
						POSLLHSta = SET;
						UART_SendString(EnaTIMEUTC, 11);
					}
					else if((UBXDataRec.pPayload[0] == 0x06) && (UBXDataRec.pPayload[1] == 0x01) && (TIMEUTCSta == RESET))
					{
						TIMEUTCSta = SET;
						ubloxSetRate(10, 2, TimeRef_GPS);
					}
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
