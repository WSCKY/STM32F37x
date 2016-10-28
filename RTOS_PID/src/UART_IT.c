/**
  ******************************************************************************
  * @file    UART_IT.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    17-June-2015
  * @brief   UART communication (IT mode).
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "UART_IT.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void NVICConfig(void);
/* Private functions ---------------------------------------------------------*/

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

/**
  * @brief  UART Initialize.
  * @param  BaudRate: Baudrate.
  * @retval None
  */
void UART_Init(uint32_t BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
	/* UART1 Pin Configuration(PA9, PA10) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* UART1 Configuration */
	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	NVICConfig();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  NVIC configuration.
  * @param  None
  * @retval None
  */
static void NVICConfig(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Send String By UART.
  * @param  *pBuffer: pointer to a string to send.
  * @param  Length: Buffer length.
  * @retval None
  */
void UART_SendString(uint8_t *pBuffer, uint8_t Length)
{
	while(Length)
	{
		while((USART1->ISR & 0x40) == 0);
		USART1->TDR = *pBuffer;
		pBuffer ++;
		Length --;
	}
}

/**
  * @brief  Redefine fputc function.
  * @param  
  * @retval 
  */
PUTCHAR_PROTOTYPE
{
	USART_SendData(USART1, (uint8_t) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	return ch;
}

/*************************************** END OF FILE ***************************************/
