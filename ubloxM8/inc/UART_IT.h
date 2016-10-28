/**
  ******************************************************************************
  * @file    UART_IT.h
  * @author  '^_^'
  * @version V0.0.0
  * @date    17-June-2015
  * @brief   Header for UART_IT.c module.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_IT_H
#define __UART_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32f37x_conf.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported Definitions ------------------------------------------------------*/
#define MyUART								USART1
#define MyUART_IRQHandler					USART1_IRQHandler
/* Exported functions ------------------------------------------------------- */
void UART_Init(uint32_t BaudRate);
void UARTSendByte(uint8_t Byte);
void UART_SendString(uint8_t *pBuffer, uint8_t Length);

#endif /* __UART_IT_H */

/*************************************** END OF FILE ***************************************/
