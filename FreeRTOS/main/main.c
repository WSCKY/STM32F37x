/**
  ******************************************************************************
  * @file    main.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    27-June-2015
  * @brief   Main program body.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osTimerId MyTimer;
/* Private function prototypes -----------------------------------------------*/
static void MyRLEDTask(void const * argument);
static void MyBLEDTask(void const * argument);
static void MyUARTTask(void const * argument);
static void TimerCallback(void const *n);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/* Setup SysTick Timer for 1 msec interrupts.
     ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.
    
    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.

    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function 
       call. The NVIC_SetPriority() is defined inside the core_cm4.h file.

    4. To adjust the SysTick time base, use the following formula:
                            
         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
    
       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF
   */
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		/* Capture error */ 
		while (1);
	}
	/* LED Init */
	LED_Init(BLE_LED | RED_LED);
	BLE_LED_Off(); RED_LED_Off();

	/* UART Init */
	UART_Init(115200);

	/* Create Start task */
	osThreadDef(RLED_Thread, MyRLEDTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate (osThread(RLED_Thread), NULL);

	osThreadDef(BLED_Thread, MyBLEDTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate (osThread(BLED_Thread), NULL);

	osThreadDef(UART_Thread, MyUARTTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate (osThread(UART_Thread), NULL);

	/* Create Timer */
	osTimerDef(Timer, TimerCallback);
	MyTimer = osTimerCreate(osTimer(Timer), osTimerPeriodic, (void *)0);

	/* Start the Timer */
	osTimerStart(MyTimer, 2000);

	/* Start scheduler */
	osKernelStart (NULL, NULL);

	while(1);
}

static void MyRLEDTask(void const * argument)
{
	while(1)
	{
		/* Toggles the Red LED */
		RED_LED_Toggle();
		/* Insert delay 1000 ms */
		osDelay(1000);
	}
}

static void MyBLEDTask(void const * argument)
{
	while(1)
	{
		/* Toggles the Blue LED */
		BLE_LED_Toggle();
		/* Insert delay 100 ms */
		osDelay(100);
	}
}

static void MyUARTTask(void const * argument)
{
	while(1)
	{
		printf("Hello YUNEEC!\r\n");
		osDelay(1000);
	}
}

static void TimerCallback(void const *n)
{
	printf("Timer Task...\r\n");
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/******************************** END OF FILE *********************************/
