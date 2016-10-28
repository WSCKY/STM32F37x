/**
  ******************************************************************************
  * @file    control.c
  * @author  '^_^'
  * @version V0.0.0
  * @date    2-July-2015
  * @brief   curve control.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "control.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint16_t PointX[] = {0, 85, 170, 255, 340, 425, 510, 595, 680};
const uint16_t PointY[] = {0, 230, 300, 350, 400, 450, 500, 550, 610};
uint16_t Curve_K[8];
uint16_t Curve_B[8];
/* Private function prototypes -----------------------------------------------*/
static int16_t Get_64K(uint16_t Ax, uint16_t Ay, uint16_t Bx, uint16_t By);
static int16_t Get_64B(uint16_t Ax, uint16_t Ay, uint16_t Bx, uint16_t By);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Control by Curve.//Y = Kx + B
  * @param  
  * @retval 
  */
int16_t Curve_Ctrl(uint16_t mdata)
{
	uint32_t tmpY;
	uint8_t Curve_index;

	Curve_index = mdata / 85.0f;//0 - 7
	if(Curve_index > 7)
		Curve_index = 7;
	tmpY = (mdata * Curve_K[Curve_index]) + Curve_B[Curve_index];//Y = Kx + B
	tmpY = tmpY / 50.0f;
	return tmpY;
}

/**
  * @brief  Calculate K.
  * @param  
  * @retval 
  */
static int16_t Get_64K(uint16_t Ax, uint16_t Ay, uint16_t Bx, uint16_t By)
{
	int16_t Tmp1, Tmp2;
	Tmp1 = By - Ay;
	Tmp2 = Bx - Ax;
	return (Tmp1 * 50 / Tmp2);
}

/**
  * @brief  Calculate B.
  * @param  
  * @retval 
  */
static int16_t Get_64B(uint16_t Ax, uint16_t Ay, uint16_t Bx, uint16_t By)
{
	int16_t Tmp1, Tmp2, TmpK;
	Tmp1 = By - Ay;
	Tmp2 = Bx - Ax;
	TmpK = (Tmp1 * 50 / Tmp2);
	return ((Ay * 50 - (Ax * TmpK)));
}

/**
  * @brief  Calculate curve parameter.
  * @param  None
  * @retval None
  */
void CalCtrlCurve(void)
{
	uint8_t Index;
	for(Index = 0; Index < 8; Index ++)
	{
			Curve_K[Index] = Get_64K(PointX[Index], PointY[Index], PointX[Index + 1], PointY[Index + 1]);
			Curve_B[Index] = Get_64B(PointX[Index], PointY[Index], PointX[Index + 1], PointY[Index + 1]);
	}
}

/******************************** END OF FILE *********************************/
