/*
 * Test.cpp
 *
 *  Created on: 25 mars 2018
 *      Author: jpb
 */

#include <string.h>
#include <math.h>

#include <RobotCore/RobotCore.h>
#include <tests/Test.h>

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "thb-fsm.h"
#include "thb-bsp.h"

static char SendResponse[80];
static char TestResponse[64];

static char Int1[12];
static char Int2[12];
static char Float1[12];
static char Float2[12];
static char Float3[12];
static char Float4[12];

static int32_t count_left = 0;
static int32_t count_right = 0;

static void test_printInt(int i, char * Res)
{
	char tmpSign[2];
	if (i < 0 )
		tmpSign[0] = '-';
	else
		tmpSign[0] = ' ';
	tmpSign[1] = '\0';

	int tmpVal = (i < 0) ? -i : i;

	sprintf (Res, "%s%10.10i", tmpSign, tmpVal);
}

static void test_printFloat(float f, char * Res, int pres)
{
	//https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
	//float adc_read = 678.0123;

	char tmpSign[2];// = (f < 0) ? "-" : " ";
	if (f < 0 )
		tmpSign[0] = '-';
	else
		tmpSign[0] = ' ';
	tmpSign[1] = '\0';

	float tmpVal = (f < 0) ? -f : f;

	int tmpInt1 = tmpVal;                  // Get the integer (678).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	for(int Pwr=0; Pwr < pres; Pwr++)
		tmpFrac *= 10;

	int tmpInt2 = trunc(tmpFrac);  // Turn into integer (123).

	// Print as parts, note that you need 0-padding for fractional bit.
	switch (pres)
	{
	case 1:
	{
		sprintf (Res, "%s%8.8d.%01d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 2:
	{
		sprintf (Res, "%s%7.7d.%02d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 3:
	{
		sprintf (Res, "%s%6.6d.%03d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 4:
	{
		sprintf (Res, "%s%5.5d.%04d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 5:
	{
		sprintf (Res, "%s%4.4d.%05d", tmpSign, tmpInt1, tmpInt2);
	}break;

	}
}

static void test_printDouble(double d, char * Res, int pres)
{
	//https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
	//float adc_read = 678.0123;

	char tmpSign[2];// = (f < 0) ? "-" : " ";
	if (d < 0 )
		tmpSign[0] = '-';
	else
		tmpSign[0] = ' ';
	tmpSign[1] = '\0';

	double tmpVal = (d < 0) ? -d : d;

	int tmpInt1 = tmpVal;                  // Get the integer (678).
	double tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	for(int Pwr=0; Pwr < pres; Pwr++)
		tmpFrac *= 10;

	int tmpInt2 = trunc(tmpFrac);  // Turn into integer (123).

	// Print as parts, note that you need 0-padding for fractional bit.
	switch (pres)
	{
	case 1:
	{
		sprintf (Res, "%s%8.8d.%01d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 2:
	{
		sprintf (Res, "%s%7.7d.%02d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 3:
	{
		sprintf (Res, "%s%6.6d.%03d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 4:
	{
		sprintf (Res, "%s%5.5d.%04d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 5:
	{
		sprintf (Res, "%s%4.4d.%05d", tmpSign, tmpInt1, tmpInt2);
	}break;

	}
}

static void test_send_reponse(uint32_t State, char * pu8_Buff)
{
	sprintf(SendResponse, "RSP:%2.2u:%2.2u:%s\n", MODE_TESTS, (unsigned int)State, pu8_Buff);
	//printf("Response [%i] : %s", strlen(SendResponse), SendResponse);
	thb_UART5_SendData(SendResponse, strlen(SendResponse));
}

Test::Test(RobotCore * Rob) {
	Robocore = Rob;

}

Test::~Test() {
}

int Test::Run(uint32_t State)
{
	GPIO_PinState PwrIn;
	int32_t count_1 = 0;
	int32_t count_2 = 0;
	double Distance_1 = 0.0;
	double Distance_2 = 0.0;
	char dir_1 = '\0';
	char dir_2 = '\0';
	uint32_t PercentPower = Robocore->GetPercentPower();

	switch (State)
	{
    	case STATE_IDLE :
    	{
    		thb_SetPwmRight(0);
    		thb_SetPwmLeft(0);
    	    return 0;
    	}break;
	    case STATE_GET_PWR_STATE :
	    {
	    	PwrIn = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0);
	    	printf("JPB3 : %d \n", PwrIn);

	    	if (PwrIn == GPIO_PIN_RESET)
	    	  sprintf(TestResponse, "%s", "ON ");
	    	else
	    	  sprintf(TestResponse, "%s", "OFF");

	    	test_send_reponse(State, TestResponse);

	    }break;
	    case STATE_ENC_READ :
	    {
	    	count_1 = Robocore->EncoderRightGetAbsoluteStep();
	    	count_2 = Robocore->EncoderLeftGetAbsoluteStep();
	    	Distance_1 = Robocore->EncoderRightGetAbsoluteMM();
	    	Distance_2 = Robocore->EncoderLeftGetAbsoluteMM();

	    	if (count_1 > 0)
	    	{
	    		dir_1 = '+';
	    		count_right = count_1;
	    	}
	    	else
	    	{
	    		dir_1 = '-';
	    		count_right = -count_1;
	    	}

	    	if (count_2 > 0)
	    	{
	    		dir_2 = '+';
	    		count_left = count_2;
	    	}
	    	else
	    	{
	    		dir_2 = '-';
	    		count_left = -count_2;
	    	}

	    	test_printDouble(Distance_1, Float1, 2);
	    	test_printDouble(Distance_2, Float2, 2);

	    	/*printf("Tick right = %10.10u (%c) - %s\n",
	    			(unsigned int)count_right, dir_1, Float1);
	    	printf("Tick left  = %10.10u (%c) - %s\n",
	    			(unsigned int)count_left, dir_2, Float2);
*/
	    	sprintf(TestResponse, "%10.10u:%c:%10.10u:%c:%s:%s",
	    			(unsigned int)count_left, dir_2,
					(unsigned int)count_right, dir_1, Float1, Float2);
	    	test_send_reponse(State, TestResponse);
	    }break;
	    case STATE_MOVE_FORWARD :
	    {
	    	printf("JPB3 : test marche avant \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

	    }break;
	    case STATE_MOVE_BACKWARD :
	    {
	    	printf("JPB3 : test marche arriere \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	    }break;
	    case STATE_TURN_RIGTH :
	    {
	    	printf("JPB3 : test rotation a droite \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
	    }break;
	    case STATE_TURN_LEFT :
	    {
	    	printf("JPB3 : test rotation a gauche \n");
	    	thb_SetPwmRight(PercentPower);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	    	thb_SetPwmLeft(PercentPower);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	    }break;
	    case STATE_ODOM_PARAMS :
	    {
	    	Odom_t OdomVal = Robocore->GetOdomValue();

	    	printf("JPB3 : test odometrie \n");

	    	test_printInt(OdomVal.pos_x, Int1);
	    	test_printInt(OdomVal.pos_y, Int2);
	    	test_printFloat(OdomVal.angle, Float1, 3);
	    	test_printFloat(OdomVal.speed, Float2, 3);
	    	test_printFloat(OdomVal.accel, Float3, 3);
	    	//test_printFloat(0000.78345, Float4, 5);

	    	//printf(">> %s:%s:%s:%s \n",Float1,Float2,Float3,Float4);
	    	sprintf(TestResponse, "%s:%s:%s:%s",Int1,Int2,Float1,Float2);
	    	test_send_reponse(State, TestResponse);
	    }break;
	}
	osDelay(2000);
	thb_fsm_ChangeModeState(MODE_TESTS, STATE_IDLE);
	return 0;
}