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

static char SendResponse[96];
static char TestResponse[96-12];

//static char Int1[12];
//static char Int2[12];
static char Float1[12];
static char Float2[12];
static char Float3[12];
static char Float4[12];
static char Float5[12];

#if 0
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
#endif

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
		sprintf (Res, "%s%8.8d.%1.1d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 2:
	{
		sprintf (Res, "%s%7.7d.%02d", tmpSign, tmpInt1, tmpInt2);
	}break;
	case 3:
	{
		sprintf (Res, "%s%6.6d.%3.3d", tmpSign, tmpInt1, tmpInt2);
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
	Res[11] = '\0';
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
	Res[11] = '\0';
}

static void test_printAngle(double rad, char * Res)
{
	//https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
	//float adc_read = 678.0123;

	char tmpSign[2];// = (f < 0) ? "-" : " ";
	if (rad < 0 )
		tmpSign[0] = '-';
	else
		tmpSign[0] = ' ';
	tmpSign[1] = '\0';

	double tmpVal = (rad < 0) ? -rad : rad;

	int tmpInt1 = tmpVal;                  // Get the integer (678).
	double tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	for(int Pwr=0; Pwr < 3; Pwr++)
		tmpFrac *= 10;

	int tmpInt2 = trunc(tmpFrac);  // Turn into integer (123).

	tmpVal = (rad < 0) ? -rad : rad;
	tmpVal = (tmpVal*180)/M_PI;
	int tmpInt3 = tmpVal;

	sprintf (Res, "%s%1.1d.%03d(%3.3d)", tmpSign, tmpInt1, tmpInt2,tmpInt3);
	Res[11] = '\0';
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

void Test::TurnLeft(float PercentPower, double TargetAngle)
{
	Odom_t OdomVal = Robocore->GetOdomValue();
	double StartAngle = OdomVal.angle;

	Robocore->SetMotionMotor(MOTION_MOTOR_LEFT, BACKWARD, PercentPower);
	Robocore->SetMotionMotor(MOTION_MOTOR_RIGHT, FORWARD, PercentPower);

	while((((OdomVal.angle-StartAngle)*180)/M_PI) < TargetAngle)
	{
		osDelay(50);
		OdomVal = Robocore->GetOdomValue();
	}
	Robocore->SetMotionMotor(MOTION_MOTOR_LEFT, BACKWARD, 0);
	Robocore->SetMotionMotor(MOTION_MOTOR_RIGHT, FORWARD, 0);

	printf("TurnLeft: target angle %f => achieved %f\n",
			TargetAngle,
			(((OdomVal.angle-StartAngle)*180)/M_PI));
}

int Test::Run(uint32_t State)
{
	GPIO_PinState PwrIn;
	int32_t count_1 = 0;
	int32_t count_2 = 0;
	int32_t count_1_d = 0;
	int32_t count_2_d = 0;
	int32_t count_left = 0;
	int32_t count_right = 0;
	int32_t count_left_d = 0;
	int32_t count_right_d = 0;
	double Distance_1 = 0.0;
	double Distance_2 = 0.0;
	char dir_1 = '\0';
	char dir_2 = '\0';
	char dir_1_d = '\0';
	char dir_2_d = '\0';

	uint32_t PercentPower = Robocore->GetPercentPower();

	switch (State)
	{
    	case STATE_IDLE :
    	{
    		Robocore->SetMotionMotor(MOTION_MOTOR_LEFT, BACKWARD, 0);
    		Robocore->SetMotionMotor(MOTION_MOTOR_RIGHT, FORWARD, 0);
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
	    	count_1    = Robocore->EncoderRightGetAbsoluteStep();
	    	count_2    = Robocore->EncoderLeftGetAbsoluteStep();
	    	Distance_1 = Robocore->EncoderRightGetAbsoluteMM();
	    	Distance_2 = Robocore->EncoderLeftGetAbsoluteMM();

	    	count_1_d = Robocore->EncoderRightGetAbsoluteStepFromDelta();
	    	count_2_d = Robocore->EncoderLeftAbsoluteStepFromDelta();
	    	dir_1_d = '\0';
	    	dir_2_d = '\0';

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

	    	if (count_1_d > 0)
	    	{
	    		dir_1_d = '+';
	    		count_right_d = count_1_d;
	    	}
	    	else
	    	{
	    		dir_1_d = '-';
	    		count_right_d = -count_1_d;
	    	}

	    	if (count_2_d > 0)
	    	{
	    		dir_2_d = '+';
	    		count_left_d = count_2_d;
	    	}
	    	else
	    	{
	    		dir_2_d = '-';
	    		count_left_d = -count_2_d;
	    	}

	    	test_printDouble(Distance_1, Float1, 2);
	    	test_printDouble(Distance_2, Float2, 2);

	    	/*printf("Tick right = %10.10u (%c) - %s\n",
	    			(unsigned int)count_right, dir_1, Float1);
	    	printf("Tick left  = %10.10u (%c) - %s\n",
	    			(unsigned int)count_left, dir_2, Float2);
*/
	    	sprintf(TestResponse, "%10.10u:%c:%10.10u:%c:%s:%s:%10.10u:%c:%10.10u:%c",
	    			(unsigned int)count_left, dir_2,
					(unsigned int)count_right, dir_1,
					Float2, Float1,
					(unsigned int)count_left_d, dir_2_d,
					(unsigned int)count_right_d, dir_1_d);
	    	test_send_reponse(State, TestResponse);
	    }break;
	    case STATE_MOVE_FORWARD :
	    {
	    	printf("JPB3 : test marche avant \n");
	    	thb_SetPwmRight(PercentPower);

	    	Robocore->SetMotionMotor(MOTION_MOTOR_LEFT, FORWARD, PercentPower);
	    	Robocore->SetMotionMotor(MOTION_MOTOR_RIGHT, FORWARD, PercentPower);
	    }break;
	    case STATE_MOVE_BACKWARD :
	    {
	    	printf("JPB3 : test marche arriere \n");
	    	thb_SetPwmRight(PercentPower);
	    	Robocore->SetMotionMotor(MOTION_MOTOR_LEFT, BACKWARD, PercentPower);
	    	Robocore->SetMotionMotor(MOTION_MOTOR_RIGHT, BACKWARD, PercentPower);
	    }break;
	    case STATE_TURN_RIGTH :
	    {
	    	printf("JPB3 : test rotation a droite \n");
	    	thb_SetPwmRight(PercentPower);
	    	Robocore->SetMotionMotor(MOTION_MOTOR_LEFT, FORWARD, PercentPower);
	    	Robocore->SetMotionMotor(MOTION_MOTOR_RIGHT, BACKWARD, PercentPower);
	    }break;
	    case STATE_TURN_LEFT :
	    {
	    	printf("JPB3 : test rotation a gauche \n");
	    	thb_SetPwmRight(PercentPower);
	    	Robocore->SetMotionMotor(MOTION_MOTOR_LEFT, BACKWARD, PercentPower);
	    	Robocore->SetMotionMotor(MOTION_MOTOR_RIGHT, FORWARD, PercentPower);
	    }break;

	    case STATE_TURN_LEFT_45 :
	    {
	    	printf("JPB3 : test rotation a gauche de 45 degre \n");
	    	thb_SetPwmRight(PercentPower);

	    	TurnLeft(PercentPower, 45.0);
	    }break;

	    case STATE_TURN_LEFT_90 :
	    {
	    	printf("JPB3 : test rotation a gauche de 90 degre \n");
	    	thb_SetPwmRight(PercentPower);

	    	TurnLeft(PercentPower, 90.0);
	    }break;

	    case STATE_TURN_LEFT_180 :
	    {
	    	printf("JPB3 : test rotation a gauche de 180 degre \n");
	    	thb_SetPwmRight(PercentPower);

	    	TurnLeft(PercentPower, 180.0);
	    }break;

	    case STATE_TURN_LEFT_270 :
	    {
	    	printf("JPB3 : test rotation a gauche de 270 degre \n");
	    	thb_SetPwmRight(PercentPower);

	    	TurnLeft(PercentPower, 270.0);
	    }break;

	    case STATE_ODOM_PARAMS :
	    {
	    	Odom_t OdomVal = Robocore->GetOdomValue();

	    	printf("JPB3 : test odometrie\n");

	    	test_printDouble(OdomVal.pos_x, Float4, 3);
	    	test_printDouble(OdomVal.pos_y, Float5, 3);
	    	test_printAngle(OdomVal.angle, Float1);
	    	test_printFloat(OdomVal.speed, Float2, 3);
	    	test_printFloat(OdomVal.accel, Float3, 3);

	    	sprintf(TestResponse, "%s:%s:%s:%s",Float4,Float5,Float1,Float2);
	    	test_send_reponse(State, TestResponse);
	    }break;
	}
	osDelay(2000);
	thb_fsm_ChangeModeState(MODE_TESTS, STATE_IDLE);
	return 0;
}
