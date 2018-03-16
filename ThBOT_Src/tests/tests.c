
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "thb-tests.h"
#include "thb-fsm.h"
#include "thb-bsp.h"

static char SendResponse[64];
static char TestResponse[48];

#define TIMER_INIT_VALUE     2147483647
static uint32_t count_left = 0;
static uint32_t count_right = 0;

static void test_send_reponse(uint32_t State, char * pu8_Buff)
{
	sprintf(SendResponse, "RSP:%2.2u:%2.2u:%s\n", MODE_TESTS, (unsigned int)State, pu8_Buff);
	//printf("Response [%i] : %s", strlen(SendResponse), SendResponse);
	thb_UART5_SendData(SendResponse, strlen(SendResponse));
}

void test_1(uint32_t Evt)
{
  uint32_t count_1 = 0;
  uint32_t count_2 = 0;
  GPIO_PinState DirLeft = GPIO_PIN_SET;
  GPIO_PinState DirRigh = GPIO_PIN_SET;
  GPIO_PinState PwrIn;

	count_1 = __HAL_TIM_GET_COUNTER(&htim2);
	if (((count_1 /10000) != 0) && ((count_1%10000) == 0))
	{
		DirLeft ^= 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, DirLeft);
		printf("JPB1 : %i - %i \n", (unsigned int)count_1, (unsigned int)count_2);
	}

	count_2 = __HAL_TIM_GET_COUNTER(&htim5);
	if (((count_2 /10000) != 0) && ((count_2%10000) == 0))
	{
		DirRigh ^= 1;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, DirRigh);
		printf("JPB2 : %i - %i \n", (unsigned int)count_1, (unsigned int)count_2);
		PwrIn = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0);
		printf("JPB3 : %d \n", PwrIn);
	}
}

void exec_test(uint32_t State)
{
	GPIO_PinState PwrIn;
	uint32_t count_1 = 0;
	uint32_t count_2 = 0;
	char dir_1 = '\0';
	char dir_2 = '\0';

	switch (State)
	{
    	case STATE_IDLE :
    	{
    		thb_SetPwmRight(0);
    		thb_SetPwmLeft(0);
    	    return;
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
	    	count_1 = __HAL_TIM_GET_COUNTER(&htim2);
	    	count_2 = __HAL_TIM_GET_COUNTER(&htim5);

	    	if (count_1 >= TIMER_INIT_VALUE)
	    	{
	    		count_right = count_1 - TIMER_INIT_VALUE;
	    		dir_1 = '+';
	    	}
	    	else
	    	{
	    		count_right = TIMER_INIT_VALUE - count_1;
	    		dir_1 = '-';
	    	}

	    	if (count_2 > TIMER_INIT_VALUE)
	    	{
	    		count_left  = count_2 - TIMER_INIT_VALUE;
	    		dir_2 = '-';
	    	}
	    	else
	    	{
	    		count_left  = TIMER_INIT_VALUE - count_2;
	    		dir_2 = '+';
	    	}

	    	printf("Tick right = %10.10u (%c)\n",
	    			(unsigned int)count_right, dir_1);
	    	printf("Tick left  = %10.10u (%c)\n",
	    			(unsigned int)count_left, dir_2);
	    	sprintf(TestResponse, "%10.10u:%c:%10.10u:%c",
	    			(unsigned int)count_left, dir_2,
					(unsigned int)count_right, dir_1);
	    	test_send_reponse(State, TestResponse);
	    }break;
	    case STATE_MOVE_FORWARD :
	    {
	    	printf("JPB3 : test marche avant \n");
	    	thb_SetPwmRight(5000);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	    	thb_SetPwmLeft(5000);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 1);

	    }break;
	    case STATE_MOVE_BACKWARD :
	    {
	    	printf("JPB3 : test marche arriere \n");
	    	thb_SetPwmRight(5000);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	    	thb_SetPwmLeft(5000);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    }break;
	    case STATE_TURN_RIGTH :
	    {
	    	printf("JPB3 : test rotation a droite \n");
	    	thb_SetPwmRight(5000);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	    	thb_SetPwmLeft(5000);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 1);
	    }break;
	    case STATE_TURN_LEFT :
	    {
	    	printf("JPB3 : test rotation a gauche \n");
	    	thb_SetPwmRight(5000);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	    	thb_SetPwmLeft(5000);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    }break;
	}
	osDelay(2000);
	thb_fsm_ChangeModeState(MODE_TESTS, STATE_IDLE);
}


void thb_test_PrintOdometry(char * pu8_Buff)
{
	uint32_t Mode;
	uint32_t State;

	thb_fsm_GetModeState(&Mode, &State);

	if ((Mode == MODE_TESTS) && (State == STATE_ODOM_PARAMS))
	//if ((Mode == MODE_IDLE) && (State == STATE_IDLE))
	{
		//printf("PosX=%10.5f PosY=%10.5f Teta=%10.5f Speed=%10.5f\n",
		//						PosX, PosY,
		//						Teta, Speed);

		//sprintf(TestResponse, "%10.5f:%10.5f:%10.5f:%10.5f:",
		//		PosX, PosY,
		//		Teta, Speed);
		//test_send_reponse(STATE_ODOM_PARAMS, TestResponse);
		printf("%s\n", "Problem printing float!!!");
	}
}
