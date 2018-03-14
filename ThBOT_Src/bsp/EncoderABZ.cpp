/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "EncoderABZ.h"

#define TIMER_INIT_VALUE     2147483647

static int32_t absoluteCountLeft = 0;
static int32_t absoluteCountRight = 0;


EncoderABZ::EncoderABZ(uint32_t id) {
	this->id = id;
}

EncoderABZ::~EncoderABZ() {
	// TODO Auto-generated destructor stub
}

int32_t EncoderABZ::GetDeltaStep()
{
	int32_t absoluteCount = 0;
	int32_t deltaCount = 0;

	absoluteCount = GetAbsoluteStep();

	switch (this->id)
	{
		case ENCODER_1:
		{
			deltaCount = absoluteCountLeft - absoluteCount;
			absoluteCountLeft = absoluteCount;
		}break;
		case ENCODER_2:
		{
			deltaCount = absoluteCountRight - absoluteCount;
			absoluteCountRight = absoluteCount;
		}break;
	}
	return deltaCount;
}

int32_t EncoderABZ::GetAbsoluteStep()
{
	int32_t absoluteCount = 0;

	switch (this->id)
	{
		case ENCODER_1:
		{
			const uint32_t count_1 = __HAL_TIM_GET_COUNTER(&htim5);
			absoluteCount = TIMER_INIT_VALUE - count_1;
		}break;
		case ENCODER_2:
		{
			const uint32_t count_1 = __HAL_TIM_GET_COUNTER(&htim2);
			absoluteCount = TIMER_INIT_VALUE - count_1;
		}break;
	}
	return absoluteCount;
}
