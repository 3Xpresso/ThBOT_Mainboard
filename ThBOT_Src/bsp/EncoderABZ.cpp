/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "math.h"

#include "ThBot_platform.h"
#include "EncoderABZ.h"

#define TIMER_INIT_VALUE     2147483647

EncoderABZ::EncoderABZ(Encodeur_t id) {
	this->id                     = id;
	this->LastAbsoluteValue      = 0;
	this->AbsoluteStepFromDelta  = 0;
	this->MM_per_step            = (M_PI * ENCODER_WHEEL_SIZE / ENCODER_NB_STEP);
}

EncoderABZ::~EncoderABZ() {
	// TODO Auto-generated destructor stub
}

int32_t EncoderABZ::GetDeltaStep()
{
	int32_t new_absolute = 0;
	int32_t delta = 0;

	new_absolute = GetAbsoluteStep();

	// calculate delta (new - old)
	delta = new_absolute - this->LastAbsoluteValue;
	this->AbsoluteStepFromDelta += delta;

	//update reference = new
	this->LastAbsoluteValue = new_absolute;

	return delta;
}

double EncoderABZ::GetDeltaMM()
{
	int32_t delta_step = GetDeltaStep();
	double delta_mm = delta_step * MM_per_step;

	return delta_mm;
}

double EncoderABZ::GetAbsoluteMM()
{
	int32_t new_absolute = 0;
	double new_distance = 0.0;

	new_absolute = GetAbsoluteStep();
	new_distance = new_absolute * MM_per_step;

	return new_distance;
}

int32_t EncoderABZ::GetAbsoluteStep()
{
	uint32_t Count = 0;
	int32_t absoluteCount = 0;

	switch (this->id)
	{
		case ENCODER_LEFT:
		{
			const uint32_t count_1 = __HAL_TIM_GET_COUNTER(&htim5);
			if (count_1 > TIMER_INIT_VALUE)
			{
				Count = count_1 - TIMER_INIT_VALUE;
				absoluteCount = -Count;
			}
			else
			{
				absoluteCount = TIMER_INIT_VALUE-count_1;
			}
//			printf("JPB : %i\n", absoluteCount);
		}break;
		case ENCODER_RIGHT:
		{
			const uint32_t count_1 = __HAL_TIM_GET_COUNTER(&htim2);
			if (count_1 >= TIMER_INIT_VALUE)
				absoluteCount = count_1 - TIMER_INIT_VALUE;
			else
			{
				Count = TIMER_INIT_VALUE-count_1;
				absoluteCount = -Count;
			}
//			printf("JPB : %i\n", absoluteCount);
		}break;
	}
	return absoluteCount;
}

int32_t EncoderABZ::GetAbsoluteStepFromDelta()
{
	return this->AbsoluteStepFromDelta;
}
