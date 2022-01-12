#include <encoder.h>

#define ENC_EXTI 		LL_EXTI_LINE_2
#define TIMER_FREQUENCY	100000.0f
#define PULSE_NUMBER	24.0f

float currentVelocity = 0;

void encoder_init()
{
	LL_EXTI_EnableIT_0_31 (ENC_EXTI);
}

//measures velocity in rad/s
void encoder_measureVelocity()
{
	uint32_t time = timers_getVelocityTimer();
	timers_resetVeloctiyTimer();
	currentVelocity = (float)time;
	currentVelocity = PULSE_NUMBER * PIx2 / (currentVelocity * TIMER_FREQUENCY);
}

//returns velocity in rad/s
float encoder_getVelocity()
{
	return currentVelocity;
}
