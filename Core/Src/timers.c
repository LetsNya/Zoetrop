#include <timers.h>


#define timerPWM 		TIM1
#define timerVelocity	TIM2
#define timerLED		TIM3
#define PWM_MAX 		8000
#define CLOCK_FREQUENCY	160000000

//Initialization of timer perypherial (called once at the begining)
void timers_init()
{
	LL_TIM_OC_SetCompareCH1(timerPWM,0);
	LL_TIM_EnableCounter(timerPWM);
	LL_TIM_EnableCounter(timerVelocity);
	LL_TIM_EnableCounter(timerLED);
}


//Enable timer output (0 - disable, >0 - enable)
void timers_enablePWM(char enable)
{
	if(enable < 0)
	LL_TIM_EnableAllOutputs(timerPWM);
	else
	LL_TIM_DisableAllOutputs(timerPWM);
}

//Sets PWM of a function (range from 0.0f to 1.0f)
void timers_setPWM(float fPWM)
{
	fPWM = normalisef(fPWM, 0.0f, 1.0f);
	int iPWM = (int)(fPWM * 8000.0f);
	LL_TIM_OC_SetCompareCH1(timerPWM,iPWM);
}


//Gets current velocity counter value
uint32_t timers_getVelocityTimer()
{
	return LL_TIM_GetCounter(timerVelocity);
}

//Resets current velocity counter value
void timers_resetVeloctiyTimer()
{
	LL_TIM_SetCounter(timerVelocity,0);
}

//Sets frequency of blinks in LED (in Hz)
void timers_setLEDFrequency(uint32_t freq)
{
	uint32_t prescaler = LL_TIM_GetPrescaler(timerLED);
	uint32_t clockDivision = LL_TIM_GetClockDivision(timerLED);

	freq = CLOCK_FREQUENCY/(clockDivision * prescaler * freq);

	LL_TIM_SetAutoReload(timerLED,freq);
}

//Sets LED PWM width (from 0.0f to 1.0f)
void timers_setPulseWidthLED(float width)
{
	width = width * 65535.0f;
	LL_TIM_OC_SetCompareCH1(timerLED,(uint32_t)width);
}

