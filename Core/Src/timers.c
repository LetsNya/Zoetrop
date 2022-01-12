#include <timers.h>


#define timerPWM 	TIM1
#define PWM_MAX 	8000

//Initialization of timer perypherial (called once at the begining)
void timers_init()
{
	LL_TIM_OC_SetCompareCH1(timerPWM,0);
	LL_TIM_EnableCounter(timerPWM);
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
