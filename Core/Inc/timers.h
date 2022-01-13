#include <main.h>
#include <commons.h>

void timers_init();
void timers_enablePWM(char enable);
void timers_setPWM(float fPWM);
uint32_t timers_getVelocityTimer();
void timers_resetVeloctiyTimer();
void timers_setLEDFrequency(uint32_t freq);
void timers_setPulseWidthLED(float width);
