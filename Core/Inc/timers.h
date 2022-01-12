#include <main.h>
#include <commons.h>

void timers_init();
void timers_enablePWM(char enable);
void timers_setPWM(float fPWM);
uint32_t timers_getVelocityTimer();
void timers_resetVeloctiyTimer();
