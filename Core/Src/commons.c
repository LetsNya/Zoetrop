#include <commons.h>

float normalisef(float a, float min, float max)
{
	if(a <= min) return min;
	else if(a >= max) return max;
	else return a;
}
