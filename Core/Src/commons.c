#include <commons.h>


//Helper function for capping minimal and maximal value of float type variable
float normalisef(float a, float min, float max)
{
	if(a <= min) return min;
	else if(a >= max) return max;
	else return a;
}
