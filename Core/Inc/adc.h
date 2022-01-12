#include <main.h>



typedef enum
{
	potenciometer_1,
	potenciometer_2
}potenciometer_t;


void adc_init();
float adc_pot_return(potenciometer_t POT);
