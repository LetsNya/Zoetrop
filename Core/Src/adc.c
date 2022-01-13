#include <adc.h>

#define ADC_MAX 4095.0f

#define ADC_POT1 ADC1
#define ADC_POT2 ADC2


//Analog-to-digital converter initialization
void adc_init()
{
	LL_ADC_StartCalibration(ADC_POT1, LL_ADC_SINGLE_ENDED);
	LL_ADC_StartCalibration(ADC_POT2, LL_ADC_SINGLE_ENDED);

	while(LL_ADC_IsCalibrationOnGoing(ADC_POT1));
	while(LL_ADC_IsCalibrationOnGoing(ADC_POT2));

	HAL_Delay(100);

	LL_ADC_Enable(ADC_POT1);
	LL_ADC_Enable(ADC_POT2);

	LL_ADC_REG_SetContinuousMode(ADC_POT1, LL_ADC_REG_CONV_CONTINUOUS);
	LL_ADC_REG_SetContinuousMode(ADC_POT2, LL_ADC_REG_CONV_CONTINUOUS);

	LL_ADC_REG_StartConversion (ADC_POT1);
	LL_ADC_REG_StartConversion (ADC_POT2);
}

//Get the value of chosen potenciometer (potenciometer_1 or potenciometer_2)
float adc_pot_return(potenciometer_t POT)
{
	float conversion = 0;

	if(POT == potenciometer_1)
		conversion = (float)LL_ADC_REG_ReadConversionData32(ADC_POT1) / ADC_MAX;
	else
		conversion = (float)LL_ADC_REG_ReadConversionData32(ADC_POT2) / ADC_MAX;

	return conversion;

}
