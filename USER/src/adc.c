#include "headfile.h"
uint16 my_adc_collect(void)
{
	uint8 count;
	uint16 max,min;
	uint16 adc_collect[10];
	uint16 sum=0;
	for(count=0;count<3;count++)
	{
		adc_collect[count]=adc_once(AD_CHANNEL, AD_RESOLUTION); 
	}
	max=min=adc_collect[0];
	for(count=0;count<10;count++)
	{
		sum+=adc_collect[count];
		if(adc_collect[count]>=max)
		{
			max=adc_collect[count];
		}
		else if(adc_collect[count]<=min)
		{
			min=adc_collect[count];
		}
	}
	return (sum-max-min)/8;
}