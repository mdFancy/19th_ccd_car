#include "headfile.h"
extern float ccd_error;
extern int16 steer_duty;
extern int16 error;
void steer_control(void)
{
	error=error_pid(ccd_error);
	steer_duty=757-error;   //steer_duty=730+KP*ccd_error+KD*(ccd_error-last_error);
	if(steer_duty<=697)
	{
		steer_duty=697;
	}
	else if(steer_duty>817)
	{
		steer_duty=817;
	}
	pwm_duty(PWMB_CH1_P74,steer_duty);
}

