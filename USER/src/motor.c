#include "headfile.h"
#define DIR_R P60
#define DIR_L P64
//#define PWMA_CH2P_P62 //PWM_R
//#define PWMA_CH4P_P66 //PWM_L
extern int16 Motor_PWM_R;
extern int16 Motor_PWM_L;
extern int16 left_target_v;
extern int16 right_target_v;

void motor_init(void)
{
	DIR_R=0;
	DIR_L=0;
	gpio_mode(DIR_R,GPO_PP);
	gpio_mode(DIR_L,GPO_PP);
	pwm_init(PWMA_CH2P_P62,17000, 0);
	pwm_init(PWMA_CH4P_P66,17000, 0);
}//电机初始化

void speed_set_mode1(void)
{
	left_target_v=left_motor_pid();
	right_target_v=right_motor_pid();
	if(left_target_v>=10000)
	{
		left_target_v=10000;
	}
	if(left_target_v<=-10000)
	{
		left_target_v=-10000;
	}
	if(right_target_v>=10000)
	{
		right_target_v=10000;
	}
	if(right_target_v<=-10000)
	{
		right_target_v=-10000;
	}
	if(left_target_v>=0&&right_target_v<=0)//+Motor_PWM_R
	{
		DIR_R=1;//右反转
		DIR_L=0;//左正转
		pwm_duty(PWMA_CH2P_P62,-1.0*right_target_v);//右反转
		pwm_duty(PWMA_CH4P_P66,left_target_v);//左正转
		
	}
	else if(left_target_v<=0&&right_target_v>=0)
	{
		DIR_R=0;//右正转
		DIR_L=1;//左反转
		pwm_duty(PWMA_CH2P_P62,right_target_v);//右正转
		pwm_duty(PWMA_CH4P_P66,-1.0*left_target_v);//左正转
	}
	else if(left_target_v<=0&&right_target_v<=0)
	{
		DIR_R=1;//右反转
		DIR_L=1;//左反转
		pwm_duty(PWMA_CH2P_P62,-1.0*right_target_v);//右反转
		pwm_duty(PWMA_CH4P_P66,-1.0*left_target_v);//左反转  
	}
	else
	{
		DIR_R=0;//右正转
		DIR_L=0;//左正转
		pwm_duty(PWMA_CH2P_P62,right_target_v);//右正转
		pwm_duty(PWMA_CH4P_P66,left_target_v);//左正转  
	}
}

