#include "headfile.h"
extern int16 encoder_left;
extern int16 encoder_right;
extern uint8 crossroad_flag;
extern int16 encoder_sum;
extern uint8 encoder_flag;
extern uint8 count_flag2;
extern imu660ra_gyro_x;
extern float imu_sum;

//读取左编码器的值
int16 encoder_read_left(void)
{
	//外部定时器读取左编码器//
	if(DIR_LEFT==0)
	{
		encoder_left=-1*ctimer_count_read(PLUSE_LEFT);
	}
	else
	{
		encoder_left=1*ctimer_count_read(PLUSE_LEFT);
	}
	ctimer_count_clean(PLUSE_LEFT);//清除左编码器标志位//
	return encoder_left;
}

//读取右编码器的值
int16 encoder_read_right(void)
{
	if(DIR_RIGHT==1)
	{
		encoder_right=-1*ctimer_count_read(PLUSE_RIGHT);
	}
	else
	{
		encoder_right=1*ctimer_count_read(PLUSE_RIGHT);
	}
	ctimer_count_clean(PLUSE_RIGHT);//清除右编码器标志位//
	return encoder_right;
}



void encoder_integral_test(void)
{
	if(!count_flag2&&(encoder_left>=10||encoder_right>=10))
	{
		encoder_left=0;
		encoder_right=0;
		count_flag2=1;
	}
	if(encoder_flag==1)
	{
		encoder_sum=encoder_sum+((encoder_left+encoder_right)/2);//编码器积分
	}
	else
	{
		encoder_sum=0;
	}
}
