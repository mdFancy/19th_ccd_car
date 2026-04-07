#include "headfile.h"
#include "stdlib.h"

//左右电机实际速度
extern int16 left_actural_speed;
extern int16 right_actural_speed;

//参数
float KP_speed=0;
float KI_speed=0;
float KP_error=0;
float KI_error=0;
float KD_error=0;
float circle_KP_error=0;//放菜单
float circle_KD_error=0;//放菜单
float left_targetv=0;
float right_targetv=0;
float circle_left_v=0;//放菜单
float circle_right_v=0;//放菜单

//屏幕参数
extern uint16 KP_speed_key;
extern uint16 KI_speed_key;
extern uint16 left_targetv_key;
extern uint16 right_targetv_key;
extern uint16 ramp_left_targetv_key;
extern uint16 ramp_right_targetv_key;
extern uint16 KP_error_key;
extern uint16 KI_error_key;
extern uint16 KD_error_key;
extern uint16 long_KP_error_key;
extern uint16 long_KD_error_key;
extern uint16 node_key;
extern uint16 long_KP_SPEED_key;
extern uint16 long_KI_SPEED_key;
extern uint16 long_speed_key;
extern uint16 KD_gyro_key;
//extern uint16 obs_Kp_key;
//extern uint16 obs_Kd_key;
//extern uint16 curve_speed_up_key;
//extern uint16 kd_node_key;
//extern uint16 curve_KP_SPEED_key;
//extern uint16 curve_KI_SPEED_key;
int16 left_v=0;
int16 right_v=0;
extern uint16 circle_KP_error_key;   
extern uint16 circle_KD_error_key;
extern uint16 r_circle_KP_error_key;   
extern uint16 r_circle_KD_error_key;
extern uint16 circle_kp_speed_key;
extern uint16 circle_ki_speed_key;
extern uint16 circle_left_v_key;
extern uint16 circle_right_v_key;
extern float tranced_gyro_x;
//extern uint16 near_quan_key;
//extern uint16 far_quan_key;
extern uint16 circle_right_v_key;
extern float ccd_error;//误差
extern int16 steer_duty;//舵机打角值

//标志位
extern uint8 near_flagl,near_flagr,far_flagl,far_flagr;//巡线标志位
extern int16 far_road_left,far_road_right,near_road_left,near_road_right;//边线坐标
extern uint8 lcircle_status;//左圆环标志位
extern uint8 rcircle_status;//右圆环标志位
uint8  cross_road_flag=0;//十字标志位
extern uint8 ramp_flag;
extern uint8 stop_status;
extern uint8 S_status;
extern uint8 cross_road_status;
extern uint8 obs_status;
extern uint8 road_status;
extern uint8 c;
extern uint8 far_ccd_error;
extern uint8 cs;
extern enum
{
	Normal=0,
	lCircle,
	rCircle,
	Crossroad,
//	Double_crossroad,
	obstacle,
	ramp,//坡道
	Stop,
	Speedup,
};
//pid微分项
int16 last_error=0.0;
int16 return_error=0.0;
int16 return_d=0;
float last_gyro_z=0;
float last_tranced_gyro_x=0;
//陀螺仪参数
extern int16 imu660ra_gyro_z;

Speed_pram speed;
Steer_pram steer;

//差速表
const char gl_speedDifference[60]  =				
{100, 99, 97, 96, 95,	93, 92, 91, 90, 88,	87, 86, 85, 84, 82,	81, 80, 79, 78, 76,	75, 74, 73, 72, 71,	69, 68, 67, 66, 65,
63, 62, 61, 60, 59,	57, 56, 55, 54, 52,	51, 50, 48, 47, 46,	44, 43, 41, 40, 38,	37, 35, 34, 32, 30,	29, 27, 25, 24, 22};
//参数初始化
void pram_init(void)
{
	speed.KP=0;
	speed.KI=0;
	speed.left_v=0;
	speed.right_v=0;
	steer.KP=0;
	steer.KD=0;
}

//左电机速度环pi
int32 left_motor_pid(void)
{
	static int32 error=0;
	static int32 error_sum=0;
	error=speed.left_v-left_actural_speed;
	error_sum+=error;
	if(error_sum>=800)
	{
		error_sum=800;
	}
	else if(error_sum<=-800)
	{
		error_sum=-800;
	}
	return speed.KP*error+speed.KI*error_sum;
}


//右电机速度环pi
int32 right_motor_pid(void)
{
	static int32 error=0;
	static int32 error_sum=0;
	error=speed.right_v-right_actural_speed;
	error_sum+=error;
	if(error_sum>=800)
	{
		error_sum=800;
	}
	else if(error_sum<=-800)
	{
		error_sum=-800;
	}
	return speed.KP*error+speed.KI*error_sum;
}



//误差pid
float error_pid(float error)
{
	return_d=steer.KD*(error-last_error)-(KD_gyro_key)*tranced_gyro_x/200;
//	if(return_d>=30)
//	{
//		return_d=30;
//	}
//	else if(return_d<-30)
//	{
//		return_d=-30;
//	}
	return_error=steer.KP*error+return_d;
	last_error=error;
	return return_error;
}


//参数控制
void control(void)
{
	switch(road_status)
	{
		case Normal:
		{
			speed.KP=(float)KP_speed_key;
			speed.KI=((float)KI_speed_key*0.1);
			speed.left_v=(float)left_targetv_key;
			speed.right_v=(float)right_targetv_key;
			steer.KP=(((float)KP_error_key+200)*0.01);
			steer.KD=(((float)KD_error_key))*0.1;
			if(abs(ccd_error)<=node_key&&abs(far_ccd_error)<=node_key)
			{
				speed.KP=long_KP_SPEED_key;
				speed.KI=((float)long_KI_SPEED_key)*0.1;
				steer.KP=((float)long_KP_error_key)*0.1;
				steer.KD=((float)long_KD_error_key)*0.1;
				speed.left_v+=20;
				speed.right_v+=20;
				if(speed.right_v>=(float)long_speed_key+100)
				{	
					speed.right_v=(float)long_speed_key+100;
				}
				if(speed.left_v>=(float)long_speed_key+100)
				{
					speed.left_v=(float)long_speed_key+100;
				}
			}
////			else if(abs(return_d)<=kd_node_key)
////			{
////				speed.left_v+=15;
////				speed.right_v+=15;
////				speed.KP=(float)curve_KP_SPEED_key;
////				speed.KI=((float)curve_KI_SPEED_key)*0.1;
////				if(speed.right_v>=(float)curve_speed_up_key)
////				{	
////					speed.right_v=(float)curve_speed_up_key;
////				}
////				if(speed.left_v>=(float)curve_speed_up_key)
////				{
////					speed.left_v=(float)curve_speed_up_key;
////				}
////			}
			speedDifference(speed.left_v,speed.right_v);
			break;
		}
		case lCircle:
		{
			speed.KP=(float)circle_kp_speed_key;
			speed.KI=((float)circle_ki_speed_key)*0.1;
			steer.KP=((float)circle_KP_error_key)*0.1;
			steer.KD=((float)circle_KD_error_key)*0.1;
			speed.left_v=(float)circle_left_v_key;
			speed.right_v=(float)circle_right_v_key;
			if(c>=2)
			{
				speed.left_v=(float)long_speed_key+100;
				speed.right_v=(float)long_speed_key+100;
			}
			speedDifference(speed.left_v,speed.right_v);
			break;
		}
		case rCircle:
		{
			speed.KP=(float)circle_kp_speed_key;
			speed.KI=((float)circle_ki_speed_key)*0.1;
			steer.KP=((float)circle_KP_error_key)*0.1;
			steer.KD=((float)circle_KD_error_key)*0.1;
			speed.left_v=(float)circle_left_v_key;
			speed.right_v=(float)circle_right_v_key;
			if(c>=2)
			{
				speed.left_v=(float)long_speed_key+100;
				speed.right_v=(float)long_speed_key+100;
			}
			speedDifference(speed.left_v,speed.right_v);
			break;
		}
		case ramp:
		{
			speed.KP=(float)KP_speed_key;
			speed.KI=(float)KI_speed_key*0.1;
			steer.KP=((float)KP_error_key+200)*0.01-1;
			steer.KD=(((float)KD_error_key))*0.1;
			speed.left_v=(float)ramp_left_targetv_key;
			speed.right_v=(float)ramp_right_targetv_key;
			speedDifference(speed.left_v,speed.right_v);
			break;
		}
		case Stop:
		{
			speed.KP=(float)KP_speed_key;
			speed.KI=((float)KI_speed_key*0.1);
			steer.KP=(((float)KP_error_key+200)*0.01);
			steer.KD=(((float)KD_error_key))*0.1;
			speed.right_v=0;
			speed.left_v=0;
			pwm_duty(PWMB_CH4_P77,0);
			pwm_duty(PWMB_CH3_P33,0);
			break;
		}
		case obstacle:
		{
			speed.KP=(float)KP_speed_key;
			speed.KI=((float)KI_speed_key*0.1);
			steer.KP=(((float)KP_error_key+200)*0.01);
			steer.KD=(((float)KD_error_key))*0.1;
			speed.right_v=220;
			speed.left_v=220;
			speedDifference(speed.left_v,speed.right_v);
			break;
		}
		case Speedup:
		{
			speed.KP=long_KP_SPEED_key;
			speed.KI=((float)long_KI_SPEED_key)*0.1;
			steer.KP=(((float)KP_error_key+200)*0.01);
			steer.KD=(((float)KD_error_key))*0.1;
			speed.left_v=(float)long_speed_key+100;
			speed.right_v=(float)long_speed_key+100;
			speedDifference(speed.left_v,speed.right_v);
			break;
		}
		default:
		{
			steer.KP=(((float)KP_error_key+200)*0.01);
			steer.KD=(((float)KD_error_key))*0.1;
			speed.KP=(float)circle_kp_speed_key;
			speed.KI=((float)circle_ki_speed_key)*0.1;
			speed.left_v=(float)circle_left_v_key;
			speed.right_v=(float)circle_right_v_key;
			if(cs==0)
			{
				speed.left_v=(float)long_speed_key;
				speed.right_v=(float)long_speed_key;
			}
			if(cs>=1)
			{
				speed.left_v=(float)long_speed_key+50;
				speed.right_v=(float)long_speed_key+50;
			}
			speedDifference(speed.left_v,speed.right_v);
			break;
		}
	}
}


//差速计算
void speedDifference(float left_v,float right_v)
{
	if(steer_duty>757)//向左打角
	{
		speed.right_v=right_v;
		speed.left_v=left_v*(1.0*(gl_speedDifference[steer_duty-758]*0.01));
	}
	else if(steer_duty<757)//向右打角
	{
		speed.right_v=right_v*(1.0*(gl_speedDifference[756-steer_duty]*0.01));;
		speed.left_v=left_v;
	}
	else
	{
		speed.right_v=right_v;
		speed.left_v=left_v;
	}
}
