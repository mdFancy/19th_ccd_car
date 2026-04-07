#include "headfile.h"
#include <math.h>
//LEFT:35,RIGHT:87
//LEFT:35,RIGHT:69
int16 left_actural_speed=0;
int16 right_actural_speed=0;
int16 encoder_left=0;
int16 encoder_right=0;
int16 left_target_v;//设置左直行目标速度
int16 right_target_v;//设置右直行目标速度
float ccd_error=0;//ccd方向误差

uint8 is;

int16 steer_duty;
int16 error;
int16 near_left_line,near_right_line;
extern int16 far_road_left;
extern int16 far_road_right;

extern int16 near_road_left;
extern int16 near_road_right;
extern uint8 far_flagl,far_flagr;

extern uint16 ccd_data_ch2[128];
extern uint16 ccd_data[128];
int16 encoder_sum=0;
float imu_sum=0;
uint8 count_flag2=0;
extern int16 imu660ra_gyro_z;
extern int16 imu660ra_gyro_y;
extern int16 imu660ra_gyro_x;
uint8 sign=0;
extern uint8 count;
extern uint8 k;
extern uint16 dl1b_distance_mm;
extern int16 return_error;
extern int16 return_d;
extern float tranced_gyro_x;
extern uint8 c;
extern uint16 prefarleft;
extern uint16 prefarright;
extern uint16 motor_menuflag_key;
extern int16 near_road_mid;
extern int16 far_road_mid;
extern uint16 scan_key;
extern uint16 maxdata;
extern uint16 mindata;
extern Speed_pram speed;
extern encoder_flag;
void main()
{
	DisableGlobalIRQ();		//关闭总中断
	board_init();			// 初始化寄存器,勿删除此句代码。
	sign=imu660ra_init();
	lcd_init();
	lcd_clear(BLACK);
	ccd_init();
	pram_init();//参数初始化
	ctimer_count_init(PLUSE_LEFT);//左编码器初始化
	ctimer_count_init(PLUSE_RIGHT);//右编码器初始化
	wireless_uart_init();//无线串口初始化
	sign=dl1b_init();
	pwm_init(PWMB_CH1_P74,50,757);//舵机初始化 
	gpio_mode(P6_7,GPO_PP);
	P67=0;
	iap_init();  		//开启总中断
	MainMenu_Set();
	pwm_init(PWMB_CH4_P77, 50, 0);
	pwm_init(PWMB_CH3_P33, 50, 0);
	if(motor_menuflag_key==1)
	{
	   motor_init();//电机初始化
	}
	EnableGlobalIRQ();
	
	
	//45 81
    while(1)
	{
		
//		printf("%f\n",tranced_gyro_x);
//		printf("%d\n",dl1b_distance_mm);
//		printf("%d,%d,%d,%f\n",near_road_left,near_road_right,encoder_sum,ccd_error);//,near_road_left,near_road_right,ccd_error
//		printf("%d,%d,%d,%d,%d,%d\n",near_road_left,near_road_right,far_flagl,far_flagr,far_road_left,far_road_right);//,ccd_error,encoder_sum,imu_sum
//		printf("%d,%d,%d,%d,%d\n",far_road_left,far_road_right,prefarleft,prefarright,c);
//		printf("%d,%d\n",left_actural_speed,right_actural_speed);
//		printf("%d,%d,%d,%f,%d\n",far_road_left,far_road_right,encoder_sum,imu_sum,c);
//		printf("%d\n",encoder_sum);
//		printf("%d,%d,%d\n",maxdata,mindata,ccd_data[64]);
//		printf("%d,%f\n",steer_duty,ccd_error);

//		printf("%d\n",dl1b_distance_mm);
//		printf("%d,%d,%d,%d,%d\n",near_road_mid,near_road_left,near_road_right,near_flagl,near_flagr);

//	printf("%d,%d,%d,%d\n",left_target_v,right_target_v,left_actural_speed,right_actural_speed);
//		printf("%d\n",left_actural_speed);

//		cross_road_count_flag=1;
//			printf("%d\n",left_actural_speed);
//		printf("%d,%d\n",near_road_left,near_road_right);
//		printf("%d,%d\n",speed.left_v,speed.right_v);
		if(scan_key==1||scan_key==2)
		{
		  lcd();
		}
		else if(scan_key==3)
		{
			debugging();
		}
		else if(scan_key==4)
		{
			ccd_send_data(WIRELESS_UART, ccd_data);
		}
		else if(scan_key==5)
		{
			ccd_send_data(WIRELESS_UART, ccd_data_ch2);
		}
//		for(is=0;is<123;is++)
//		{
//			printf("%d\n",ccd_data[is]);//abs(ccd_data[is]-ccd_data[is+5])*100/(ccd_data[is]+ccd_data[is+5])
//		}
//		printf("%d\n",0);
		
//		lcd_showint16(0,0,right_line-left_line);
//		lcd_showint16(0,1,count);
//		lcd_showint16(0,2,status_1);
//		lcd_showint16(0,3,status_2);
//		lcd_showint16(0,4,status_3);
//		lcd_showint16(0,2,error);//显示浮点数   整数显示2位   小数显示三位
//		lcd_showint16(0,3,steer_duty);//显示浮点数   整数显示2位   小数显示三位
//		lcd_showint16(0,4,ccd_error);//显示浮点数   整数显示2位   小数显示三位
//		
//		
  }
}

