///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,逐飞科技
// * All rights reserved.
// * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
// *
// * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
// * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
// *
// * @file       		isr
// * @company	   		成都逐飞科技有限公司
// * @author     		逐飞科技(QQ790875685)
// * @version    		查看doc内version文件 版本说明
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32G12K128
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"
extern int16 left_actural_speed;
extern int16 right_actural_speed;
//UART1中断
void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //程序自动下载
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2中断
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//接收数据寄存器为：S2BUF

	}
}


//UART3中断
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//接收数据寄存器为：S3BUF

	}
}


//UART4中断
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;

		//接收数据寄存器为：S4BUF;
		if(wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback();           //无线转串口回调函数
        }
        else if(wireless_type == WIRELESS_CH9141)
        {
            bluetooth_ch9141_uart_callback();   //蓝牙转串口回调函数
        }
        else if(wireless_type == WIRELESS_CH573)
        {
            wireless_ch573_callback();          //CH573无线模块回调函数
        }
	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//点亮LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //清除中断标志
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //清除中断标志
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //清除中断标志
}

void TM0_Isr() interrupt 1
{

}
void TM1_Isr() interrupt 3
{

}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
}
void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //清除中断标志
}







uint8 count_10ms=0;
uint16 count_20ms=0;
uint8 road_status=0;
float tranced_gyro_x=0.0;
extern uint8 lcircle_status;//圆环状态量
extern uint8 rcircle_status;
extern float imu_sum;
extern uint8 imu_flag;
uint16 start_status=0;
extern uint8 cross_status;
extern uint16 ccd_data_ch2[128];
extern uint8 obs_status;
extern uint8 ramp_status;
//extern uint8 double_crossroad_status;
extern uint8 near_flagl,near_flagr,far_flagl,far_flagr;
uint16 a[35]={0};//1.左圆环 2.右圆环 3.十字 4.斑马线 5.障碍物 6.坡道
extern uint16 dl1b_distance_mm;
extern uint8 S_status;
extern uint16 bldc_key;
enum
{
	Normal=0,
	lCircle,
	rCircle,
	Crossroad,
	obstacle,
	ramp,//坡道
	Stop,
	Speedup,
};
void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //清除中断标志
	ccd_collect();	 //CCD采集数据
	start_status++;
	if(bldc_key==1&&!S_status)
	{
		pwm_duty(PWMB_CH4_P77, 790);
		pwm_duty(PWMB_CH3_P33, 790);
	}
	else
	{
		road_status=Stop;
		pwm_duty(PWMB_CH4_P77, 0);
		pwm_duty(PWMB_CH3_P33, 0);
	}
	if(start_status>=400)
	{
	start_status=400;
	road_status=find_road();//ccd数据处理,赛道判断
	//如果第一次判到圆环，则让寻路状态一直为圆环，一直执行圆环程序，直到出环，圆环标志位置零
	count_10ms++;
	count_20ms++;
	if(lcircle_status==1)
	{
		road_status=lCircle;
	}
	if(rcircle_status==1)
	{
		road_status=rCircle;
	}
	if(cross_status==1)
	{
		road_status=Crossroad;
	}
	if(obs_status==1)//障碍物
	{
		road_status=obstacle;
	}
	if(ramp_status==1)
	{
		road_status=ramp;
	}
	if(S_status==1)
	{
		road_status=Stop;
	}
	switch(road_status)
	{
		case Normal:normal_road();break;
		case lCircle:lcircle();break;
		case rCircle:rcircle();break;
		case Crossroad:crossroad();break;
//		case Double_crossroad:double_crossroad();break;
		case obstacle:obs();break;
		case ramp:ramproad();break;
		case Stop:stop();break;
		case Speedup:speedup();break;
	}
	control();
	imu660ra_get_gyro();
	tranced_gyro_x=imu660ra_gyro_transition(imu660ra_gyro_x);
	if(imu_flag==1)
	{
		imu_sum=imu_sum+1.0*tranced_gyro_x/200.0;//陀螺仪积分
	}
	steer_control();
	if(count_10ms==2)
	{
		left_actural_speed=encoder_read_left();//读取左编码器数值
		right_actural_speed=encoder_read_right();//读取右编码器数值
		encoder_integral_test();
		dl1b_get_distance();
		if(dl1b_distance_mm<=100)
		{
			P67=1;
			S_status=1;
		}
		count_10ms=0;
	}
	speed_set_mode1();//电机
}
}










//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;