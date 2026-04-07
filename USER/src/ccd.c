#include "headfile.h"
#include <stdlib.h>

uint8 i;
//int16 near_standard=0;//需要黑白值的差比和
//int16 far_standard=0;//需要黑白值的差比和
uint16 near_standard=0;
uint16 far_standard=0;
//边线
uint8 near_flagl,near_flagr;
uint8 far_flagl,far_flagr;
int16 near_road_left=0;
int16 near_road_right=127;
int16 far_road_left=0;
int16 far_road_right=127;
int16 far_right_roadline=77;
int16 near_right_roadline=90;
int16 near_road_mid=64;
int16 far_road_mid=64;
uint8 linecount=0;
uint16 prefarleft=127;
uint16 prefarright=0;
uint16 maxdata;
uint16 mindata;
uint16 maxdatach2;
uint16 mindatach2;
uint8 findflag=1;
uint32 linenearavg=0;
uint32 linefaravg=0;
uint8 far_ccd_error=0;
//元素
uint8 cross_status=0;
uint8 obs_status=0;
uint8 ramp_status=0;
uint8 zebra_count=0;
uint8 robs=0;
uint8 lobs=0;
uint8 flag=1;
uint8 SS_status=0;


extern uint8 cross_road_flag;

extern float ccd_error;//控制舵机
extern uint16 ccd_data[128];
extern uint16 ccd_data_ch2[128];
extern uint8 out_status;
extern uint8 S_status;
extern uint8 lcircle_status;
extern uint8 rcircle_status;
extern uint8 enter_status;
extern int16 encoder_sum;
extern uint8 encoder_flag;
extern uint16 dl1b_distance_mm;
extern float targetv;
extern uint16 a[];
extern uint8 k;
extern uint8 c;
extern uint8 ramp_status;
extern uint8 circle_out_status;
extern uint8 ramp_out_status;
//extern uint8 double_crossroad_status;
extern uint16 ccd_sta1_key;
extern uint16 ccd_sta2_key;
extern uint8 cross_out_status;
extern uint16 cir_line1_key;
extern uint16 cir_line2_key;

//extern uint8 double_crossroad_out_status;
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

int16 cal(uint16 a,uint16 b)//计算差比和 对比度
{
	return abs(a-b)*100/(a+b);
}

uint8 find_road()
{
	//初始化
	near_flagl=0;
	near_flagr=0;
	far_flagl=0;
	far_flagr=0;
	zebra_count=0;
	near_standard=ccd_sta1_key;
    far_standard=ccd_sta2_key;
	
	
	if(ramp_out_status==1)
	{
		near_road_mid=64;
		ramp_out_status=0;
	}
	if(cross_out_status==1)
	{
		near_road_mid=64;
		cross_out_status=0;
	}
//	if(double_crossroad_out_status==1)
//	{
//		near_road_mid=64;
//		double_crossroad_out_status=0;
//	}
//	//找近线
//	for(i=near_road_mid;i<=117;i++)
//	{
//		if((ccd_data[i]-ccd_data[i+5])>&&ccd_data[i]>420&&)//如果差比和大于标准，灰度值最低 && ccd_data[i]>ccd_data[i+5]
//		{
//			
//			near_flagr=1;//近端右线
//			near_road_right=i;
//			break;
//		}
//		
//	}
//	for(i=near_road_mid;i>=10;i--)
//	{
//		if(cal(ccd_data[i],ccd_data[i-5])>near_standard)//如果差比和大于标准，灰度值最低&& ccd_data[i]>ccd_data[i-5] && ccd_data[i-5]<ccd_data[i] && ccd_data[i-5]<ccd_data[i-10]    && ccd_data[i]>ccd_data[i-10]
//		{
//			near_flagl=1;//近端左线
//			near_road_left=i;
//			break;
//		}
//	}
//	
//	  
//	near_road_mid=(near_road_left+near_road_right)/2;//动态中线
//	
//	
//	
//	
//	
//	//找远线
//	
//	for(i=far_road_mid;i<=117;i++)
//	{
//		if(cal(ccd_data_ch2[i],ccd_data_ch2[i+5])>far_standard)//如果差比和大于标准，说明是右赛道 && ccd_data_ch2[i+5]<ccd_data_ch2[i] && ccd_data_ch2[i+5]<ccd_data_ch2[i+10] && ccd_data_ch2[i]>ccd_data_ch2[i+5]
//		{
//			far_flagr=1;//远端右线
//			far_road_right=i;         
//			break;
//		}
//	}
//	for(i=far_road_mid;i>=5;i--)
//	{
//		if(cal(ccd_data_ch2[i],ccd_data_ch2[i-5])>far_standard)//如果差比和大于标准，说明是左赛道 && ccd_data_ch2[i-5]<ccd_data_ch2[i] && ccd_data_ch2[i-5]<ccd_data_ch2[i-10] && ccd_data_ch2[i]>ccd_data_ch2[i-5]
//		{
//			far_flagl=1;//远端左线
//			far_road_left=i;
//			break;
//		}
//	}
//	far_road_mid=(far_road_left+far_road_right)/2;//动态中线
	//找近线
	for(i=near_road_mid;i<=117;i++)
	{
		if((cal(ccd_data[i],ccd_data[i+5])>near_standard))//如果差比和大于标准，灰度值最低 && ccd_data[i]>ccd_data[i+5]
		{
			
			near_flagr=1;//近端右线
			near_road_right=i;
			break;
		}
		
	}
	for(i=near_road_mid;i>=10;i--)
	{
		if(cal(ccd_data[i],ccd_data[i-5])>near_standard)//如果差比和大于标准，灰度值最低&& ccd_data[i]>ccd_data[i-5] && ccd_data[i-5]<ccd_data[i] && ccd_data[i-5]<ccd_data[i-10]    && ccd_data[i]>ccd_data[i-10]
		{
			near_flagl=1;//近端左线
			near_road_left=i;
			break;
		}
	}
	
	  
	near_road_mid=(near_road_left+near_road_right)/2;//动态中线
	
	
	
	
	
	//找远线
	
	for(i=far_road_mid;i<=117;i++)
	{
		if(cal(ccd_data_ch2[i],ccd_data_ch2[i+5])>far_standard)//如果差比和大于标准，说明是右赛道 && ccd_data_ch2[i+5]<ccd_data_ch2[i] && ccd_data_ch2[i+5]<ccd_data_ch2[i+10] && ccd_data_ch2[i]>ccd_data_ch2[i+5]
		{
			far_flagr=1;//远端右线
			far_road_right=i;         
			break;
		}
	}
	for(i=far_road_mid;i>=5;i--)
	{
		if(cal(ccd_data_ch2[i],ccd_data_ch2[i-5])>far_standard)//如果差比和大于标准，说明是左赛道 && ccd_data_ch2[i-5]<ccd_data_ch2[i] && ccd_data_ch2[i-5]<ccd_data_ch2[i-10] && ccd_data_ch2[i]>ccd_data_ch2[i-5]
		{
			far_flagl=1;//远端左线
			far_road_left=i;
			break;
		}
	}
	far_road_mid=(far_road_left+far_road_right)/2;//动态中线
	far_ccd_error=(far_road_left+far_road_right)/2-64;

	
	if(findflag)//lcd边线打印放缩参数
	{

		if(near_road_right!=127)
		{
			maxdata=ccd_data[near_road_right-5];
			mindata=ccd_data[near_road_right+5];
			maxdatach2=ccd_data_ch2[far_road_right-5];
			mindatach2=ccd_data_ch2[far_road_right+5];
			for(i=0;i<=128;i++)
			{
				linenearavg+=ccd_data[i];
				linefaravg+=ccd_data_ch2[i];
			}
			linenearavg/=128;
			linefaravg/=128;
			findflag--;
		}

	}	 
		 
		 
	if(lcircle_status==1||rcircle_status==1)//时间差存储先前边线信息
	{
		linecount++;
		if(linecount>10)
		{
			prefarleft=far_road_left;
			prefarright=far_road_right;
			linecount=0;
		}
	}
//---------------------------------------------------------元素------------------------------------------------------------------------------------------------------
	//近端边线：37,103  远端边线：45 84  十字备用：||(!near_flagr&&(127-near_road_right>20)&&near_flagl)||(!near_flagl&&(near_road_left-0>=25)&&near_flagr)
	
	//3362154      5216334
	
	if(a[k]==1||a[k]==8)//左圆环，进圆环1，不进8  
	{
		if(!far_flagl && (far_road_left-0>=20) && far_flagr && abs(far_road_right-cir_line1_key)<=5)//左线丢且跳变，右线找到且基本保持不变
		{
			P67=1;
			lcircle_status=1;
			return lCircle;
		}
		else
			return Normal;
	}

	if(a[k]==2||a[k]==9)//右圆环，进圆环2，不进9 
	{
		if(!far_flagr && (127-far_road_right>=20) && far_flagl && abs(far_road_left-cir_line2_key)<=5)//右线丢且跳变，左线找到且基本保持不变
		{
			P67=1;
			rcircle_status=1;
			return rCircle;
		}
		else 
			return Normal;
	}
	
	
	if(a[k]==3||a[k]==10)//十字
	{
		if(!far_flagr&&!far_flagl)//		if(!near_flagl && !near_flagr && abs(ccd_error)<10)//近端左右都丢线
		{
			cross_status=1;
			return Crossroad;
		}
		else 
			return Normal;
	}
	
	if(a[k]==4)//斑马线
	{
		for(i=35;i<=95;i++)
		{
			if(cal(ccd_data[i],ccd_data[i+3])>18)
			{
				zebra_count++;
			}
		}
		if(zebra_count>=20)
		 {
			 P67=1;
			 encoder_flag=1;
			 
		 }	
		 if(encoder_sum>2000)
		 {
			 S_status=1;
			 encoder_flag=0;
			 encoder_sum=0;
		 }

	}

    if(a[k]==5)//障碍物
    {
		if(flag&&near_flagl && near_flagr && abs(near_road_left-46)<5 && abs((near_road_right-near_road_left)-21)<5)//左线不变，右线跳变,右障碍
		{
			P67=1;
			obs_status=1;
			flag=0;
			robs=1;
			return obstacle;
			
		}
		else if(flag && near_flagl && near_flagr && abs(near_road_right-near_road_left-21)<5 && abs(near_road_right-82)<5)//右线不变，左线跳变，左障碍
		{
			P67=1;
			flag=0;
			obs_status=1;
			lobs=1;
			return obstacle;
		}
		else
		{
			return Normal;
		}
			
	}
	
	
	
	if(a[k]==6)//坡道
		{
			if(dl1b_distance_mm<=270 && dl1b_distance_mm>=180)//累计3帧赛道都变宽,tof测距小于500dl1b_distance_mm<500&&near_road_right-near_road_left>45&&near_flagr&&near_flagl&&abs(near_road_right-90)<=5&&abs(near_road_left-40)<=5
			{
				P67=1;
				ramp_status=1;
				return ramp;
			}
			else
				return Normal;
		}

		
	if(a[k]==7||a[k]==8||a[k]==9)
	{
		return Speedup;
	}
	return Normal;
	
}
