#include "headfile.h"
#include "stdlib.h"
uint8 k=0;
uint8 ramp_flag=0;//坡道标志位
uint8 cross_road_status=0;
int16 last_near_road_right;
int16 last_near_road_left;
int16 half_road_width=17;//需要测 近30 远17
int16 half_road_fwidth=28;
uint8 encoder_flag=0;
uint8 imu_flag=0;
uint8 count1=0,count2=0;
uint8 cross_count=0;
uint8 cross_prohebit=0;

uint16 speedupnum;
extern uint16 speedup1_key;
extern uint16 speedup2_key;
extern uint16 speedup3_key;
//uint8 double_crossroad_status=0;
uint8 lcircle_status=0;
uint8 rcircle_status=0;
extern int16 prefarleft;
extern int16 prefarright;
uint8 c=0;
uint8 circle_out_status=0;
uint8 ramp_out_status=0;
uint8 stop_status=0;
uint8 cross_out_status=0;
//uint8 double_crossroad_out_status=0;
uint8 midnear=64;
uint8 midfar=64;
uint8 cs=0;
extern uint8 cross_status;
extern uint8 obs_status;
extern float left_targetv;
extern float right_targetv;
extern uint16 a[];
extern uint8 flag;
extern float ccd_error;
float last_ccd_error=0;
extern uint8 near_flagl,near_flagr;
extern uint8 far_flagl,far_flagr;
extern int16 near_road_left,near_road_right;
extern int16 far_road_left,far_road_right;
extern int16 near_road_mid;
extern int16 encoder_sum;
extern float imu_sum;
int imu_standard;
int encoder_standard=4800;
extern uint8 cross_road_flag;
extern int16 far_road_left;
extern int16 far_road_right;
extern uint8 ramp_status;
extern uint8 astatus;
extern uint8 S_status;
extern uint16 es_key;
extern uint8 robs;
extern uint8 lobs;
extern Steer_pram steer;





//正常循迹
void normal_road(void)
{

	if(!near_flagr&&near_flagl)
	{
		if(abs(near_road_left-46)<=2)
		{
			ccd_error=(near_road_left+half_road_width-midnear);
		}
		else
		{
			ccd_error=(near_road_left+half_road_width*1.6-midnear);
		}
	}
	else if(!near_flagl&&near_flagr)
	{
		if(abs(near_road_right-80)<=2)
		{
			ccd_error=-(midnear-(near_road_right-half_road_width));
		}
		else
		{
			ccd_error=-(midnear-(near_road_right-half_road_width*1.6));
		}
	}
	else
	{
		ccd_error=((near_road_right+near_road_left)/2-midnear);
		last_ccd_error=ccd_error;
	}
}
void find_crossroad()
{
	if(!far_flagr&&far_flagl)
	{
		ccd_error=(far_road_left+half_road_fwidth-midfar);
	}
	else if(!far_flagl&&far_flagr)
	{
		ccd_error=-(midfar-(far_road_right-half_road_fwidth));
	}
	else if(!far_flagl&&!far_flagr)
	{
		ccd_error=0;
	}
	else
	{
		ccd_error=(far_road_right+far_road_left)/2-midfar;
	}
}
//void find_right_crossroad()
//{
//	if(!far_flagr&&far_flagl)
//	{
//		ccd_error=(far_road_left+half_road_fwidth*1.2-midfar);
//	}
//	else if(!far_flagl&&far_flagr)
//	{
//		ccd_error=-(midfar-(far_road_right-half_road_fwidth*1.2));
//	}
//	else if(!far_flagl&&!far_flagr)
//	{
//		ccd_error=5;
//	}
//	else
//	{
//		ccd_error=(far_road_right+far_road_left)/2-midfar;
//	}
//}
//坡道循迹
void ramproad()
{
	encoder_flag=1;
	ramp_flag=1;
	
	ccd_error=(far_road_right+far_road_left)/2-midfar;//0
	if(encoder_sum>=5500)
	{
		P67=0;
		encoder_flag=0;
		ramp_flag=0;
		ramp_status=0;
		ramp_out_status=1;
		k++;
	}
	
}
void stop()
{
	ccd_error=0;
	P67=1;
}
void obs()//障碍物
{
	encoder_flag=1;
	if(encoder_sum<5000)
	{
		P67=1;
		if(robs==1)
		{
			ccd_error=(near_road_left+(half_road_width-15)-midnear);//左线偏移
		}
		else if(lobs==1)
		{
			ccd_error=-(midnear-(near_road_right-half_road_width+15));//右线
		}
	}
	else
	{
		P67=0;
		encoder_flag=0;
		encoder_sum=0;
		robs=0;
		lobs=0;
		obs_status=0;
		flag=1;
		k++;
	}
	
}

//右圆环循迹
void rcircle(void)
{
	if(a[k]==2)//入圆环
	{
		imu_flag=1;//陀螺仪开始积分
		
	//    P67=1;
		switch(c)
		{
			case(0)://寻外线直行
			{
				encoder_flag=1;
				P67=1;
				if(far_road_right>prefarright&&encoder_sum>3000) c=1;
				
				
				ccd_error=(near_road_left+half_road_width-midnear);//左线
				break;
			}
			case(1)://寻内线入环
			{
				P67=0;
				if(imu_sum>42) c=2;//&&far_flagr&&far_road_right>64
				
				
				ccd_error=-(midfar-(far_road_right-half_road_fwidth*0.8));//换右线
				if(ccd_error>20) ccd_error=20;
				encoder_sum=0;
				encoder_flag=0;
				break;
			}
			case(2)://环内外线
			{
				P67=1;
				if(imu_sum>255) c=3;
				
				
				ccd_error=(far_road_left+half_road_fwidth*1.4-midfar);//左线
				if(ccd_error<0)
				{
					ccd_error=-ccd_error;
				}
				break;
			}
			case(3)://寻内线出环
			{
				P67=0;
				if(near_flagl&&imu_sum>320) c=4;//
				
				near_road_mid=64;
				ccd_error=-(midfar-(far_road_right-half_road_fwidth*0.8));//右线
				break;
			}
			case(4)://寻外线出环
			{
				P67=1;
				encoder_flag=1;//编码器开始积分
				if(encoder_sum>4000&&abs(ccd_error)<7&&near_flagl&&near_flagr)
				{
					c=5;
				}
				
				near_road_mid=64;
				ccd_error=(near_road_left+half_road_width-midnear);//左线
				if(ccd_error<0)
				{
					ccd_error=-ccd_error;
				}
				
				break;
			}
			case(5)://退出圆环
			{
				rcircle_status=0;
				P67=0;
				c=0;
				imu_sum=0;
				encoder_sum=0;
				circle_out_status=1;
				count1=0;
				count2=0;
				encoder_flag=0;
				imu_flag=0;
				k++;
				break;
			}
		}
	}
//	if(a[k]==9)//不入圆环
//	{
//		encoder_flag=1;
//		ccd_error=(near_road_left+half_road_width-midnear);//
//		if(encoder_sum>7000)
//		{

//			rcircle_status=0;
//			P67=0;
//			c=0;
//			imu_sum=0;
//			encoder_sum=0;
//			circle_out_status=1;
//			encoder_flag=0;
//			imu_flag=0;
//			k++;
//		}
//	}
    
}

//左圆环循迹
void lcircle(void)
{
	if(a[k]==1)//进圆环
	{
			imu_flag=1;//陀螺仪开始积分
		
		switch(c)
		{
			case(0)://外线直行
			{
				encoder_flag=1;
				P67=1;
				if(far_road_left<prefarleft&&encoder_sum>3000) c=1;
				
				
				ccd_error=-(midnear-(near_road_right-half_road_width));//换右线
				break;
			}
			case(1)://寻内线入环
			{
				P67=0;
				if(imu_sum<-42) c=2;	
				
				
				ccd_error=(far_road_left+half_road_fwidth*0.8-midfar);//左线
				if(ccd_error<-20) ccd_error=-20;
				encoder_flag=0;
				encoder_sum=0;
				break;
			}
			case(2)://环内外线
			{
				P67=1;
				if(imu_sum<-255) c=3;
				
				
				ccd_error=-(midfar-(far_road_right-half_road_fwidth*1.4));//换右线
				if(ccd_error>0)
				{
					ccd_error=-ccd_error;
				}
				break;
			}
			case(3)://内线出环
			{
				P67=0;
				if(near_flagr&&imu_sum<-320) c=4;//
				
				near_road_mid=64;
				ccd_error=(far_road_left+half_road_fwidth*0.8-midfar);//左线
				break;
			}
			case(4)://寻外线出环直行
			{
				P67=1;
				encoder_flag=1;//编码器开始积分
				if(encoder_sum>4000&&abs(ccd_error)<7&&near_flagl&&near_flagr)
				{
					c=5;
				}						 
				 
				near_road_mid=64;
				ccd_error=-(midnear-(near_road_right-half_road_width));//换右线
				if(ccd_error>0)
				{
					ccd_error=-ccd_error;
				}
				break;
			}
			case(5)://退出状态
			{
				lcircle_status=0;
				P67=0;
				c=0;
				imu_sum=0;
				encoder_sum=0;
				circle_out_status=1;
				count1=0;
				count2=0;
				encoder_flag=0;
				imu_flag=0;
				k++;
				break;
			}
		}
	}
//	if(a[k]==8)//不入圆环
//	{
//		encoder_flag=1;
//		ccd_error=-(midnear-(near_road_right-half_road_width));
//		if(encoder_sum>7000)
//		{
//			

//			lcircle_status=0;
//			P67=0;
//			c=0;
//			imu_sum=0;
//			encoder_sum=0;
//			circle_out_status=1;
//			encoder_flag=0;
//			imu_flag=0;
//			k++;
//		}
//	}
}
//十字循迹
void crossroad()
{
	if(a[k]==3)//十字
	{	
		
		switch(cs)
		{
			case(0)://入十字
			{
				find_crossroad();
				P67=1;
				encoder_flag=1;
		
				if(encoder_sum>3000&&far_flagr&&far_flagl) cs=1;
				
				break;
			}
			case(1)://十字内
			{
				if(!far_flagr&&far_flagl)
				{
					ccd_error=(far_road_left+half_road_fwidth-midfar);
				}
				else if(!far_flagl&&far_flagr)
				{
					ccd_error=-(midfar-(far_road_right-half_road_fwidth));
				}
				else if(!far_flagl&&!far_flagr)
				{
					ccd_error=0;
				}
				else
				{
					ccd_error=(far_road_right+far_road_left)/2-midfar;
				}
				P67=0;
				encoder_flag=0;
				encoder_sum=0;
				if(!far_flagr&&far_flagl&&ccd_error<=5)
				{
					ccd_error=5;
				}
				else if(far_flagr&&!far_flagl&&ccd_error>=-5)
				{
					ccd_error=-5;
				}
				if(!far_flagr&&!far_flagl) cs=2;
				
				break;
			}
			case(2)://出十字
			{
				find_crossroad();
				P67=1;
				encoder_flag=1;
				if(encoder_sum>2000&&far_flagr&&far_flagl) cs=3;//
				break;
			}
			case(3):
			{
				cs=0;
				P67=0;
				cross_status=0;
				cross_out_status=1;
				imu_sum=0;
				encoder_sum=0;
				encoder_flag=0;
				imu_flag=0;
				k++;
				break;
			}
		}	
		
	}
	else if(a[k]==10)//不处理十字
	{
		P67=1;
		encoder_flag=1;
		find_crossroad();
		if(encoder_sum>2000&&far_flagr&&far_flagl)
		{
			 P67=0;
			 cross_status=0;
			 cross_out_status=1;
			 imu_sum=0;
			 encoder_sum=0;
			 encoder_flag=0;
			 imu_flag=0;
			 k++;
		}
	}
}
void speedup()
{
	encoder_flag=1;
	if(a[k]==7)speedupnum=speedup1_key*1000;
	else if(a[k]==8) speedupnum=speedup2_key*1000;
	else speedupnum=speedup3_key*1000;
	if(encoder_sum>speedupnum)
	{
		
		encoder_flag=0;
		encoder_sum=0;
		k++;
	}
}


		
////双十字
//void double_crossroad(void)
//{
//	double_crossroad_status=1;
//	encoder_flag=1;
//	P67=1;
//	if(!far_flagr&&far_flagl)
//	{
//		ccd_error=(far_road_left+half_road_fwidth-midfar);
//	}
//	else if(!far_flagl&&far_flagr)
//	{
//		ccd_error=-(midfar-(far_road_right-half_road_fwidth));
//	}
//	else if(!far_flagl&&!far_flagr)
//	{
//		ccd_error=0;
//	}
//	else
//	{
//		ccd_error=(far_road_right+far_road_left)/2-midfar;
//	}
//	if(encoder_sum>=46000)
//	{
//		P67=0;
//		double_crossroad_status=0;
//		double_crossroad_out_status=1;
//		imu_sum=0;
//		encoder_sum=0;
//		encoder_flag=0;
//		imu_flag=0;
//		k++;
//	}
//}