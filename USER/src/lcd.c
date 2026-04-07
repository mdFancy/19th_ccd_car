#include "headfile.h"
extern uint16 ccd_data[128];
extern uint16 ccd_data_ch2[128];
extern int16 near_road_left;
extern int16 near_road_right;
extern int16 far_road_left;
extern int16 far_road_right;

extern uint16 maxdata;
extern uint16 mindata;
extern uint16 maxdatach2;
extern uint16 mindatach2;

extern uint16 scan_key;
extern uint32 linenearavg;
extern uint32 linefaravg;
extern uint8 near_flagl,near_flagr;
extern uint8 far_flagl,far_flagr;
extern int16 encoder_sum;
extern float imu_sum;
int16 old_ccd_data[128]={0};
int16 old_ccd_data_ch2[128]={0};
uint8 lastlx=0;
uint8 lastrx=0;
uint8 j=0;

uint16 reflect_data(uint16 a)
{
	return (a-mindata)*128/(maxdata-mindata)/20*20;
}
uint16 reflect_data_ch2(uint16 a)
{
	return (a-mindatach2)*128/(maxdatach2-mindatach2)/20*20;
}

//127 159
void lcd()
{

	int16 x=0,y=0;	
	lcd_showstr(0,0,"nL");
	lcd_showstr(0,1,"nR");
	lcd_showstr(0,2,"fL");
	lcd_showstr(0,3,"fR");
	lcd_showstr(0,4,"nA");
	lcd_showstr(0,5,"fA");
	
	lcd_showint16(7,0,near_road_left);
	lcd_showint16(7,1,near_road_right);
	lcd_showint16(7,2,far_road_left);
	lcd_showint16(7,3,far_road_right);
	lcd_showint32(7,4,linenearavg,4);
	lcd_showint32(7,5,linefaravg,4);
	if(tsl1401_finish_flag)//接收后flag会置1
	{
		tsl1401_finish_flag=0;
		if(scan_key==1)
		{
			for(x=0;x<128;x++)
			{
				y=ccd_data[x];
				
				if(y!=old_ccd_data[x])//如果新的点不等于旧的点，黑色覆盖原来的点，红色重新画点，更新old_ccd_data
				{
					lcd_drawpoint(x+32,reflect_data(old_ccd_data[x]),BLACK);
					lcd_drawpoint(x+32,reflect_data(y),RED);
					old_ccd_data[x]=y;
				}
				if(x==near_road_left)
				{
					for(j=0;j<128;j++)
					{
						lcd_drawpoint(lastlx+32,j,BLACK);
					}

					for(j=0;j<128;j++)
					{
						lcd_drawpoint(x+32,j,GREEN);
					}
					lastlx=x;
				}
				if(x==near_road_right)
				{
					for(j=0;j<128;j++)
					{
						lcd_drawpoint(lastrx+32,j,BLACK);
					}
					for(j=0;j<128;j++)
					{
						lcd_drawpoint(x+32,j,WHITE);
					}
					lastrx=x;
				}
			}
		}
		else if(scan_key==2)
		{
			for(x=0;x<128;x++)
			{
				y=ccd_data_ch2[x];
				
				if(y!=old_ccd_data_ch2[x])//如果新的点不等于旧的点，黑色覆盖原来的点，红色重新画点，更新old_ccd_data
				{
					lcd_drawpoint(x+32,reflect_data_ch2(old_ccd_data_ch2[x]),BLACK);
					lcd_drawpoint(x+32,reflect_data_ch2(y),RED);
					old_ccd_data_ch2[x]=y;
				}
				if(x==far_road_left)
				{
					for(j=0;j<128;j++)
					{
						lcd_drawpoint(lastlx+32,j,BLACK);
					}

					for(j=0;j<128;j++)
					{
						lcd_drawpoint(x+32,j,GREEN);
					}
					lastlx=x;
				}
				if(x==far_road_right)
				{
					for(j=0;j<128;j++)
					{
						lcd_drawpoint(lastrx+32,j,BLACK);
					}
					for(j=0;j<128;j++)
					{
						lcd_drawpoint(x+32,j,WHITE);
					}
					lastrx=x;
				}
			}
		}
		
	}  
}
void debugging()
{
	lcd_showstr(0,0,"flflag");
	lcd_showstr(0,1,"frflag");
	lcd_showstr(0,2,"nlflag");
	lcd_showstr(0,3,"nrflag");
	lcd_showstr(0,4,"encoder");
	lcd_showstr(0,5,"imu");
	lcd_showstr(0,6,"nL");
	lcd_showstr(0,7,"nR");
	lcd_showstr(0,8,"fL");
	lcd_showstr(0,9,"fR");

	
	lcd_showuint8(39,0,far_flagl);
	lcd_showuint8(39,1,far_flagr);
	lcd_showuint8(39,2,near_flagl);
	lcd_showuint8(39,3,near_flagr);
	lcd_showuint16(39,4,encoder_sum);
	lcd_showfloat(39,5,imu_sum,3,2);
	lcd_showint16(39,6,near_road_left);
	lcd_showint16(39,7,near_road_right);
	lcd_showint16(39,8,far_road_left);
	lcd_showint16(39,9,far_road_right);

}