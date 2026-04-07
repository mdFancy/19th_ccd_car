#include "headfile.h"

/**
  * @brief      按键初始化
  * @param      无
  * @return     void——无返回值
  */
void KEY_Init(void)                //初始化按键
{                                                                      
	gpio_mode(CONFIG_KEY_CEN_PIN,   GPIO);           //中   54
	gpio_mode(CONFIG_KEY_UP_PIN,   GPIO);       //上    70
	gpio_mode(CONFIG_KEY_DOWN_PIN,   GPIO);       //下    71
	gpio_mode(CONFIG_KEY_LEFT_PIN,   GPIO);      //左    72
	gpio_mode(CONFIG_KEY_RIGHT_PIN,   GPIO);      //右    73
}


uint8 Key_Scan(void)                //扫描按键标志值    下上左右中12345   无0
{
	if(!CONFIG_KEY_DOWN_PIN)	  
	{	
		delay_ms(10);
		while(!CONFIG_KEY_DOWN_PIN);
		delay_ms(10);
		return KeyDown;
		
	}
	if(!CONFIG_KEY_LEFT_PIN)	  
	{	
		delay_ms(10);
		while(!CONFIG_KEY_LEFT_PIN);
		delay_ms(10);
		return KeyLeft;
		
	}
	if(!CONFIG_KEY_RIGHT_PIN)	  
	{	
		delay_ms(10);
		while(!CONFIG_KEY_RIGHT_PIN);
		delay_ms(10);
		return KeyRight;
		
	}
	if(!CONFIG_KEY_UP_PIN)	  
	{	
		delay_ms(10);
		while(!CONFIG_KEY_UP_PIN);
		delay_ms(10);
		return KeyUp;
	}
	if(!CONFIG_KEY_CEN_PIN)	  
	{	
		delay_ms(10);
		while(!CONFIG_KEY_CEN_PIN);
		delay_ms(10);
		return KeyCen;
	}
	
	return KeyNone;
}