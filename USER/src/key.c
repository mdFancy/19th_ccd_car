#include "headfile.h"
void key_init(void)
{
	P70=1;
	P71=1;
	P72=1;
	P73=1;
	P75=1;
	P76=1;//IO口设置为高电平
}
uint8 status_scan(void)
{
	if(P75&&P56==0)
	{
		return 1;
	}
	return 0;
}