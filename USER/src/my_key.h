#ifndef __MY_KEY_H__
#define __MY_KEY_H__

#include "headfile.h"

#define CONFIG_KEY_DOWN_PIN  P71
#define CONFIG_KEY_LEFT_PIN  P72
#define CONFIG_KEY_RIGHT_PIN P73
#define CONFIG_KEY_CEN_PIN   P54
#define CONFIG_KEY_UP_PIN    P70
#define KeyDown  	1
#define KeyUp 		2
#define KeyLeft   3
#define KeyRight  4
#define KeyCen  	5
#define KeyNone  	0
void KEY_Init(void);

uint8 Key_Scan(void);

#endif
