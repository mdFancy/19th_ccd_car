#ifndef _ENCODER_H
#define _ENCODER_H
#define PLUSE_LEFT 		CTIM0_P34
#define DIR_LEFT    	P35 
#define PLUSE_RIGHT 	CTIM3_P04
#define DIR_RIGHT    	P53 
int16 encoder_read_left(void);
int16 encoder_read_right(void);
void encoder_integral_test();
#endif

////
//#define PLUSE_RIGHT CTIM3_P04 
//#define PLUSE_LEFT CTIM0_P34 
//#define DIR_LEFT P50
//#define DIR_RIGHT P51