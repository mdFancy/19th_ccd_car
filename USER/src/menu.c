#include "Menu.h"

void Menu_Null();
void Menu_Motor();
void Menu_Speed();
void Menu_ADC();
void Menu_ADC_Max();
void Menu_ADC_Min();
void Menu_ADvalue();
void Menu_Round();
void Menu_RoundType();
void Menu_Servo();
void Menu_Type();
void Menu_Thre();
void Menu_Barrier();
void Menu_BarrierType();
void Menu_Buzz();

void EEPROM_Read();
void EEPROM_Write();

void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);
void Menu_ProcessPro(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor);
uint8 Menu_Move(MENU_PRMT *prmt, uint8 key);
void adjustParam(Site_t site, uint16 *param, uint16 BGcolor, uint16 PENcolor);

//void EEPROM_init()

uint16 xdata EEPROM_Data[103] = {0};

void EEPROM_Write() //102 下个103
{                   
	EEPROM_Data[0]  =  speedL.Kp;
	EEPROM_Data[1]  =  speedL.Ki;
	EEPROM_Data[2]  =  speedL.Kd;
	
	EEPROM_Data[3]  =  speedL.setspeed_basic;
	EEPROM_Data[26] =  speedL.setspeed_round;
	EEPROM_Data[62]	= speedL.setspeed_endready;
	EEPROM_Data[63] = speedL.setspeed_straight;
	EEPROM_Data[68] =  speedL.setspeed_ramp;
	EEPROM_Data[71] =  speedL.setspeed_barrier;
	EEPROM_Data[75] =  speedL.setspeed_end;
	EEPROM_Data[76] =  speedL.setspeed_brake;
	EEPROM_Data[77] =  brake_delay_num;
	
	EEPROM_Data[4]  =  L_Hor_Max;
	EEPROM_Data[5]  =  L_Ver_Max;
	EEPROM_Data[6]  =  LB_Hor_Max;
	EEPROM_Data[7]  =  LB_Ver_Max;
	EEPROM_Data[8]  =  RB_Ver_Max;
	EEPROM_Data[9]  =  RB_Hor_Max;
	EEPROM_Data[10] =  R_Ver_Max;
	EEPROM_Data[11] =  R_Hor_Max;
	
	EEPROM_Data[12] =  L_Hor_Min;
	EEPROM_Data[13] =  L_Ver_Min;
	EEPROM_Data[14] =  LB_Hor_Min;
	EEPROM_Data[15] =  LB_Ver_Min;
	EEPROM_Data[16] =  RB_Ver_Min;
	EEPROM_Data[17] =  RB_Hor_Min;
	EEPROM_Data[18] =  R_Ver_Min;
	EEPROM_Data[19] =  R_Hor_Min;
	
	EEPROM_Data[20] =  follow.distance_threshold1;
	EEPROM_Data[21] =  follow.distance_threshold2;
	EEPROM_Data[22] =  follow.distance_threshold3;
	EEPROM_Data[23] =  follow.inround_threshold;
	EEPROM_Data[24] =  follow.outround_threshold;
	EEPROM_Data[25] =  follow.round_dir;

	EEPROM_Data[27] = servo.Kp;
	EEPROM_Data[28] = servo.Kd;
	EEPROM_Data[29] = servo.Bend_Kp;
	EEPROM_Data[30] = servo.Bend_Kd;

	EEPROM_Data[64]	= servo.gyro_Kp;
	EEPROM_Data[65] = servo.gyro_Bend1_Kp;
	EEPROM_Data[69] = servo.gyro_Bend2_Kp;
	
	//EEPROM_Data[81]	= follow.barrier_dir;
	EEPROM_Data[82] = start_type;
	EEPROM_Data[73] = end_type;
	EEPROM_Data[83] = new_type;
	EEPROM_Data[86] = zebra_type[0];
	EEPROM_Data[87] = zebra_type[1];
	EEPROM_Data[88] = zebra_type[2];
	EEPROM_Data[89] = zebra_type[3];
	EEPROM_Data[90] = zebra_type[4];
	EEPROM_Data[91] = zebra_type[5];
	EEPROM_Data[97] = out_type;
	EEPROM_Data[98] = out_delay1;
	EEPROM_Data[99] = out_delay2;
	EEPROM_Data[100] = endready_type;
	EEPROM_Data[101] = endready_delay;

	EEPROM_Data[70] = follow.ramp_threshold1;
	EEPROM_Data[80] = follow.ramp_threshold2;
	EEPROM_Data[31] = follow.barr_dis_threshold;

	EEPROM_Data[67] = follow.tof_dis_threshold1;
	EEPROM_Data[79] = follow.tof_dis_threshold2;
	
	EEPROM_Data[32] = follow.barr_dir_threshold1;
	EEPROM_Data[33] = follow.barr_dis_threshold1;
	EEPROM_Data[34] = follow.barr_dir_threshold2;
	EEPROM_Data[35] = follow.barr_dis_threshold2;
	EEPROM_Data[36] = follow.barr_dir_threshold3;
	EEPROM_Data[37] = follow.barr_dis_threshold3;

	EEPROM_Data[61] = follow.start_dis_threshold;

	EEPROM_Data[38] = follow.end_dis_threshold1;
	EEPROM_Data[60] = follow.end_dis_threshold2;		 	
	EEPROM_Data[39] = follow.endready_dis_threshold1;
	EEPROM_Data[40] = follow.endready_dis_threshold2;
	
	EEPROM_Data[93] = follow.loseline_threshold;
	EEPROM_Data[94] = follow.protect_threshold;		 	
	EEPROM_Data[95] = follow.bend_threshold;
	EEPROM_Data[96] = follow.ramp_gyro_threshold;

	EEPROM_Data[41] = round_type[0];
	EEPROM_Data[42] = round_type[1];
	EEPROM_Data[43] = round_type[2];
	EEPROM_Data[44] = round_type[3];
	EEPROM_Data[45] = round_type[4];
	EEPROM_Data[46] = round_type[5];

	EEPROM_Data[47] = barrier_type[0];
	EEPROM_Data[48] = barrier_type[1];
	EEPROM_Data[49] = barrier_type[2];
	EEPROM_Data[50] = barrier_type[3];
	EEPROM_Data[51] = barrier_type[4];
	EEPROM_Data[52] = barrier_type[5];
	EEPROM_Data[66] = barrier_type[6];
	EEPROM_Data[84] = barrier_type[7];
	EEPROM_Data[85] = barrier_type[8];
	EEPROM_Data[81] = barrier_type[9];
	EEPROM_Data[102] = follow.barrierreal_num;

	EEPROM_Data[53] = buzz[0];
	EEPROM_Data[54] = buzz[1];
	EEPROM_Data[55] = buzz[2];
	EEPROM_Data[56] = buzz[3];
	EEPROM_Data[57] = buzz[4];
	EEPROM_Data[58] = buzz[5];
	EEPROM_Data[59] = buzz[6];
	//EEPROM_Data[60] = buzz[7];
	EEPROM_Data[72] = buzz[8];
	EEPROM_Data[74] = buzz[9];
	EEPROM_Data[92] = buzz[10];

}

void EEPROM_Read() //102 下个103
{                   
	speedL.Kp = EEPROM_Data[0];
	speedL.Ki = EEPROM_Data[1];
	speedL.Kd = EEPROM_Data[2];
	
	speedL.setspeed_basic 		= EEPROM_Data[3];
	speedL.setspeed_round 		= EEPROM_Data[26];
	speedL.setspeed_endready 			= EEPROM_Data[62];
	speedL.setspeed_straight 	= EEPROM_Data[63];
	speedL.setspeed_ramp 		= EEPROM_Data[68];
	speedL.setspeed_barrier 		= EEPROM_Data[71];
	speedL.setspeed_end 		= EEPROM_Data[75];
	speedL.setspeed_brake 		= EEPROM_Data[76];
	brake_delay_num 		= EEPROM_Data[77];

	
	L_Hor_Max = EEPROM_Data[4];
	L_Ver_Max = EEPROM_Data[5];
	LB_Hor_Max = EEPROM_Data[6];
	LB_Ver_Max = EEPROM_Data[7];
	RB_Ver_Max = EEPROM_Data[8];
	RB_Hor_Max = EEPROM_Data[9];
	R_Ver_Max = EEPROM_Data[10];
	R_Hor_Max = EEPROM_Data[11];
	
	L_Hor_Min = EEPROM_Data[12];
	L_Ver_Min = EEPROM_Data[13];
	LB_Hor_Min = EEPROM_Data[14];
	LB_Ver_Min = EEPROM_Data[15];
	RB_Ver_Min = EEPROM_Data[16];
	RB_Hor_Min = EEPROM_Data[17];
	R_Ver_Min = EEPROM_Data[18];
	R_Hor_Min = EEPROM_Data[19];
	
	follow.distance_threshold1 = EEPROM_Data[20];
	follow.distance_threshold2 = EEPROM_Data[21];
	follow.distance_threshold3 = EEPROM_Data[22];
	follow.inround_threshold = EEPROM_Data[23];
	follow.outround_threshold = EEPROM_Data[24];
	follow.round_dir = EEPROM_Data[25];

	servo.Kp 			= EEPROM_Data[27];
	servo.Kd 			= EEPROM_Data[28];
	servo.Bend_Kp 		= EEPROM_Data[29];
	servo.Bend_Kd 		= EEPROM_Data[30];

	servo.gyro_Kp 		= EEPROM_Data[64];
	servo.gyro_Bend1_Kp 	= EEPROM_Data[65];
	servo.gyro_Bend2_Kp 	= EEPROM_Data[69];
	
	start_type = EEPROM_Data[82];
	end_type = EEPROM_Data[73];
	new_type = EEPROM_Data[83];
	zebra_type[0] = EEPROM_Data[86];
	zebra_type[1] = EEPROM_Data[87];
	zebra_type[2] = EEPROM_Data[88];
	zebra_type[3] = EEPROM_Data[89];
	zebra_type[4] = EEPROM_Data[90];
	zebra_type[5] = EEPROM_Data[91];
	out_type = EEPROM_Data[97];
	out_delay1 = EEPROM_Data[98];
	out_delay2 = EEPROM_Data[99];
	endready_type = EEPROM_Data[100];
	endready_delay = EEPROM_Data[101];

	follow.ramp_threshold1 = EEPROM_Data[70];
	follow.ramp_threshold2 = EEPROM_Data[80];
	follow.barr_dis_threshold = EEPROM_Data[31];

	follow.tof_dis_threshold1 = EEPROM_Data[67];
	follow.tof_dis_threshold2 = EEPROM_Data[79];
	
	follow.barr_dir_threshold1 = EEPROM_Data[32];
	follow.barr_dis_threshold1 = EEPROM_Data[33];
	follow.barr_dir_threshold2 = EEPROM_Data[34];
	follow.barr_dis_threshold2 = EEPROM_Data[35];
	follow.barr_dir_threshold3 = EEPROM_Data[36];
	follow.barr_dis_threshold3 = EEPROM_Data[37];

	follow.start_dis_threshold = EEPROM_Data[61];
	
	follow.end_dis_threshold1 		= EEPROM_Data[38];
	follow.end_dis_threshold2 		= EEPROM_Data[60];
	follow.endready_dis_threshold1 	= EEPROM_Data[39];
	follow.endready_dis_threshold2 	= EEPROM_Data[40];
	
	follow.loseline_threshold = EEPROM_Data[93];
	follow.protect_threshold = EEPROM_Data[94];
	follow.bend_threshold 	= EEPROM_Data[95];
	follow.ramp_gyro_threshold 	= EEPROM_Data[96];

	round_type[0] = EEPROM_Data[41];
	round_type[1] = EEPROM_Data[42];
	round_type[2] = EEPROM_Data[43];
	round_type[3] = EEPROM_Data[44];
	round_type[4] = EEPROM_Data[45];
	round_type[5] = EEPROM_Data[46];

	barrier_type[0] = EEPROM_Data[47];
	barrier_type[1] = EEPROM_Data[48];
	barrier_type[2] = EEPROM_Data[49];
	barrier_type[3] = EEPROM_Data[50];
	barrier_type[4] = EEPROM_Data[51];
	barrier_type[5] = EEPROM_Data[52];
	barrier_type[6] = EEPROM_Data[66];
	barrier_type[7] = EEPROM_Data[84];
	barrier_type[8] = EEPROM_Data[85];
	barrier_type[9] = EEPROM_Data[81];
	follow.barrierreal_num = EEPROM_Data[102];

	buzz[0] = EEPROM_Data[53];
	buzz[1] = EEPROM_Data[54];
	buzz[2] = EEPROM_Data[55];
	buzz[3] = EEPROM_Data[56];
	buzz[4] = EEPROM_Data[57];
	buzz[5] = EEPROM_Data[58];
	buzz[6] = EEPROM_Data[59];
	//buzz[7] = EEPROM_Data[60];
	buzz[8] = EEPROM_Data[72];
	buzz[9] = EEPROM_Data[74];
	buzz[10] = EEPROM_Data[92];


}

//----------------------------------   主菜单   -------------------------------

// 一级菜单
MENU_PRMT MainMenu_Prmt;

MENU_TABLE MainMenu_Table[] = 
{

	{"1.MotorDebug         ", Menu_Motor, NULL},
    
	{"2.SpeedParam         ", Menu_Speed, NULL}, 

 	{"3.Adc_Data           ", Menu_ADC  , NULL},
	
	{"4.Adc_Max            ", Menu_ADC_Max, NULL},
	
	{"5.Adc_Min            ", Menu_ADC_Min, NULL},
	
	{"6.ADvalue            ", Menu_ADvalue, NULL},
	
	{"7.RoundDebug         ", Menu_Round  , NULL},
	
	{"8.RoundType          ", Menu_RoundType, NULL},

  {"9.ServoDebug         ", Menu_Servo  , NULL},
	
	{"10.TypeParam          ", Menu_Type, NULL}, 
	
	{"11.ThreParam         ", Menu_Thre, NULL}, 
	
	{"12.BarrierDebug      ", Menu_Barrier, NULL},

	{"13.BarrierType       ", Menu_BarrierType, NULL},
	
	{"14.BuzzType				   ", Menu_Buzz, NULL},


};

// 二级菜单
MENU_PRMT Motor_Prmt;

MENU_TABLE Motor_MenuTable[] = 
{
	{"1.MOTOR_KP          ", Menu_Null, &speedL.Kp},

	{"2.MOTOR_KI          ", Menu_Null, &speedL.Ki},

	{"3.MOTOR_KD          ", Menu_Null, &speedL.Kd},
};


MENU_PRMT Speed_Prmt;

MENU_TABLE Speed_MenuTable[] = 
{
	{"speed_basic          ", Menu_Null, &speedL.setspeed_basic},
	
	{"speed_round          ", Menu_Null, &speedL.setspeed_round},

	{"speed_endready              ", Menu_Null, &speedL.setspeed_endready},

	{"speed_straight       ", Menu_Null, &speedL.setspeed_straight},	
	
	{"speed_ramp           ", Menu_Null, &speedL.setspeed_ramp},
	
	{"speed_barr          ", Menu_Null, &speedL.setspeed_barrier},
	
	{"speed_end       ", Menu_Null, &speedL.setspeed_end},
	
	{"speed_brake          ", Menu_Null, &speedL.setspeed_brake},
	
	{"brake_delay          ", Menu_Null, &brake_delay_num},

};


MENU_PRMT ADC_Max_Prmt;

MENU_TABLE ADC_Max_MenuTable[] = 
{
	{"1.L_Hor          ", Menu_Null, &L_Hor_Max},

	{"2.L_Ver          ", Menu_Null, &L_Ver_Max},

	{"3.LB_Hor         ", Menu_Null, &LB_Hor_Max},
	
	{"4.LB_Ver         ", Menu_Null, &LB_Ver_Max},

	{"5.RB_Ver         ", Menu_Null, &RB_Ver_Max},
	
	{"6.RB_Hor         ", Menu_Null, &RB_Hor_Max},

	{"7.R_Ver          ", Menu_Null, &R_Ver_Max},

	{"8.R_Hor          ", Menu_Null, &R_Hor_Max},
};


MENU_PRMT ADC_Min_Prmt;

MENU_TABLE ADC_Min_MenuTable[] = 
{
	{"1.L_Hor          ", Menu_Null, &L_Hor_Min},

	{"2.L_Ver          ", Menu_Null, &L_Ver_Min},

	{"3.LB_Hor         ", Menu_Null, &LB_Hor_Min},
	
	{"4.LB_Ver         ", Menu_Null, &LB_Ver_Min},

	{"5.RB_Ver         ", Menu_Null, &RB_Ver_Min},
	
	{"6.RB_Hor         ", Menu_Null, &RB_Hor_Min},

	{"7.R_Ver          ", Menu_Null, &R_Ver_Min},

	{"8.R_Hor          ", Menu_Null, &R_Hor_Min},
};

MENU_PRMT Round_Prmt;

MENU_TABLE Round_MenuTable[] = 
{
	{"1.distance1            ", Menu_Null, &follow.distance_threshold1},

	{"2.distance2            ", Menu_Null, &follow.distance_threshold2},

	{"3.distance3            ", Menu_Null, &follow.distance_threshold3},
	
	{"4.inround              ", Menu_Null, &follow.inround_threshold},

	{"5.outround             ", Menu_Null, &follow.outround_threshold},
	
	{"6.DirPWM               ", Menu_Null, &follow.round_dir},
		
	{"7.speed                ", Menu_Null, &speedL.setspeed_round},
	
};


MENU_PRMT RoundType_Prmt;

MENU_TABLE RoundType_MenuTable[] = 
{
	{"1.round1            ", Menu_Null, &round_type[0]},

	{"2.round2            ", Menu_Null, &round_type[1]},

	{"3.round3            ", Menu_Null, &round_type[2]},
	
	{"4.round4            ", Menu_Null, &round_type[3]},

	{"5.round5            ", Menu_Null, &round_type[4]},

	{"6.round6            ", Menu_Null, &round_type[5]},

};

MENU_PRMT Servo_Prmt;

MENU_TABLE Servo_MenuTable[] = 
{
	{"1.SERVO_KP          ", Menu_Null, &servo.Kp},

	{"2.SERVO_KD          ", Menu_Null, &servo.Kd},
	
	{"3.BEND_KP           ", Menu_Null, &servo.Bend_Kp},

	{"4.BEND_KD           ", Menu_Null, &servo.Bend_Kd},
	
	{"5.gyro_Kp            ", Menu_Null, &servo.gyro_Kp},

	{"6.gyro_bend1      ", Menu_Null, &servo.gyro_Bend1_Kp},
	
	{"7.gyro_bend2       ", Menu_Null, &servo.gyro_Bend2_Kp},

};

MENU_PRMT Type_Prmt;

MENU_TABLE Type_MenuTable[] = 
{		
	{"1.start_type          ", Menu_Null, &start_type  },
	
	{"2.end_type            ", Menu_Null, &end_type  },
	
	{"3.new_type            ", Menu_Null, &new_type  },
	
	{"4.zebra1            ", Menu_Null, &zebra_type[0]},

	{"5.zebra2            ", Menu_Null, &zebra_type[1]},

	{"6.zebra3            ", Menu_Null, &zebra_type[2]},
	
	{"7.zebra4            ", Menu_Null, &zebra_type[3]},

	{"8.zebra5            ", Menu_Null, &zebra_type[4]},

	{"9.zebra6            ", Menu_Null, &zebra_type[5]},

	{"10.out_type            ", Menu_Null, &out_type  },

	{"11.out_delay1            ", Menu_Null, &out_delay1  },
	
	{"12.out_delay2            ", Menu_Null, &out_delay2  },

	{"13.endr_type            ", Menu_Null, &endready_type  },

	{"14.endr_delay            ", Menu_Null, &endready_delay  },
};

MENU_PRMT Barrier_Prmt;

MENU_TABLE Barrier_MenuTable[] = 
{
	{"1.tof_dis1            ", Menu_Null, &follow.tof_dis_threshold1},

	{"2.tof_dis2             ", Menu_Null, &follow.tof_dis_threshold2},

	{"7.barr_dir1            ", Menu_Null, &follow.barr_dir_threshold1},
	
	{"8.barr_dis1            ", Menu_Null, &follow.barr_dis_threshold1},
	
	{"9.barr_dir2             ", Menu_Null, &follow.barr_dir_threshold2},

	{"10.barr_dis2             ", Menu_Null, &follow.barr_dis_threshold2},
	
	{"11.barr_dir3            ", Menu_Null, &follow.barr_dir_threshold3},
	
	{"12.barr_dis3            ", Menu_Null, &follow.barr_dis_threshold3},
		
	{"13.ramp_dis1             ", Menu_Null, &follow.ramp_threshold1},
	
	{"13.ramp_dis2             ", Menu_Null, &follow.ramp_threshold2},
	
	{"14.barr_dis            ", Menu_Null, &follow.barr_dis_threshold},
	
};

MENU_PRMT BarrierType_Prmt;

MENU_TABLE BarrierType_MenuTable[] = 
{
	{"1.barrier1            ", Menu_Null, &barrier_type[0]},

	{"2.barrier2            ", Menu_Null, &barrier_type[1]},

	{"3.barrier3            ", Menu_Null, &barrier_type[2]},
	
	{"4.barrier4            ", Menu_Null, &barrier_type[3]},

	{"5.barrier5            ", Menu_Null, &barrier_type[4]},

	{"6.barrier6            ", Menu_Null, &barrier_type[5]},

	{"7.barrier7            ", Menu_Null, &barrier_type[6]},

	{"8.barrier8            ", Menu_Null, &barrier_type[7]},

	{"9.barrier9            ", Menu_Null, &barrier_type[8]},
	
	{"10.barrier10            ", Menu_Null, &barrier_type[9]},

	{"11.barrrealnum            ", Menu_Null, &follow.barrierreal_num},

	//{"12.barrier12            ", Menu_Null, &barrier_type[11]},
};

MENU_PRMT Thre_Prmt;

MENU_TABLE Thre_MenuTable[] = 
{
	{"1.loseline            ", Menu_Null, &follow.loseline_threshold},

	{"2.protect             ", Menu_Null, &follow.protect_threshold},

	{"3.endr_dis1            ", Menu_Null, &follow.endready_dis_threshold1},
	
	{"4.endr_dis2            ", Menu_Null, &follow.endready_dis_threshold2},
	
	{"5.end_dis1              ", Menu_Null, &follow.end_dis_threshold1},
	
	{"5.end_dis2              ", Menu_Null, &follow.end_dis_threshold2},
	
	{"6.bend2                ", Menu_Null, &follow.bend_threshold},
	
	{"7.ramp_gyro                ", Menu_Null, &follow.ramp_gyro_threshold},

	
};

MENU_PRMT Buzz_Prmt;

MENU_TABLE Buzz_MenuTable[] = 
{

	{"1.barrier            ", Menu_Null, &buzz[0]},

	{"2.bend1            ", Menu_Null, &buzz[1]},

	{"3.bend2            ", Menu_Null, &buzz[2]},
	
	{"4.round            ", Menu_Null, &buzz[3]},

	{"5.inround            ", Menu_Null, &buzz[4]},

	{"6.outround            ", Menu_Null, &buzz[5]},
	
	{"7.loseline            ", Menu_Null, &buzz[6]},

	{"8.ramp_new            ", Menu_Null, &buzz[7]},
	
	{"9.zebra            ", Menu_Null, &buzz[8]},

	{"10.end            ", Menu_Null, &buzz[9]},
	
	{"11.bend0            ", Menu_Null, &buzz[10]},
	
};

/******************************************************************************
* FunctionName   : MainMenu_Set()
* Description    : 常规设置
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void MainMenu_Set(void)
{
  uint8 menuNum = sizeof(MainMenu_Table)/sizeof(MainMenu_Table[0]);    // 菜单项数

  DisableGlobalIRQ();

  if(P53 == 0) //拨码开关在上
  {
	MY_iap_read_bytes(0x00, EEPROM_Data, sizeof(EEPROM_Data)/sizeof(uint16));	
	EEPROM_Read();
  }
	
  lcd_clear(BLACK);

  Menu_Process(" -= Setting =- ", &MainMenu_Prmt, MainMenu_Table, menuNum);
	  	
	EEPROM_Write();
	
	MY_extern_iap_write_bytes(0x00, EEPROM_Data, sizeof(EEPROM_Data)/sizeof(uint16));

  lcd_clear(BLACK);

	speedR.setspeed =	speedL.setspeed;
	speedR.setspeed_basic=speedL.setspeed_basic;
	speedR.setspeed_round=speedL.setspeed_round;
	speedR.setspeed_endready=speedL.setspeed_endready;
	speedR.setspeed_straight=speedL.setspeed_straight;
	speedR.setspeed_ramp=speedL.setspeed_ramp;
	speedR.setspeed_barrier=speedL.setspeed_barrier;
	speedR.setspeed_end=speedL.setspeed_end;
	speedR.setspeed_brake=speedL.setspeed_brake;
	
  EnableGlobalIRQ();
}

/******************************************************************************
* FunctionName   : Menu_PrmtInit()
* Description    : 初始化菜单参数
* EntryParameter : prmt - 菜单参数, num - 每页显示项数, page - 最大显示页数
* ReturnValue    : None
*******************************************************************************/
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page)
{
  prmt->ExitMark = 0;       //清除退出菜单标志      
  prmt->Cursor   = 0;       //光标清零
  prmt->PageNo   = 0;       //页清零
  prmt->Index    = 0;       //索引清零
  prmt->DispNum  = num;     //页最多显示项目数
  prmt->MaxPage  = page;    //最多页数
}

/******************************************************************************
* FunctionName   : Menu_Display()
* Description    : 显示菜单项
* EntryParameter : menuTable - 显示页，dispNum - 每一页的显示项数，cursor - 光标位置
* ReturnValue    : None
*******************************************************************************/
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor)
{
  uint8 i;
  uint16 num_t;
  Site_t site;
  for (i=0; i<dispNum; i++)
  {	
    if (cursor == i)
    {
      /* 蓝底显示当前光标选中菜单项 */
      site.x = 0;    site.y = (i+1);
      lcd_MYshowstr(site.x,site.y,menuTable[pageNo+i].MenuName, BLUE, WHITE);
      /* 若此菜单项有需要调的参数，则显示该参数 */
      if(menuTable[pageNo+i].DebugParam != NULL)
      {
        site.x = TFT_X_MAX-NUM;  //数字坐标
        num_t = (*(menuTable[pageNo+i].DebugParam));
        lcd_MYshowuint16(site.x,site.y,num_t, BLUE, WHITE);
      }
    }
    else
    {
      /* 正常显示其余菜单项 */
      site.x = 0;    site.y = (i+1);
      lcd_MYshowstr(site.x,site.y,menuTable[pageNo+i].MenuName, BLACK, WHITE);
      /* 若此菜单项有需要调的参数，则显示该参数 */
      if(menuTable[pageNo+i].DebugParam != NULL)
      {
        site.x = TFT_X_MAX-NUM;
        num_t = (*(menuTable[pageNo+i].DebugParam));
        lcd_MYshowuint16(site.x,site.y,num_t, BLACK, WHITE);
      }
    }
  }
}

/******************************************************************************
* FunctionName   : Menu_Move()
* Description    : 菜单移动
* EntryParameter :  prmt - 菜单参数, key - 按键值
* ReturnValue    : 有确认返回0，否则返回1
******************************************************************************/
uint8 Menu_Move(MENU_PRMT *prmt, uint8 key)
{
  uint8 rValue = 1;
  switch (key) 
  {
    case KeyUp:                   // 向上
    { 
      if (prmt->Cursor != 0)              // 光标不在顶端
      {
        prmt->Cursor--;                 // 光标上移
      }
      else                                // 光标在顶端
      {
        if (prmt->PageNo != 0)          // 页面没有到最小
        {
          prmt->PageNo--;             // 向上翻
        }
        else
        {
          prmt->Cursor = prmt->DispNum-1;    // 光标到底
          prmt->PageNo = prmt->MaxPage-1;    // 最后页
        }
      }
      break;
    }
    
    case KeyDown:                   // 向下
    {
      if (prmt->Cursor < prmt->DispNum-1)        // 光标没有到底，移动光标
      {
        prmt->Cursor++;                        // 光标向下移动
      }
      else                                       // 光标到底
      {
        if (prmt->PageNo < prmt->MaxPage-1)    // 页面没有到底，页面移动
        {
          prmt->PageNo++;                    // 下翻一页
        }
        else                                   // 页面和光标都到底，返回开始页
        {
          prmt->Cursor = 0;
          prmt->PageNo = 0;
        }
      }
      break;
    }
    
    case KeyCen:                   // 中键确认
    { 
      prmt->Index = prmt->Cursor + prmt->PageNo;   //计算执行项的索引 
      rValue = 0;        
      break;
    }
    
    case KeyLeft:                   // 左键返回上级菜单
    {
      //prmt->Cursor = 0;
      //prmt->PageNo = 0;
      prmt->ExitMark = 1;              
      break;
    }

    case KeyRight:                   // 右键跳到底部
    {
      prmt->Cursor = prmt->DispNum-1;             // 光标到底
      prmt->PageNo = prmt->MaxPage-1;             // 最后页
      break;
    }
    default:break;
	} 
  	return rValue;                    // 返回执行索引
}

void adjustParam(Site_t site, uint16 *param, uint16 BGcolor, uint16 PENcolor)
{
  uint8 key = KeyNone;
  uint16 e = 1;
  do{
      key = Key_Scan();
      switch(key)
      {
        case(KeyUp):
          (*param)+=e;
          lcd_MYshowuint16(site.x,site.y, (uint16)(*param), BGcolor, PENcolor);
          break;            
        case(KeyDown):
          (*param)-=e;
          lcd_MYshowuint16(site.x,site.y, (uint16)(*param), BGcolor, PENcolor);
          break;
        case(KeyLeft):
          e /= 10;

          lcd_MYshowuint16(site.x,site.y, (uint16)(*param), BGcolor, PENcolor);
          break;
        case(KeyRight):
          e *= 10;
          lcd_MYshowuint16(site.x,site.y, (uint16)(*param), BGcolor, PENcolor);
          break;
        default: break;       
      }
	  lcd_MYshowuint16(96, 0, e, BLACK, WHITE);
	  if(e<1)	e=1;
      if(*param <= 0)  *param = 0; 
      if(*param >= 65535) *param = 0;
  }while(key != KeyCen);
}

/******************************************************************************
* FunctionName   : Menu_Process()
* Description    : 处理菜单项
* EntryParameter : menuName - 菜单名称，prmt - 菜单参数，table - 菜单表项, num - 菜单项数
* ReturnValue    : None
******************************************************************************/
void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)
{
  uint8 key = KeyNone;
  Site_t site;
  uint8 page;    //显示菜单需要的页数
  
  if(num - PAGE_DISP_NUM <= 0)
    page = 1;
  else
  {
    page = num - PAGE_DISP_NUM + 1;
    num = PAGE_DISP_NUM;
  }
  // 显示项数和页数设置
  Menu_PrmtInit(prmt, num, page); 
  
  do
  {
    lcd_MYshowstr(0, 0, menuName, BLACK, WHITE);    // 显示菜单标题
    
    // 显示菜单项
    Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
    
    key = Key_Scan();                       // 获取按键
    
    if ( Menu_Move(prmt, key) == 0 )           // 菜单移动，按下确认键
    {
      // 判断此菜单项有无需要调节的参数，有则进入参数调节
      if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Menu_Null)
      {
        site.x = TFT_X_MAX-NUM;	site.y = (1 + prmt->Cursor);
        lcd_MYshowuint16(site.x,site.y, *(table[prmt->Index].DebugParam), BLUE, RED);
        adjustParam(site, table[prmt->Index].DebugParam, BLUE, RED);
      }
      // 不是参数调节的话就执行菜单函数
      else
      {
        table[prmt->Index].ItemHook();         // 执行相应项
      }
    }
  }while (prmt->ExitMark == 0);
  lcd_clear(BLACK);
  delay_ms(50);
}
void Menu_ProcessPro(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)
{
  uint8 key = KeyNone;
  Site_t site;
  uint8 page;    //显示菜单需要的页数
  
  if(num - PAGE_DISP_NUM <= 0)
    page = 1;
  else
  {
    page = num - PAGE_DISP_NUM + 1;
    num = PAGE_DISP_NUM;
  }
  // 显示项数和页数设置
  Menu_PrmtInit(prmt, num, page); 
  
  do
  {
    lcd_MYshowstr(0, 0, menuName, BLACK, WHITE);    // 显示菜单标题
    
    // 显示菜单项
    Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
    
    key = Key_Scan();                       // 获取按键
    
    if ( Menu_Move(prmt, key) == 0 )           // 菜单移动，按下确认键
    {
      // 判断此菜单项有无需要调节的参数，有则进入参数调节
      if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Menu_Null)
      {
        site.x = TFT_X_MAX-NUM;	site.y = (1 + prmt->Cursor);
        lcd_MYshowuint16(site.x,site.y, *(table[prmt->Index].DebugParam), BLUE, RED);
        adjustParam(site, table[prmt->Index].DebugParam, BLUE, RED);
      }
      // 不是参数调节的话就执行菜单函数
      else
      {
        table[prmt->Index].ItemHook();         // 执行相应项
      }
    }
  }while (prmt->ExitMark == 0);
  lcd_clear(BLACK);
  delay_ms(50);
}

//无下级菜单
void Menu_Null()
{
  delay_ms(100);
}

void Menu_Motor()
{
	lcd_clear(BLACK);
	Menu_ProcessPro("1.MotorDebug        ", &Motor_Prmt, Motor_MenuTable, sizeof(Motor_MenuTable)/sizeof(Motor_MenuTable[0]));
	delay_ms(100);
}

void Menu_Speed()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("2.SpeedParam        ", &Speed_Prmt, Speed_MenuTable, sizeof(Speed_MenuTable)/sizeof(Speed_MenuTable[0]));
	delay_ms(100);
}

void Menu_ADC()
{
	lcd_clear(BLACK);
   while(!Key_Scan())
	{
		AD_Get_Data();

		lcd_showstr(15,0,"L_Hor");
		lcd_showstr(15,1,"L_Ver");
		lcd_showstr(15,2,"LB_Hor");
		lcd_showstr(15,3,"LB_Ver");
		lcd_showstr(15,4,"RB_Ver");
		lcd_showstr(15,5,"RB_Hor");
		lcd_showstr(15,6,"R_Ver");
		lcd_showstr(15,7,"R_Hor");

		
		lcd_showuint16(80,0,adc_data[0]);
		lcd_showuint16(80,1,adc_data[1]);
		lcd_showuint16(80,2,adc_data[2]);
		lcd_showuint16(80,3,adc_data[3]);
		lcd_showuint16(80,4,adc_data[4]);
		lcd_showuint16(80,5,adc_data[5]);
		lcd_showuint16(80,6,adc_data[6]);
		lcd_showuint16(80,7,adc_data[7]);

  	}
		delay_ms(100);
		lcd_clear(BLACK);
}

void Menu_ADC_Max()
{
	  lcd_clear(BLACK);
	Menu_ProcessPro("4.Adc_Max        ", &ADC_Max_Prmt, ADC_Max_MenuTable, sizeof(ADC_Max_MenuTable)/sizeof(ADC_Max_MenuTable[0]));
	delay_ms(100);
}

void Menu_ADC_Min()
{
	lcd_clear(BLACK);
	Menu_ProcessPro("5.Adc_Min        ", &ADC_Min_Prmt, ADC_Min_MenuTable, sizeof(ADC_Min_MenuTable)/sizeof(ADC_Min_MenuTable[0]));
	delay_ms(100);
}

void Menu_ADvalue()
{
	lcd_clear(BLACK);
  while(!Key_Scan())
	{
		AD_Get_Data();
		ADC_DataAnalyse();

		lcd_showstr(15,0,"L_Hor");
		lcd_showstr(15,1,"L_Ver");
		lcd_showstr(15,2,"LB_Hor");
		lcd_showstr(15,3,"LB_Ver");
		lcd_showstr(15,4,"RB_Ver");
		lcd_showstr(15,5,"RB_Hor");
		lcd_showstr(15,6,"R_Ver");
		lcd_showstr(15,7,"R_Hor");

		
		lcd_showuint16(80,0,ADvalue_get[0]);
		lcd_showuint16(80,1,ADvalue_get[1]);
		lcd_showuint16(80,2,ADvalue_get[2]);
		lcd_showuint16(80,3,ADvalue_get[3]);
		lcd_showuint16(80,4,ADvalue_get[4]);
		lcd_showuint16(80,5,ADvalue_get[5]);
		lcd_showuint16(80,6,ADvalue_get[6]);
		lcd_showuint16(80,7,ADvalue_get[7]);

  	}
		delay_ms(100);
		lcd_clear(BLACK);
}

void Menu_Round()
{
	lcd_clear(WHITE);
	Menu_ProcessPro("7.RoundDebug        ", &Round_Prmt, Round_MenuTable, sizeof(Round_MenuTable)/sizeof(Round_MenuTable[0]));
	delay_ms(100);
}

void Menu_RoundType()
{
	lcd_clear(WHITE);
	Menu_ProcessPro("8.RoundType        ", &RoundType_Prmt, RoundType_MenuTable, sizeof(RoundType_MenuTable)/sizeof(RoundType_MenuTable[0]));
	delay_ms(100);
}

void Menu_Servo()
{
	lcd_clear(BLACK);
	Menu_ProcessPro("9.ServoDebug        ", &Servo_Prmt, Servo_MenuTable, sizeof(Servo_MenuTable)/sizeof(Servo_MenuTable[0]));
	delay_ms(100);
}

void Menu_Type()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("10.TypeParam          ", &Type_Prmt, Type_MenuTable, sizeof(Type_MenuTable)/sizeof(Type_MenuTable[0]));
	if(new_type==1)
	{
		speedL.setspeed_basic=80;
		speedR.setspeed_basic=80;
		follow.inround_threshold = 490;
		follow.distance_threshold3 = 95;
		follow.barr_dis_threshold2 = 25;
		follow.barr_dir_threshold3 = 55;
		follow.ramp_gyro_threshold = 50;
		follow.tof_dis_threshold1 = 900;
		follow.tof_dis_threshold2 = 700;
	}
	else
	{
		speedL.setspeed_basic=130;
		speedR.setspeed_basic=130;
		follow.inround_threshold = 950;
		follow.distance_threshold3 = 50;
		follow.barr_dis_threshold2 = 30;
		follow.barr_dir_threshold3 = 45;
		follow.ramp_gyro_threshold = 100;
		follow.tof_dis_threshold1 = 1000;
		follow.tof_dis_threshold2 = 900;
	}
	if(end_type==INFR)
	{
		follow.end_dis_threshold1 = 35;
	}
	else if(end_type==HALL)
	{
		follow.end_dis_threshold1 = 60;
	}
	delay_ms(100);
}

void Menu_Thre()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("11.ThreParam          ", &Thre_Prmt, Thre_MenuTable, sizeof(Thre_MenuTable)/sizeof(Thre_MenuTable[0]));
	delay_ms(100);
}

void Menu_Barrier()
{
	lcd_clear(WHITE);
	Menu_ProcessPro("12.Barrier        ", &Barrier_Prmt, Barrier_MenuTable, sizeof(Barrier_MenuTable)/sizeof(Barrier_MenuTable[0]));
	delay_ms(100);
}

void Menu_BarrierType()
{
	lcd_clear(WHITE);
	Menu_ProcessPro("13.BarrierType        ", &BarrierType_Prmt, BarrierType_MenuTable, sizeof(BarrierType_MenuTable)/sizeof(BarrierType_MenuTable[0]));
	delay_ms(100);
}

void Menu_Buzz()
{
	lcd_clear(WHITE);
	Menu_ProcessPro("14.BuzzType        ", &Buzz_Prmt, Buzz_MenuTable, sizeof(Buzz_MenuTable)/sizeof(Buzz_MenuTable[0]));
	delay_ms(100);
}