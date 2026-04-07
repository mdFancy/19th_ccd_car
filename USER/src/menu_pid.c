#include "headfile.h"
#include "menu_pid.h"                                
#include "my_key.h"                                 //菜单必不可少的按键
#include "lcd.h"                                    //引用外部函数（注意此时开启总中断才能用镜头采集数据）

void Plan_Null(); //
void MainMenu_Set();
void MainMenu_SetA();                               //对PlanA的处理函数
void MainMenu_SetB();                               //对PlanB的处理函数
//void MainMenu_Nearset();                        //对近远端扫线的处理函数（采集数据）    菜单内开中断可参考
//void MainMenu_Farset();                         //
void MainMenu_SetStore();                           //对存储设置的处理函数


void Menu_Null();                                   //无下级函数的声明
void Menu_Store();                                  //Plan A
void Menu_Motor_A();
void Menu_Error_A();
void Menu_V_A();
void Menu_Order_A();
void Menu_Ccd_A();
void Menu_Circle_A();
              
void Menu_Motor_B();                                //Plan B
void Menu_Error_B();
void Menu_V_B();
void Menu_Order_B();
void Menu_Ccd_B();
void Menu_Circle_B();

void EEPROM_ReadA();                                //PlanAB的EEPROM的赋值函数
void EEPROM_WriteA();
void EEPROM_ReadB();           
void EEPROM_WriteB();

void Planchoose_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);       //菜单的处理函数
void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);        
void Menu_ProcessPro(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);
//void Menu_Process_NearScan(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);  //处理扫线
//void Menu_Process_FarScan(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);   //处理扫线
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor);
uint8 Menu_Move(MENU_PRMT *prmt, uint8 key);                                          
void adjustParam(Site_t site, uint16 *param);                                            //调参

//extern void near_lcd();                                                                     //外部采线函数，在屏幕上显示
//extern void far_lcd();
uint16 store_flag=0;                                                                         //用于（以下省略）控制储存方案
uint16 motor_menuflag_key=0;                                                                 //控制电机工作（在发车前屏幕工作时中断开启情况下控制电机）
uint16 bldc_key=0;
uint16 scan_key=0;

uint16 KP_speed_key=0;//速度环kp                                                              //调给外部参数
uint16 KI_speed_key=0;
uint16 long_KP_SPEED_key=0;
uint16 long_KI_SPEED_key=0;
uint16 long_speed_key=0;

uint16 KP_error_key=0;
uint16 KI_error_key=0;
uint16 KD_error_key=0;
uint16 long_KP_error_key=0;
uint16 long_KD_error_key=0;
uint16 node_key=0;
uint16 KD_gyro_key=0;
uint16 cir_line1_key=0;
uint16 cir_line2_key=0;

uint16 left_targetv_key=0;
uint16 right_targetv_key=0;
uint16 ramp_left_targetv_key=0;
uint16 ramp_right_targetv_key=0;

uint16 ele1_key=0;
uint16 ele2_key=0;
uint16 ele3_key=0;
uint16 ele4_key=0;
uint16 ele5_key=0;
uint16 ele6_key=0;
uint16 ele7_key=0;
uint16 ele8_key=0;
uint16 ele9_key=0;
uint16 ele10_key=0;
uint16 ele11_key=0;
uint16 ele12_key=0;
uint16 ele13_key=0;
uint16 ele14_key=0;
uint16 ele15_key=0;
uint16 ele16_key=0;
uint16 ele17_key=0;
uint16 ele18_key=0;
uint16 ele19_key=0;
uint16 ele20_key=0;

uint16 ccd_sta1_key=0;
uint16 ccd_sta2_key=0;

uint16 circle_KP_error_key=0;   
uint16 circle_KD_error_key=0;
uint16 r_circle_KP_error_key=0;   
uint16 r_circle_KD_error_key=0;
uint16 circle_kp_speed_key=0;   
uint16 circle_ki_speed_key=0;
uint16 circle_left_v_key=0;
uint16 circle_right_v_key=0;
//uint16 curve_speed_up_key=0;
//uint16 kd_node_key=0;
//uint16 curve_KP_SPEED_key=0;
//uint16 curve_KI_SPEED_key=0;

uint16 speedup1_key=0;
uint16 speedup2_key=0;
uint16 speedup3_key=0;



extern uint16 a[];//ele_order


//float data2[2]; //每个二级菜单对应一个数组   motor                       //用于显示参数的计算方案（防止遗忘）（暂未完善） 
//float data3[7];  //error    
//float data4[2];  // V
//uint16 data6[2];  //ccd
//float data7[6];   //circle

uint16 Adata1[48]={0};                                                  //存PlanA B，store的各参数的数组
uint16 Bdata1[48]={0};
uint16 Storedata1[7]={0};

uint16 xdata AEEPROM_Data[48] = {0};                                     //用于赋值的数组
uint16 xdata BEEPROM_Data[48] = {0};
uint16 xdata StoreEEPROM_Data[7] ={0};
void EEPROM_WriteStore(uint16* Storedata1,uint16* StoreEEPROM_Data)       
{           
  StoreEEPROM_Data[0]  = Storedata1[0];
	StoreEEPROM_Data[1]  = Storedata1[1];
	StoreEEPROM_Data[2]  = Storedata1[2];
		StoreEEPROM_Data[3]  = Storedata1[3];
	StoreEEPROM_Data[4]  = Storedata1[4];
	StoreEEPROM_Data[5]  = Storedata1[5];
		StoreEEPROM_Data[6]  = Storedata1[6];
}
void EEPROM_ReadStore(uint16* Storedata1,uint16* StoreEEPROM_Data)        
{           
  Storedata1[0]= StoreEEPROM_Data[0];
  Storedata1[1]= StoreEEPROM_Data[1];
	 Storedata1[2]= StoreEEPROM_Data[2];
	Storedata1[3]= StoreEEPROM_Data[3];
	Storedata1[4]= StoreEEPROM_Data[4];
	 Storedata1[5]= StoreEEPROM_Data[5];
	Storedata1[6]= StoreEEPROM_Data[6];
}
void EEPROM_WriteA(uint16* Adata1,uint16* AEEPROM_Data) //102 下个103           
{            
  AEEPROM_Data[0]  = Adata1[0];
	AEEPROM_Data[1]  = Adata1[1];
	
	AEEPROM_Data[2]  = Adata1[2];  
  AEEPROM_Data[3]  = Adata1[3]; 
	AEEPROM_Data[4]  = Adata1[4];
	
	AEEPROM_Data[5]  = Adata1[5];
	AEEPROM_Data[6]  = Adata1[6];
	
	AEEPROM_Data[7]  = Adata1[7];
	AEEPROM_Data[8]  = Adata1[8];
	AEEPROM_Data[9]  = Adata1[9];  
  AEEPROM_Data[10]  = Adata1[10]; 
	AEEPROM_Data[11]  = Adata1[11];
	AEEPROM_Data[12]  = Adata1[12];
	AEEPROM_Data[13]  = Adata1[13];
	AEEPROM_Data[14]  = Adata1[14];
	
	AEEPROM_Data[15]  = Adata1[15];
	AEEPROM_Data[16]  = Adata1[16];
	
	AEEPROM_Data[17]  = Adata1[17];
	AEEPROM_Data[18]  = Adata1[18];
	AEEPROM_Data[19]  = Adata1[19];
	AEEPROM_Data[20]  = Adata1[20];
	
	AEEPROM_Data[21]  = Adata1[21];
	AEEPROM_Data[22]  = Adata1[22];
	
	AEEPROM_Data[23]  = Adata1[23];
	AEEPROM_Data[24]  = Adata1[24];
	AEEPROM_Data[25]  = Adata1[25];//node_key;
	
	AEEPROM_Data[26]  = Adata1[26];
	AEEPROM_Data[27]  = Adata1[27];
	AEEPROM_Data[28]  = Adata1[28];//node_key;
	AEEPROM_Data[29]  = Adata1[29];//KD_gyro_key_key;
	AEEPROM_Data[30]  = Adata1[30];//obs_kp
	AEEPROM_Data[31]  = Adata1[31];//obs_kd
	AEEPROM_Data[32]  = Adata1[32];//ramp_left_v;
	AEEPROM_Data[33]  = Adata1[33];//ramp_right_v;
	AEEPROM_Data[31]  = Adata1[31];//obs_kd
	AEEPROM_Data[32]  = Adata1[32];//ramp_left_v;
	AEEPROM_Data[33]  = Adata1[33];//ramp_right_v;
	AEEPROM_Data[34]  = Adata1[34];//circle_kp_speed_key
	AEEPROM_Data[35]  = Adata1[35];//circle_ki_speed_key
	AEEPROM_Data[36]  = Adata1[36];
	
	AEEPROM_Data[37]  = Adata1[37];
	AEEPROM_Data[38]  = Adata1[38];
	
	AEEPROM_Data[39]  = Adata1[39];
	AEEPROM_Data[40]  = Adata1[40];
	AEEPROM_Data[41]  = Adata1[41];
	
	AEEPROM_Data[42]  = Adata1[42];
	AEEPROM_Data[43]  = Adata1[43];
	AEEPROM_Data[44]  = Adata1[44];
	AEEPROM_Data[45]  = Adata1[45];
	AEEPROM_Data[46]  = Adata1[46];
	AEEPROM_Data[47]  = Adata1[47];
//	AEEPROM_Data[48]  = Adata1[48];//uint16 curve_speed_up_key
//	AEEPROM_Data[49]  = Adata1[49];//uint16 kd_node_key
//	AEEPROM_Data[50]  = Adata1[50];//curve_KP_SPEED_key
//	AEEPROM_Data[51]  = Adata1[51];//curve_KI_SPEED_key
	
//	EEPROM_Data[2]  =  targetv;//targetv
//   //	EEPROM_Data[2]  =  speedL.Kd;
}

void EEPROM_ReadA(uint16* Adata1,uint16* AEEPROM_Data) //102 下个103           
{       
    Adata1[0] = AEEPROM_Data[0];//speedL.Kp
	  Adata1[1] = AEEPROM_Data[1];//speedL.Ki
	
    Adata1[2] = AEEPROM_Data[2];  
  	Adata1[3] = AEEPROM_Data[3];
	  Adata1[4] = AEEPROM_Data[4];
	
	  Adata1[5] = AEEPROM_Data[5];
	  Adata1[6] = AEEPROM_Data[6];
	
	  Adata1[7] = AEEPROM_Data[7];                 //ele
	  Adata1[8] = AEEPROM_Data[8];
	  Adata1[9] = AEEPROM_Data[9];  
		Adata1[10]= AEEPROM_Data[10]; 
		Adata1[11]= AEEPROM_Data[11];
		Adata1[12]= AEEPROM_Data[12];
		Adata1[13]= AEEPROM_Data[13];
		Adata1[14]= AEEPROM_Data[14];
	
		Adata1[15]= AEEPROM_Data[15];                //ccd
		Adata1[16]= AEEPROM_Data[16];
	
		Adata1[17]= AEEPROM_Data[17];                  //circle
		Adata1[18]= AEEPROM_Data[18];
		Adata1[19]= AEEPROM_Data[19];
		Adata1[20]= AEEPROM_Data[20];
		
		Adata1[21]= AEEPROM_Data[21];
		Adata1[22]= AEEPROM_Data[22];
		
		Adata1[23]= AEEPROM_Data[23];
		Adata1[24]= AEEPROM_Data[24];
		Adata1[25]= AEEPROM_Data[25];
		
		Adata1[26]= AEEPROM_Data[26];
		Adata1[27]= AEEPROM_Data[27];
		Adata1[28]= AEEPROM_Data[28];
	  Adata1[29]= AEEPROM_Data[29];
		Adata1[30]= AEEPROM_Data[30];
	  Adata1[31]= AEEPROM_Data[31];
    Adata1[32]= AEEPROM_Data[32];
	  Adata1[33]= AEEPROM_Data[33];
		Adata1[34]=	AEEPROM_Data[34];//circle_kp_speed_key
	  Adata1[35]= AEEPROM_Data[35];//circle_ki_speed_key
		Adata1[36]= AEEPROM_Data[36];
	
	  Adata1[37]= AEEPROM_Data[37];
	  Adata1[38]= AEEPROM_Data[38];
	
	  Adata1[39]= AEEPROM_Data[39];
		Adata1[40]= AEEPROM_Data[40];
	  Adata1[41]= AEEPROM_Data[41];
	
	  Adata1[42]= AEEPROM_Data[42];
	  Adata1[43]= AEEPROM_Data[43];
		Adata1[44]= AEEPROM_Data[44];
		Adata1[45]= AEEPROM_Data[45];
	  Adata1[46]= AEEPROM_Data[46];
    Adata1[47]= AEEPROM_Data[47];
//	  Adata1[48]= AEEPROM_Data[48];//uint16 curve_speed_up_key
//	  Adata1[49]= AEEPROM_Data[49];//uint16 kd_node_key
//		Adata1[50]= AEEPROM_Data[50];//curve_KP_SPEED_key
//		Adata1[51]= AEEPROM_Data[51];//curve_KI_SPEED_key
}

void EEPROM_WriteB(uint16* Bdata1,uint16* BEEPROM_Data) //102 下个103           
{           
  BEEPROM_Data[0]  = Bdata1[0];
	BEEPROM_Data[1]  = Bdata1[1];
	
	BEEPROM_Data[2]  = Bdata1[2];  
  BEEPROM_Data[3]  = Bdata1[3]; 
	BEEPROM_Data[4]  = Bdata1[4];
	
	BEEPROM_Data[5]  = Bdata1[5];
	BEEPROM_Data[6]  = Bdata1[6];
	
	BEEPROM_Data[7]  = Bdata1[7];
	BEEPROM_Data[8]  = Bdata1[8];
	BEEPROM_Data[9]  = Bdata1[9];  
  BEEPROM_Data[10]  = Bdata1[10]; 
	BEEPROM_Data[11]  = Bdata1[11];
	BEEPROM_Data[12]  = Bdata1[12];
	BEEPROM_Data[13]  = Bdata1[13];
	BEEPROM_Data[14]  = Bdata1[14];
	
	BEEPROM_Data[15]  = Bdata1[15];
	BEEPROM_Data[16]  = Bdata1[16];
	
	BEEPROM_Data[17]  = Bdata1[17];
	BEEPROM_Data[18]  = Bdata1[18];
	BEEPROM_Data[19]  = Bdata1[19];

	BEEPROM_Data[20]  = Bdata1[20];
	
	BEEPROM_Data[21]  = Bdata1[21];
	BEEPROM_Data[22]  = Bdata1[22];
	
	BEEPROM_Data[23]  = Bdata1[23];
	BEEPROM_Data[24]  = Bdata1[24];
	BEEPROM_Data[25]  = Bdata1[25];//node_key;
	
	BEEPROM_Data[26]  = Bdata1[26];
	BEEPROM_Data[27]  = Bdata1[27];
	BEEPROM_Data[28]  = Bdata1[28];//node_key;
	BEEPROM_Data[29]  = Bdata1[29];//KD_gyro_key_key;
	BEEPROM_Data[30]  = Bdata1[30];//obs_kp
	BEEPROM_Data[31]  = Bdata1[31];//obs_kd
	BEEPROM_Data[32]  = Bdata1[32];//ramp_left_v;
	BEEPROM_Data[33]  = Bdata1[33];//ramp_right_v;
	BEEPROM_Data[34]  = Bdata1[34];//circle_kp_speed_key
	BEEPROM_Data[35]  = Bdata1[35];//circle_ki_speed_key
	BEEPROM_Data[36]  = Bdata1[36];
	
	BEEPROM_Data[37]  = Bdata1[37];
	BEEPROM_Data[38]  = Bdata1[38];
	
	BEEPROM_Data[39]  = Bdata1[39];
	BEEPROM_Data[40]  = Bdata1[40];
	BEEPROM_Data[41]  = Bdata1[41];
	
	BEEPROM_Data[42]  = Bdata1[42];
	BEEPROM_Data[43]  = Bdata1[43];
	BEEPROM_Data[44]  = Bdata1[44];
	BEEPROM_Data[45]  = Bdata1[45];
	BEEPROM_Data[46]  = Bdata1[46];
	BEEPROM_Data[47]  = Bdata1[47];
//	BEEPROM_Data[48]  = Bdata1[48];//uint16 curve_speed_up_key
//	BEEPROM_Data[49]  = Bdata1[49];//uint16 kd_node_key
//	BEEPROM_Data[50]  = Bdata1[50];//curve_KP_SPEED_key
//	BEEPROM_Data[51]  = Bdata1[51];//curve_KI_SPEED_key
}

void EEPROM_ReadB(uint16* Bdata1,uint16* BEEPROM_Data) //102 下个103           
{       
    Bdata1[0] = BEEPROM_Data[0];//speedL.Kp
	  Bdata1[1] = BEEPROM_Data[1];//speedL.Ki
	
    Bdata1[2] = BEEPROM_Data[2];  
  	Bdata1[3] = BEEPROM_Data[3];
	  Bdata1[4] = BEEPROM_Data[4];
	
	  Bdata1[5] = BEEPROM_Data[5];
	  Bdata1[6] = BEEPROM_Data[6];
	
	  Bdata1[7] = BEEPROM_Data[7];                 //ele
	  Bdata1[8] = BEEPROM_Data[8];
	  Bdata1[9] = BEEPROM_Data[9];  
		Bdata1[10]= BEEPROM_Data[10]; 
		Bdata1[11]= BEEPROM_Data[11];
		Bdata1[12]= BEEPROM_Data[12];
		Bdata1[13]= BEEPROM_Data[13];
		Bdata1[14]= BEEPROM_Data[14];
	
		Bdata1[15]= BEEPROM_Data[15];                //ccd
		Bdata1[16]= BEEPROM_Data[16];
	
		Bdata1[17]= BEEPROM_Data[17];                  //circle
		Bdata1[18]= BEEPROM_Data[18];
		Bdata1[19]= BEEPROM_Data[19];
		Bdata1[20]= BEEPROM_Data[20];
		
		Bdata1[21]= BEEPROM_Data[21];
		Bdata1[22]= BEEPROM_Data[22];
		
		Bdata1[23]= BEEPROM_Data[23];
		Bdata1[24]= BEEPROM_Data[24];
		Bdata1[25]= BEEPROM_Data[25];
		
		Bdata1[26]= BEEPROM_Data[26];
		Bdata1[27]= BEEPROM_Data[27];
		Bdata1[28]= BEEPROM_Data[28];
	  Bdata1[29]= BEEPROM_Data[29];
		Bdata1[30]= BEEPROM_Data[30];
	  Bdata1[31]= BEEPROM_Data[31];
    Bdata1[32]= BEEPROM_Data[32];
	  Bdata1[33]= BEEPROM_Data[33];
		Bdata1[34]= BEEPROM_Data[34];//circle_kp_speed_key
    Bdata1[35]=	BEEPROM_Data[35];//circle_ki_speed_key
		Bdata1[36]= BEEPROM_Data[36];
	
	  Bdata1[37]= BEEPROM_Data[37];
	  Bdata1[38]= BEEPROM_Data[38];
	
	  Bdata1[39]= BEEPROM_Data[39];
		Bdata1[40]= BEEPROM_Data[40];
	  Bdata1[41]= BEEPROM_Data[41];
	
	  Bdata1[42]= BEEPROM_Data[42];
	  Bdata1[43]= BEEPROM_Data[43];
		Bdata1[44]= BEEPROM_Data[44];
		Bdata1[45]= BEEPROM_Data[45];
	  Bdata1[46]= BEEPROM_Data[46];
    Bdata1[47]= BEEPROM_Data[47];
//	  Bdata1[48]= BEEPROM_Data[48];//uint16 curve_speed_up_key
//	  Bdata1[49]= BEEPROM_Data[49];//uint16 kd_node_key
//		Bdata1[50]= BEEPROM_Data[50];//curve_KP_SPEED_key
//		Bdata1[51]= BEEPROM_Data[51];//curve_KI_SPEED_key
}
//----------------------------------   主菜单   -------------------------------
//选择方案(一级菜单)
MENU_PRMT Plan_Prmt;                         

MENU_TABLE Plan_Table[] =
{                                                               //主菜单下的方案名称，要运行的函数，要调的参数 
	{"1.PlanA         ", MainMenu_SetA, NULL},                    //                    此处函数为对一级菜单各项的包装处理函数            标志MainMenu
    
	{"2.PlanB         ", MainMenu_SetB, NULL}, 
	
//	{"3.ScanNear      ", MainMenu_Nearset,NULL},
//	
//	{"4.ScanFar       ", MainMenu_Farset,NULL},
	
	{"3.store         ", MainMenu_SetStore,NULL},
};

// 二级菜单                                             
MENU_PRMT MainMenuA_Prmt;

MENU_TABLE MainMenuA_Table[] =                         //主菜单显示项目数组MainMenu_Table[]
{

	{"1.MotorDebug         ", Menu_Motor_A, NULL},      //主菜单下的项目名称，要运行的函数，要调的参数  float *param
    
	{"2.ErrorParam         ", Menu_Error_A, NULL}, 
	
	{"3.TargetV            ", Menu_V_A, NULL},

  {"4.Order              ", Menu_Order_A,NULL},

  {"5.Ccd                ", Menu_Ccd_A,NULL},
	
	{"6.Circle             ", Menu_Circle_A,NULL},


};

MENU_PRMT MainMenuB_Prmt;

MENU_TABLE MainMenuB_Table[] =                         //主菜单显示项目数组MainMenu_Table[]
{

	{"1.MotorDebug         ", Menu_Motor_B, NULL},      //主菜单下的项目名称，要运行的函数，要调的参数  float *param
    
	{"2.ErrorParam         ", Menu_Error_B, NULL}, 
	
	{"3.TargetV            ", Menu_V_B, NULL},

  {"4.Order              ", Menu_Order_B,NULL},

  {"5.Ccd                ", Menu_Ccd_B,NULL},
	
	{"6.Circle             ", Menu_Circle_B,NULL},


};

//MENU_PRMT MainMenuSN_Prmt;

//MENU_TABLE MainMenuSN_Table[] =                        
//{
//	{"1.ScanNear           ", Menu_Null, NULL},
//};

//MENU_PRMT MainMenuSF_Prmt;

//MENU_TABLE MainMenuSF_Table[] =                         
//{
//	{"1.ScanF              ", Menu_Null, NULL}, 
//};

MENU_PRMT MainMenuStore_Prmt;

MENU_TABLE MainMenuStore_Table[] =                         
{
	{"1.Store              ", Menu_Store, NULL}, 
	
};
// 三级菜单

MENU_PRMT Store_Prmt;

MENU_TABLE Store_MenuTable[] = 
{
	{"1.Sflag    ", Menu_Null, &Storedata1[0]},
	
	{"2.mflag    ", Menu_Null, &Storedata1[1]},
		
	{"3.Bldcflag   ", Menu_Null, &Storedata1[2]},
	
	{"4.scanflag   ", Menu_Null, &Storedata1[3]},
	
	{"5.speedup1    ", Menu_Null, &Storedata1[4]},
		
	{"6.speedup2   ", Menu_Null, &Storedata1[5]},
	
	{"7.speedup3   ", Menu_Null, &Storedata1[6]},
	
	
};



MENU_PRMT MotorA_Prmt;

MENU_TABLE MotorA_MenuTable[] = 
{
	{"1.Speed_KP  ", Menu_Null, &Adata1[0]},

	{"2.Speed_KI  ", Menu_Null, &Adata1[1]},
	
	{"3.Long_KP   ", Menu_Null, &Adata1[26]},

	{"4.Long_KI   ", Menu_Null, &Adata1[27]},
	
//	{"5.Curv_KP   ", Menu_Null, &Adata1[50]},

//	{"6.Curv_KI   ", Menu_Null, &Adata1[51]},
	
};

MENU_PRMT MotorB_Prmt;

MENU_TABLE MotorB_MenuTable[] = 
{
	{"1.Speed_KP  ", Menu_Null, &Bdata1[0]},

	{"2.Speed_KI  ", Menu_Null, &Bdata1[1]},
	
	{"3.Long_KP   ", Menu_Null, &Bdata1[26]},

	{"4.Long_KI   ", Menu_Null, &Bdata1[27]},
	
//	{"5.Curv_KP   ", Menu_Null, &Bdata1[50]},

//	{"6.Curv_KI   ", Menu_Null, &Bdata1[51]},
	
};

MENU_PRMT ErrorA_Prmt;

MENU_TABLE ErrorA_MenuTable[] = 
{
	{"1.E_KP      ", Menu_Null, &Adata1[2]},
	
	{"2.E_KI      ", Menu_Null, &Adata1[3]},
	
  {"3.E_KD      ", Menu_Null, &Adata1[4]},
	
	{"4.NODE      ", Menu_Null, &Adata1[23]},
	
	{"5.LONG_P    ", Menu_Null, &Adata1[24]},
	
  {"6.LONG_D    ", Menu_Null, &Adata1[25]},
	
	{"7.Kd_gp     ", Menu_Null, &Adata1[29]},
	
//	{"8.kd_key  ", Menu_Null, &Adata1[49]},

};

MENU_PRMT ErrorB_Prmt;

MENU_TABLE ErrorB_MenuTable[] = 
{
	{"1.E_KP      ", Menu_Null, &Bdata1[2]},
	
	{"2.E_KI      ", Menu_Null, &Bdata1[3]},
	
  {"3.E_KD      ", Menu_Null, &Bdata1[4]},
	
	{"4.NODE      ", Menu_Null, &Bdata1[23]},
	
	{"5.LONG_P    ", Menu_Null, &Bdata1[24]},
	
  {"6.LONG_D    ", Menu_Null, &Bdata1[25]},
	
	{"7.Kd_gp     ", Menu_Null, &Bdata1[29]},
	
//	{"8.kd_key  ", Menu_Null, &Bdata1[49]},

};


MENU_PRMT VA_Prmt;

MENU_TABLE VA_MenuTable[] = 
{
	{"1.L_V      ", Menu_Null, &Adata1[5]},
	
	{"2.R_V      ", Menu_Null, &Adata1[6]},
	
	{"3.Long_V    "  , Menu_Null, &Adata1[28]},
	
	{"4.Ramp_L_V  ", Menu_Null, &Adata1[32]},
	
	{"5.Ramp_R_V  ", Menu_Null, &Adata1[33]},
	
//	{"6.Curv_V   ", Menu_Null, &Adata1[48]},
	
 
};


MENU_PRMT VB_Prmt;

MENU_TABLE VB_MenuTable[] = 
{
	{"1.L_V ", Menu_Null, &Bdata1[5]},
	
	{"2.R_V ", Menu_Null, &Bdata1[6]},
	
	{"3.Long_V  ", Menu_Null, &Bdata1[28]},
	
	{"4.Ramp_L_V", Menu_Null, &Bdata1[32]},
	
	{"5.Ramp_R_V", Menu_Null, &Bdata1[33]},
	
//	{"6.Curv_V   ", Menu_Null, &Bdata1[48]},
	
 
};

MENU_PRMT OrderA_Prmt;

MENU_TABLE OrderA_MenuTable[] = 
{
	{"1.ele1  ", Menu_Null, &Adata1[7]},
	
	{"2.ele2  ", Menu_Null, &Adata1[8]},
	
	{"3.ele3  ", Menu_Null, &Adata1[9]},
	
	{"4.ele4  ", Menu_Null, &Adata1[10]},
	
	{"5.ele5  ", Menu_Null, &Adata1[11]},
	
	{"6.ele6  ",Menu_Null, &Adata1[12]},
	
	{"7.ele7  ", Menu_Null, &Adata1[13]},
	
	{"8.ele8  ", Menu_Null, &Adata1[14]},
	
	{"9.ele9  ", Menu_Null, &Adata1[36]},
	
	{"10.ele10  ", Menu_Null, &Adata1[37]},
	
	{"11.ele11  ", Menu_Null, &Adata1[38]},
	
	{"12.ele12 ", Menu_Null, &Adata1[39]},
	
	{"13.ele13  ", Menu_Null, &Adata1[40]},
	
	{"14.ele14  ",Menu_Null, &Adata1[41]},
	
	{"15.ele15 ", Menu_Null, &Adata1[42]},
	
	{"16.ele16 ", Menu_Null, &Adata1[43]},
	
	{"17.ele17  ", Menu_Null, &Adata1[44]},
	
	{"18.ele18  ", Menu_Null, &Adata1[45]},
	
	{"19.ele19 ", Menu_Null, &Adata1[46]},
	
	{"20.ele20  ", Menu_Null, &Adata1[47]},
	
};

MENU_PRMT OrderB_Prmt;

MENU_TABLE OrderB_MenuTable[] = 
{
	{"1.ele1  ", Menu_Null, &Bdata1[7]},
	
	{"2.ele2  ", Menu_Null, &Bdata1[8]},
	
	{"3.ele3  ", Menu_Null, &Bdata1[9]},
	
	{"4.ele4  ", Menu_Null, &Bdata1[10]},
	
	{"5.ele5  ", Menu_Null, &Bdata1[11]},
	
	{"6.ele6  ", Menu_Null, &Bdata1[12]},
	
	{"7.ele7  ", Menu_Null, &Bdata1[13]},
	
	{"8.ele8  ", Menu_Null, &Bdata1[14]},
	
	{"9.ele9  ", Menu_Null, &Bdata1[36]},
	
	{"10.ele10  ", Menu_Null, &Bdata1[37]},
	
	{"11.ele11  ", Menu_Null, &Bdata1[38]},
	
	{"12.ele12 ", Menu_Null, &Bdata1[39]},
	
	{"13.ele13  ", Menu_Null, &Bdata1[40]},
	
	{"14.ele14  ",Menu_Null, &Bdata1[41]},
	
	{"15.ele15 ", Menu_Null, &Bdata1[42]},
	
	{"16.ele16 ", Menu_Null, &Bdata1[43]},
	
	{"17.ele17  ", Menu_Null, &Bdata1[44]},
	
	{"18.ele18  ", Menu_Null, &Bdata1[45]},
	
	{"19.ele19 ", Menu_Null, &Bdata1[46]},
	
	{"20.ele20  ", Menu_Null, &Bdata1[47]},
	
 
};

MENU_PRMT CcdA_Prmt;

MENU_TABLE CcdA_MenuTable[] = 
{
	{"1.sta1  ", Menu_Null, &Adata1[15]},
	
	{"2.sta2  ", Menu_Null, &Adata1[16]},
 
};

MENU_PRMT CcdB_Prmt;

MENU_TABLE CcdB_MenuTable[] = 
{
	{"1.sta1  ", Menu_Null, &Bdata1[15]},

	{"2.sta2  ", Menu_Null, &Bdata1[16]},
 
};

MENU_PRMT CircleA_Prmt;

MENU_TABLE CircleA_MenuTable[] = 
{
	{"1.Kp_err", Menu_Null, &Adata1[17]},
	
	{"2.Kd_err", Menu_Null, &Adata1[18]},
	
	{"3.R_Kp_err", Menu_Null, &Adata1[21]},
	
	{"4.R_Kd_err", Menu_Null, &Adata1[22]},
	
	{"5.L_V  ", Menu_Null, &Adata1[19]},
	
	{"6.R_V  ", Menu_Null, &Adata1[20]},
	
	{"7.Kp_speed",Menu_Null,&Adata1[34]},
	
	{"8.Ki_speed",Menu_Null,&Adata1[35]},
	
	{"9.line1   ",Menu_Null,&Adata1[30]},
	
	{"10.line2  ",Menu_Null,&Adata1[31]},
	
};
MENU_PRMT CircleB_Prmt;

MENU_TABLE CircleB_MenuTable[] = 
{
	{"1.Kp_err", Menu_Null, &Bdata1[17]},
	
	{"2.Kd_err", Menu_Null, &Bdata1[18]},
	
	{"3.R_Kp_err", Menu_Null, &Bdata1[21]},
	
	{"4.R_Kd_err", Menu_Null, &Bdata1[22]},
	
	{"5.L_V ", Menu_Null, &Bdata1[19]},
	
	{"6.R_V  ", Menu_Null, &Bdata1[20]},
	
	{"7.Kp_speed",Menu_Null,&Bdata1[34]},
	
	{"8.Ki_speed",Menu_Null,&Bdata1[35]},
	
	{"9.line1   ",Menu_Null,&Bdata1[30]},
	
	{"10.line2  ",Menu_Null,&Bdata1[31]},
 
};
/******************************************************************************
* FunctionName   : MainMenu_Set()
* Description    : 常规设置
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/

void MainMenu_Set(void)                                                                                                              //主函数                  
	
{
  uint8 planNum = sizeof(Plan_Table)/sizeof(Plan_Table[0]);    // 菜单项数

  DisableGlobalIRQ();//关闭总中断

	iap_read_bytes(0x0000, AEEPROM_Data, sizeof(AEEPROM_Data)/sizeof(uint16));	//读进rom  
  EEPROM_ReadA(Adata1,AEEPROM_Data); //从data读数据给变

	iap_read_bytes(0x0200, BEEPROM_Data, sizeof(BEEPROM_Data)/sizeof(uint16));	//读进rom  
  EEPROM_ReadB(Bdata1,BEEPROM_Data); //从data读数据给变
	
	iap_read_bytes(0x0400, StoreEEPROM_Data, sizeof(StoreEEPROM_Data)/sizeof(uint16));	//读进rom  
  EEPROM_ReadStore(Storedata1,StoreEEPROM_Data); //从data读数据给变

  lcd_clear(BLACK);

	Planchoose_Process("Choose Plan", &Plan_Prmt, Plan_Table, planNum);    
	
	iap_erase_page(0x0000);
  iap_erase_page(0x0200);
	iap_erase_page(0x0400);

	EEPROM_WriteA(Adata1,AEEPROM_Data);
  EEPROM_WriteB(Bdata1,BEEPROM_Data); 	//写进data
	EEPROM_WriteStore(Storedata1,StoreEEPROM_Data);

	iap_write_bytes(0x0000, AEEPROM_Data, sizeof(AEEPROM_Data)/sizeof(uint16));         //写进rom
  iap_write_bytes(0x0200, BEEPROM_Data, sizeof(BEEPROM_Data)/sizeof(uint16));         //写进rom
  iap_write_bytes(0x0400, StoreEEPROM_Data, sizeof(StoreEEPROM_Data)/sizeof(uint16)); //写进rom
	

	store_flag=Storedata1[0];
	motor_menuflag_key=Storedata1[1];
	bldc_key=Storedata1[2];
	scan_key=Storedata1[3];
	speedup1_key=Storedata1[4];
	speedup2_key=Storedata1[5];
	speedup3_key=Storedata1[6];

	
	//      Store_flag为1时外部调用PlanA参数          为2时外部调用PlanB                                                                    
	//      Store_flag为1时外部调用PlanA          为2时外部调用PlanB
	//      Store_flag为1时外部调用PlanA          为2时外部调用PlanB
	if(store_flag==1)
	{
	//A
  lcd_clear(BLACK);
	KP_speed_key=Adata1[0];
	KI_speed_key=Adata1[1];
  
  KP_error_key=Adata1[2];
  KI_error_key=Adata1[3];
  KD_error_key=Adata1[4];
	
	left_targetv_key=Adata1[5];
	right_targetv_key=Adata1[6];
		
	ele1_key=Adata1[7];
  ele2_key=Adata1[8];
	ele3_key=Adata1[9];
	ele4_key=Adata1[10];
	ele5_key=Adata1[11];
	ele6_key=Adata1[12];
	ele7_key=Adata1[13];
	ele8_key=Adata1[14];
	ele9_key=Adata1[36];
  ele10_key=Adata1[37];
	ele11_key=Adata1[38];
	ele12_key=Adata1[39];
	ele13_key=Adata1[40];
	ele14_key=Adata1[41];
	ele15_key=Adata1[42];
	ele16_key=Adata1[43];	
	ele17_key=Adata1[44];
	ele18_key=Adata1[45];
	ele19_key=Adata1[46];
	ele20_key=Adata1[47];
	
  ccd_sta1_key=Adata1[15];
  ccd_sta2_key=Adata1[16];
  circle_KP_error_key=Adata1[17];   
  circle_KD_error_key=Adata1[18];
	r_circle_KP_error_key=Adata1[21];   
  r_circle_KD_error_key=Adata1[22];
  circle_left_v_key=Adata1[19];
  circle_right_v_key=Adata1[20];
	long_KP_error_key=Adata1[24];
	long_KD_error_key=Adata1[25];
	node_key=Adata1[23];
	long_KP_SPEED_key=Adata1[26];
	long_KI_SPEED_key=Adata1[27];
	long_speed_key=Adata1[28];
	KD_gyro_key=Adata1[29];
	cir_line1_key=Adata1[30];
	cir_line2_key=Adata1[31];
	ramp_left_targetv_key=Adata1[32];
	ramp_right_targetv_key=Adata1[33];
	circle_kp_speed_key=Adata1[34];
	circle_ki_speed_key=Adata1[35];
//	curve_speed_up_key=Adata1[48];
//	kd_node_key=Adata1[49];
//	curve_KP_SPEED_key=Adata1[50];
//	curve_KI_SPEED_key=Adata1[51];


//	data2[0]=(float)KP_speed_key;                     //显示
//  data2[1]=(float)KI_speed_key;
//	data3[0]=(((float)KP_error_key)+200)/100.0;
//	data3[1]=(float)KI_error_key;
//	data3[2]=(((float)KD_error_key)+100)/10.0;
//	data4[0]=(float)left_targetv_key;
//	data4[1]=(float)right_targetv_key;
//	data6[0]=ccd_sta1_key;
//  data6[1]=ccd_sta2_Key;
//	data7[0]=((float)circle_KP_error_key)/10.0;
//	data7[1]=((float)circle_KD_error_key)/10.0;
//	data7[2]=(float)circle_left_v_key;
//	data7[3]=(float)circle_right_v_key;
//	data7[4]=((float)r_circle_KP_error_key)/10.0;
//	data7[5]=((float)r_circle_KD_error_key)/10.0;
//data3[]=(float)obs_Kp_key*0.1;
//data3[]=(float)obs_Kd_key*0.1;
  }
	else if(store_flag==2)
	{
	//B
	KP_speed_key=Bdata1[0];
	KI_speed_key=Bdata1[1];
 
  KP_error_key=Bdata1[2];
  KI_error_key=Bdata1[3];
  KD_error_key=Bdata1[4];
	
	left_targetv_key=Bdata1[5];
	right_targetv_key=Bdata1[6];
	ele1_key=Bdata1[7];
  ele2_key=Bdata1[8];
	ele3_key=Bdata1[9];
	ele4_key=Bdata1[10];
	ele5_key=Bdata1[11];
	ele6_key=Bdata1[12];
	ele7_key=Bdata1[13];
	ele8_key=Bdata1[14];
	ele9_key=Bdata1[36];
  ele10_key=Bdata1[37];
	ele11_key=Bdata1[38];
	ele12_key=Bdata1[39];
	ele13_key=Bdata1[40];
	ele14_key=Bdata1[41];
	ele15_key=Bdata1[42];
	ele16_key=Bdata1[43];	
	ele17_key=Bdata1[44];
	ele18_key=Bdata1[45];
	ele19_key=Bdata1[46];
	ele20_key=Bdata1[47];
	
  ccd_sta1_key=Bdata1[15];
  ccd_sta2_key=Bdata1[16];
  circle_KP_error_key=Bdata1[17];   
  circle_KD_error_key=Bdata1[18];
	r_circle_KP_error_key=Bdata1[21];   
  r_circle_KD_error_key=Bdata1[22];
  circle_left_v_key=Bdata1[19];
  circle_right_v_key=Bdata1[20];
	long_KP_error_key=Bdata1[24];
	long_KD_error_key=Bdata1[25];
	node_key=Bdata1[23];
	long_KP_SPEED_key=Bdata1[26];
	long_KI_SPEED_key=Bdata1[27];
	long_speed_key=Bdata1[28];
	KD_gyro_key=Bdata1[29];
	cir_line1_key=Bdata1[30];
	cir_line2_key=Bdata1[31];
	ramp_left_targetv_key=Bdata1[32];
	ramp_right_targetv_key=Bdata1[33];
	circle_kp_speed_key=Bdata1[34];
	circle_ki_speed_key=Bdata1[35];
//	curve_speed_up_key=Bdata1[48];
//	kd_node_key=Bdata1[49];
//	curve_KP_SPEED_key=Bdata1[50];
//	curve_KI_SPEED_key=Bdata1[51];


//	data2[0]=(float)KP_speed_key;                     //显示
//  data2[1]=(float)KI_speed_key;
//	data3[0]=(((float)KP_error_key)+200)/100.0;
//	data3[1]=(float)KI_error_key;
//	data3[2]=(((float)KD_error_key)+100)/10.0;
//	data4[0]=(float)left_targetv_key;
//	data4[1]=(float)right_targetv_key;
//	data6[0]=ccd_sta1_key;
//  data6[1]=ccd_sta2_Key;
//	data7[0]=((float)circle_KP_error_key)/10.0;
//	data7[1]=((float)circle_KD_error_key)/10.0;
//	data7[2]=(float)circle_left_v_key;
//	data7[3]=(float)circle_right_v_key;
//	data7[4]=((float)r_circle_KP_error_key)/10.0;
//	data7[5]=((float)r_circle_KD_error_key)/10.0;
//data3[]=(float)obs_Kp_key*0.1;
//data3[]=(float)obs_Kd_key*0.1;

  }
	a[0]=ele1_key;              //order;
	a[1]=ele2_key;
	a[2]=ele3_key;
	a[3]=ele4_key;
	a[4]=ele5_key;
	a[5]=ele6_key;
	a[6]=ele7_key;
	a[7]=ele8_key;
	a[8]=ele9_key;              //order;
	a[9]=ele10_key;
	a[10]=ele11_key;
	a[11]=ele12_key;
	a[12]=ele13_key;
	a[13]=ele14_key;
	a[14]=ele15_key;
	a[15]=ele16_key;
	a[16]=ele17_key;              
	a[17]=ele18_key;
	a[18]=ele19_key;
	a[19]=ele20_key;

	
  EnableGlobalIRQ();//开启总中断（扫线的情况才开启，一般菜单都关闭，若开启总中断注意电机是否会开启）
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
	prmt->MENUNUM  = 0;
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
      lcd_showstr(site.x,site.y,menuTable[pageNo+i].MenuName);
      /* 若此菜单项有需要调的参数，则显示该参数 */
      if(menuTable[pageNo+i].DebugParam != NULL)
      {
        site.x = TFT_X_MAX-NUM;  //数字坐标
        num_t = (*(menuTable[pageNo+i].DebugParam));
        lcd_showuint16(site.x,site.y,num_t);//BLUE, WHITE
      }
    }
    else
    {
      /* 正常显示其余菜单项 */
      site.x = 0;    site.y = (i+1);
      lcd_showstr(site.x,site.y,menuTable[pageNo+i].MenuName);//     
      /* 若此菜单项有需要调的参数，则显示该参数 */
      if(menuTable[pageNo+i].DebugParam != NULL)
      {
        site.x = TFT_X_MAX-NUM;
        num_t = (*(menuTable[pageNo+i].DebugParam));
        lcd_showuint16(site.x,site.y,num_t);
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

void adjustParam(Site_t site, uint16 *param)
{
  uint8 key = KeyNone;
  uint8 e = 1;
  do{ 
      key = Key_Scan();
      switch(key)
      {
        case(KeyUp):                         //void lcd_MYshowuint16(uint16 x,uint16 y,uint16 dat, uint16 BGcolor, uint16 PENcolor);
          (*param)+=e;
           lcd_MYshowuint16(site.x,site.y,(uint16)(*param),BLUE,YELLOW);                     //lcd_showuint16(site.x,site.y, (uint16)(*param));   

          break;            
        case(KeyDown):
          (*param)-=e;
          lcd_MYshowuint16(site.x,site.y,(uint16)(*param),BLUE,YELLOW);           break;
        case(KeyLeft):
          e /= 10;

          lcd_MYshowuint16(site.x,site.y,(uint16)(*param),BLUE,YELLOW);           break;
        case(KeyRight):
          e *= 10;
           lcd_MYshowuint16(site.x,site.y,(uint16)(*param),BLUE,YELLOW);           break;
        default: break;       
      }
	  lcd_showuint16(69, 0, e);
	  if(e<=1)	e=1;
		if(e>=100) e=100;
      if(*param <= 0)  *param = 0; 
      if(*param >= 65535) *param = 65535;
  }while(key != KeyCen);
}




/******************************************************************************
* FunctionName   : Menu_Process()
* Description    : 处理菜单项
* EntryParameter : menuName - 菜单名称，prmt - 菜单参数，table - 菜单表项, num - 菜单项数
* ReturnValue    : None
******************************************************************************/
void Planchoose_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)               //处理速度方案   
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
	MainMenuA_Prmt.MENUNUM=1;
	MainMenuB_Prmt.MENUNUM=2;

			
  do
  {
    lcd_showstr(0, 0, menuName);    // 显示菜单标题
    
    // 显示菜单项
    Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
    key = Key_Scan();                              // 获取按键
    lcd_showstr(TFT_X_MAX-NUM+3,(1 + prmt->Cursor),"<<<");
		
		
    if ( Menu_Move(prmt, key) == 0 )           // 菜单移动，按下确认键
    {
      // 判断此菜单项有无需要调节的参数，有则进入参数调节
      if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Plan_Null)
      {
        site.x = TFT_X_MAX-NUM;	site.y = (1 + prmt->Cursor);
        lcd_showuint16(site.x,site.y, *(table[prmt->Index].DebugParam));
        adjustParam(site, table[prmt->Index].DebugParam);
      }
      // 不是参数调节的话就执行菜单函数
      else
      {
        table[prmt->Index].ItemHook();         // 执行相应函数
      }
    }
  }while (prmt->ExitMark == 0);
  lcd_clear(BLACK);
  delay_ms(50);
}


void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)                               //处理指定速度方案的菜单设置（二级）
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
    lcd_showstr(0, 0, menuName);    // 显示菜单标题
    
    // 显示菜单项
    Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
    
    key = Key_Scan();                       // 获取按键
    lcd_showstr(TFT_X_MAX-NUM+3,(1 + prmt->Cursor),"<<<");
    if ( Menu_Move(prmt, key) == 0 )           // 菜单移动，按下确认键
    {
      // 判断此菜单项有无需要调节的参数，有则进入参数调节
      if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Menu_Null)
      {
        site.x = TFT_X_MAX-NUM;	site.y = (1 + prmt->Cursor);
        lcd_showuint16(site.x,site.y, *(table[prmt->Index].DebugParam));
        adjustParam(site, table[prmt->Index].DebugParam);
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


void Menu_ProcessPro(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)                         //处理二级菜单的不同参数（三级）
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
	
  MotorA_Prmt.MENUNUM=1;                             //排序，方便显示参数的计算方案                                                         
	ErrorA_Prmt.MENUNUM=2;
	VA_Prmt.MENUNUM=3;
	CircleA_Prmt.MENUNUM=4;
	
  do
  {
    lcd_showstr(0, 0, menuName);    // 显示菜单标题
    
    // 显示菜单项
    Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
    
    key = Key_Scan();                       // 获取按键
		
		
		 lcd_MYshowstr(TFT_X_MAX-NUM,(1 + prmt->Cursor),"->",BLACK,YELLOW);
    
    if ( Menu_Move(prmt, key) == 0 )           // 菜单移动，按下确认键
    {
      // 判断此菜单项有无需要调节的参数，有则进入参数调节
      if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Menu_Null)
      {
        site.x = TFT_X_MAX-NUM;	site.y = (1 + prmt->Cursor);
        lcd_MYshowuint16(site.x,site.y, *(table[prmt->Index].DebugParam),YELLOW,BLUE);
//				
//		   if(prmt->MENUNUM==1)
//			 {
////				 lcd_showfloat(TFT_X_MAX-NUM,(4 + prmt->Cursor),data2[prmt->Cursor],3,1);
//			 }
//		   else if(prmt->MENUNUM==2)
//			 { 
//				 if(prmt->Cursor==0)
//				 {
//				   lcd_MYshowstr(0,(4 + prmt->Cursor),"+2h/1h",BLACK,YELLOW);
//				 }
//				  if(prmt->Cursor==2)
//				 {
//				   lcd_MYshowstr(0,(4 + prmt->Cursor),"+1h/10",BLACK,YELLOW);
//				 }
//				 lcd_showfloat(TFT_X_MAX-NUM,(4 + prmt->Cursor),data3[prmt->Cursor],3,2);
//			 } 
//			 else if(prmt->MENUNUM==3)
//			 { 
//				 lcd_showfloat(TFT_X_MAX-NUM,(4 + prmt->Cursor),data4[prmt->Cursor],3,1);
//			 } 
//			 else if(prmt->MENUNUM==4)
//			 {
//				 lcd_showfloat(TFT_X_MAX-NUM,(5 + prmt->Cursor),data7[prmt->Cursor],3,1);
//			 }
//			 
	
        adjustParam(site, table[prmt->Index].DebugParam);
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
//在菜单开中断扫线找边线有问题
//void Menu_Process_NearScan(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)  //处理扫线
//{
//  
//  uint8 key = KeyNone;
//  Site_t site;
//  uint8 page;    //显示菜单需要的页数

//  
//  if(num - PAGE_DISP_NUM <= 0)
//    page = 1;
//  else
//  {
//    page = num - PAGE_DISP_NUM + 1;
//    num = PAGE_DISP_NUM;
//  }
//  // 显示项数和页数设置
//  Menu_PrmtInit(prmt, num, page); 
//	
//	motor_menuflag_key=0;
//	
//	EnableGlobalIRQ();//开启总中断（扫线的情况才开启，一般菜单都关闭，若开启总中断注意电机是否会开启）
//  do
//  {
////    lcd_showstr(0, 0, menuName);    // 显示菜单标题
////    
////    // 显示菜单项
////    Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);

////  	lcd_showstr(TFT_X_MAX-NUM,(1 + prmt->Cursor),"<<<");
//    key = Key_Scan(); 		// 获取按键
//    near_lcd();
////		steer_control();

//		if ( Menu_Move(prmt, key) == 0 )           // 菜单移动，按下确认键
//    {
//      // 判断此菜单项有无需要调节的参数，有则进入参数调节
//      if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Plan_Null)
//      {
//        site.x = TFT_X_MAX-NUM;	site.y = (1 + prmt->Cursor);
//        adjustParam(site, table[prmt->Index].DebugParam);
//      }
//      // 不是参数调节的话就执行菜单函数
//      else
//      {
//        table[prmt->Index].ItemHook();         // 执行相应函数
//      }
//		}
//  }while (prmt->ExitMark == 0);
//	
//	DisableGlobalIRQ();//关闭总中断（扫线的情况才开启，一般菜单都关闭，若开启总中断注意电机是否会开启）
//  lcd_clear(BLACK);

//  delay_ms(50);
//}
//							
//void Menu_Process_FarScan(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)  //处理扫线
//{
//  uint8 key = KeyNone;
//  Site_t site;
//  uint8 page;    //显示菜单需要的页数
//  
//  if(num - PAGE_DISP_NUM <= 0)
//    page = 1;
//  else
//  {
//    page = num - PAGE_DISP_NUM + 1;
//    num = PAGE_DISP_NUM;
//  }
//  // 显示项数和页数设置
//  Menu_PrmtInit(prmt, num, page); 
//	
//	motor_menuflag_key=0;
//	
//	EnableGlobalIRQ();//开启总中断（扫线的情况才开启，一般菜单都关闭，若开启总中断注意电机是否会开启）
//  do
//  {
////    lcd_showstr(0, 0, menuName);    // 显示菜单标题
////    
////    // 显示菜单项
////    Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
//    key = Key_Scan();                       // 获取按键
//    far_lcd();
//		if ( Menu_Move(prmt, key) == 0 )           // 菜单移动，按下确认键
//    {
//      // 判断此菜单项有无需要调节的参数，有则进入参数调节
//      if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Plan_Null)
//      {
//        site.x = TFT_X_MAX-NUM;	site.y = (1 + prmt->Cursor);
//        adjustParam(site, table[prmt->Index].DebugParam);
//      }
//      // 不是参数调节的话就执行菜单函数
//      else
//      {
//        table[prmt->Index].ItemHook();         // 执行相应函数
//      }
//		}
//  }while (prmt->ExitMark == 0);
//	DisableGlobalIRQ();//关闭总中断（扫线的情况才开启，一般菜单都关闭，若开启总中断注意电机是否会开启）
//  lcd_clear(BLACK);
//  delay_ms(50);
//}
//二级菜单函数     处理二级菜单参数

void Plan_Null()
{
	delay_ms(100);
}


void MainMenu_SetStore(void)                                                                                                                            //主菜单处理                 
	
{
	uint8 menuNumStore = sizeof(MainMenuStore_Table)/sizeof(MainMenuStore_Table[0]);    // 菜单项数

	lcd_clear(BLACK);

  Menu_Process(" -= SettingStore =- ", &MainMenuStore_Prmt, MainMenuStore_Table, menuNumStore);    	

}
void MainMenu_SetA(void)                                                                                                                            //主菜单处理A                  

{
	uint8 menuNumA = sizeof(MainMenuA_Table)/sizeof(MainMenuA_Table[0]);    // 菜单项数

	lcd_clear(BLACK);

  Menu_Process(" -= SettingA =- ", &MainMenuA_Prmt, MainMenuA_Table, menuNumA);    	

}

void MainMenu_SetB(void)                                                                                                                           //主菜单处理B     
	
{
	uint8 menuNumB = sizeof(MainMenuB_Table)/sizeof(MainMenuB_Table[0]);    // 菜单项数

  lcd_clear(BLACK);

  Menu_Process(" -= SettingB =- ", &MainMenuB_Prmt, MainMenuB_Table, menuNumB);   
  
}
//void MainMenu_Nearset(void)                                                                                                                                            
//	
//{
//	uint8 menuNumSN = sizeof(MainMenuSN_Table)/sizeof(MainMenuSN_Table[0]);    // 菜单项数

//	lcd_clear(BLACK);

//  Menu_Process_NearScan(" -= ScanNear =- ", &MainMenuSN_Prmt, MainMenuSN_Table, menuNumSN);    	

//}

//void  MainMenu_Farset(void)
//	
//{
//	uint8 menuNumSF = sizeof(MainMenuSF_Table)/sizeof(MainMenuSF_Table[0]);    // 菜单项数

//  lcd_clear(BLACK);

//  Menu_Process_FarScan(" -= ScanFar =- ", &MainMenuSF_Prmt, MainMenuSF_Table, menuNumSF);   
//  
//}

//三级菜单函数(无下级)     //函数处理三级级菜单参数   
void Menu_Null()
{
  delay_ms(100);
}

void Menu_Store()                            
{
	lcd_clear(BLACK);
	Menu_ProcessPro("1.Store", &Store_Prmt, Store_MenuTable, sizeof(Store_MenuTable)/sizeof(Store_MenuTable[0]));
	delay_ms(100);
}
void Menu_Motor_A()
{
	lcd_clear(BLACK);
	Menu_ProcessPro("1.Motor      ", &MotorA_Prmt, MotorA_MenuTable, sizeof(MotorA_MenuTable)/sizeof(MotorA_MenuTable[0]));
	delay_ms(100);
}
void Menu_Motor_B()
{
	lcd_clear(BLACK);
	Menu_ProcessPro("1.Motor       ", &MotorB_Prmt, MotorB_MenuTable, sizeof(MotorB_MenuTable)/sizeof(MotorB_MenuTable[0]));
	delay_ms(100);
}
void Menu_Error_A()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("2.Error       ", &ErrorA_Prmt, ErrorA_MenuTable, sizeof(ErrorA_MenuTable)/sizeof(ErrorA_MenuTable[0]));
	delay_ms(100);
}
void Menu_Error_B()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("2.Error        ", &ErrorB_Prmt, ErrorB_MenuTable, sizeof(ErrorB_MenuTable)/sizeof(ErrorB_MenuTable[0]));
	delay_ms(100);
}
void Menu_V_A()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("3.VDebug        ", &VA_Prmt, VA_MenuTable, sizeof(VA_MenuTable)/sizeof(VB_MenuTable[0]));
	delay_ms(100);
}
void Menu_V_B()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("3.VDebug        ", &VB_Prmt, VB_MenuTable, sizeof(VA_MenuTable)/sizeof(VB_MenuTable[0]));
	delay_ms(100);
}

void Menu_Order_A()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("4.Order          ", &OrderA_Prmt, OrderA_MenuTable, sizeof(OrderA_MenuTable)/sizeof(OrderA_MenuTable[0]));
	delay_ms(100);
}
void Menu_Order_B()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("4.Order         ", &OrderB_Prmt, OrderB_MenuTable, sizeof(OrderB_MenuTable)/sizeof(OrderB_MenuTable[0]));
	delay_ms(100);
}
void Menu_Ccd_A()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("5.Ccd           ", &CcdA_Prmt, CcdA_MenuTable, sizeof(CcdA_MenuTable)/sizeof(CcdA_MenuTable[0]));
	delay_ms(100);
}
void Menu_Ccd_B()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("5.Ccd             ", &CcdB_Prmt, CcdB_MenuTable, sizeof(CcdB_MenuTable)/sizeof(CcdB_MenuTable[0]));
	delay_ms(100);
}
void Menu_Circle_A()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("6.Circle          ", &CircleA_Prmt, CircleA_MenuTable, sizeof(CircleA_MenuTable)/sizeof(CircleA_MenuTable[0]));
	delay_ms(100);
}
void Menu_Circle_B()
{
  lcd_clear(BLACK);
	Menu_ProcessPro("6.Circle            ", &CircleB_Prmt, CircleB_MenuTable, sizeof(CircleB_MenuTable)/sizeof(CircleB_MenuTable[0]));
	delay_ms(100);
}
