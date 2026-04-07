#ifndef _MENU_PID_H_
#define _MENU_PID_H_

#include "headfile.h"

#define PAGE_DISP_NUM 6    // 定义LCD单页显示项目数
#define NUM 64             //数字坐标

typedef struct{

	uint8 ExitMark;     // 退出菜单(0-不退出，1-退出)

	uint8 Cursor;       // 光标值(当前光标位置)

	uint8 PageNo;       // 菜单页(显示开始项)

	uint8 Index;        // 菜单索引(当前选择的菜单项)

	uint8 DispNum;      // 显示项数(每页可以现在菜单项)

	uint8 MaxPage;      // 最大页数(最大有多少种显示页)
	
	uint8 MENUNUM;      //菜单序号(指定特定数组来显示最终数据)

}MENU_PRMT;      // 菜单参数

typedef struct{

	uint8 *MenuName;		// 菜单项目名称

	void(*ItemHook)(void);  // 要运行的菜单函数

	uint16 *DebugParam;		// 要调试的参数
	
}MENU_TABLE;     // 菜单执行


typedef struct
{
    uint8 x;
    uint8 y;
}Site_t;

extern void Plan_Null();
extern void MainMenu_Set();
extern void MainMenu_SetA();
extern void MainMenu_SetB();
extern void MainMenu_Nearset();
extern void MainMenu_Farset();   
extern void MainMenu_SetStore();     


extern void Menu_Null();
extern void Menu_Store();   
extern void Menu_Motor_A();
extern void Menu_Error_A();
extern void Menu_V_A();
extern void Menu_Order_A();
extern void Menu_Ccd_A();
extern void Menu_Circle_A();
              
extern void Menu_Motor_B();
extern void Menu_Error_B();
extern void Menu_V_B();
extern void Menu_Order_B();
extern void Menu_Ccd_B();
extern void Menu_Circle_B();

extern void EEPROM_ReadA();           
extern void EEPROM_WriteA();
extern void EEPROM_ReadB();           
extern void EEPROM_WriteB();

extern void Planchoose_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num); 
extern void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);        
extern void Menu_ProcessPro(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);
extern void Menu_Process_NearScan(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);  //处理扫线
extern void Menu_Process_FarScan(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);   //处理扫线
extern void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor);
extern uint8 Menu_Move(MENU_PRMT *prmt, uint8 key);
extern void adjustParam(Site_t site, uint16 *param);

#endif