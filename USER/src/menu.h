#ifndef __MENU_H__
#define __MENU_H__

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


extern void EEPROM_Read();
extern void EEPROM_Write();
extern void Buzzer();

extern void MainMenu_Set(void);
extern void Menu_Null();
extern void Menu_Motor();
extern void Menu_Speed();
extern void Menu_ADC();
extern void Menu_ADC_Max();
extern void Menu_ADC_Min();
extern void Menu_ADvalue();
extern void Menu_Round();
extern void Menu_RoundType();
extern void Menu_Servo();
extern void Menu_Type();
extern void Menu_Thre();
extern void Menu_Barrier();
extern void Menu_BarrierType();
extern void Menu_Buzz();

extern void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num);
extern void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor);
extern uint8 Menu_Move(MENU_PRMT *prmt, uint8 key);
extern void adjustParam(Site_t site, uint16 *param, uint16 BGcolor, uint16 PENcolor);
#endif
