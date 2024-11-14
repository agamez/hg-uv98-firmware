


#ifndef _KISS2GAOMIN_H_
#define _KISS2GAOMIN_H_

 
/*************  外部函数和变量声明 *****************/
extern void UI_TO_GPWPL();
extern unsigned char GPWPL_BUF[60];	//航点数据缓存  长度<50
extern void KISS_TO_GPWPL();		//KISS数据转成GPWPL航点格式

extern void MY_BEACON_TO_GPWPL();		//自己的信标直接转成GPWPL航点格式


/*****************************************/


#endif