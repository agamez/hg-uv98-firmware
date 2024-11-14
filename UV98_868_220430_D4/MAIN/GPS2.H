

#ifndef _GPS2_H_
#define _GPS2_H_

 
/*************  外部函数和变量声明 *****************/

extern void	GPS_INIT();
extern void UART3_RX_GPS();	  //调试UART3  GPS数据
extern bit GPS_LOCKED;

extern uchar GPS_TIME[10];	//时间
extern uchar GPS_DATE[13];	//日期

extern uchar GPS_WD[15];	//纬度
extern uchar GPS_JD[15];    //经度
extern uint GPS_NOW_DIR;	//GPS当前方向0-359度
extern uint GPS_NOW_SPEED_KNOT;	//提取的GPS当前速度0-255海里  1海里=1.852公里
extern uint GPS_SPEED_KM;	//GPS速度 单位公里


extern uchar  GPS_ALT_ERR;		//GPS没有效定位，1=无海拔数据，则数据无效  0=海拔数据正常
extern uchar GPS_HEIGHT[7];	//海拔高度度
extern long GPS_ALT_MI;

extern uchar GPS_SIG[3];    //定位的卫星数量	00-12

extern uint GPS_WAIT_TIME; //(0-255)*10秒




extern void BECON_MODE();	//定时信标
extern long GPS_ALT_MI;
extern uchar SMART_MODE;




extern uint GPS_NOW_DIR;		//GPS当前方向0-359度
extern uchar GPS_NOW_DIR_TXT[4];



extern bit GPS_DIR_NOTHING;	//0=静止，无航向数据  1=移动中，有航向数据



//extern void LC_MEMORY();  //关机记录里程

extern void LC_CLEAN();  //里程清0
extern unsigned  long  TOTAL_LC,AB_LC;    // 0-6500.0km
//extern uchar LC_TIME;   //里程周期A-Z
extern uchar GET_LC();


extern bit QUICK_SET_EN;	//0= 禁止快速设置固定站		 1=允许快速设置固定站

//extern void QUICK_SET_FIXED();	  //快速修改固定站经纬度 

#endif