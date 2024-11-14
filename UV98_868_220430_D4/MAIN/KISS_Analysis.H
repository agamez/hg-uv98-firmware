
#ifndef _KISS_Analysis_H_
#define _KISS_Analysis_H_


//extern void DISP_HX(uchar txt_id,uint JD);	 //指定的文本，显示航向
//extern void DISP_JD(uchar txt_id,uint JD);	 //指定的文本，显示2点经纬度的相对正北的角度
//extern void CLEAN_UI_DATA();

//extern uchar Resolution_UI_DATA();	 //解析UI数据类型

extern void DISP_KISS_DATA() ;  //解析并显示对方的定位数据,并显示


extern uchar UI_CALL[10];
//extern uchar UI_SSID[3];
extern uchar UI_DEST[10];
extern uchar UI_TIME[10];
extern uchar UI_WD[10];
extern uchar UI_WD_DIR;
extern uchar UI_JD[10];
extern uchar UI_JD_DIR;
extern uchar UI_ICON;

extern uchar UI_DIR[10];
extern uchar UI_SPEED[10];
extern uchar UI_ALT[10];
extern uchar UI_JULI[10];

extern uint UI_Angle_N;	  //相对正北方位

extern uint UI_Angle_CAR;  //相对车头方位

extern float UI_ALT_MI;	 //信标海拔高度
 
extern uint Elevation_angle; //相对仰角



#endif