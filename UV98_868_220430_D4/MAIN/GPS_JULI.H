/***********************************************
************************************************/
#ifndef _GPSJULI_H_
#define _GPSJULI_H_

 /*****************************************/
extern void GET_AB_JULI(uchar mode);	//求2地距离
extern uint GET_AB_Angle(uchar mode);	//求2点经纬度的相对正北的角度


extern double GET_JULI(uchar *a_lat,uchar *a_lng,uchar *b_lat,uchar *b_lng )	;

extern uint GET_AB_EL(uchar mode);	  //求2点 仰角  0-89度


extern uint GET_AB_POINT_DIR(uint A_POINT,uint GPS_DIR)	;	//相对车头方向
extern void angle_to_txt(uint angle,uchar *p);


/*****************************************/
#endif