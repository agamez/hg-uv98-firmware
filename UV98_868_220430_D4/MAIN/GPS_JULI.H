/***********************************************
************************************************/
#ifndef _GPSJULI_H_
#define _GPSJULI_H_

 /*****************************************/
extern void GET_AB_JULI(uchar mode);	// Find the distance between two places
extern uint GET_AB_Angle(uchar mode);	// Find the angle of the longitude and latitude of two points relative to true north


extern double GET_JULI(uchar *a_lat,uchar *a_lng,uchar *b_lat,uchar *b_lng )	;

extern uint GET_AB_EL(uchar mode);	  // Find the elevation angle of 2 points 0-89 degrees


extern uint GET_AB_POINT_DIR(uint A_POINT,uint GPS_DIR)	;	// Relative direction of vehicle head
extern void angle_to_txt(uint angle,uchar *p);


/*****************************************/
#endif