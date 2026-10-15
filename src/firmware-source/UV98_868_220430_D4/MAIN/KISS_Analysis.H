
#ifndef _KISS_Analysis_H_
#define _KISS_Analysis_H_


// extern void DISP_HX(uchar txt_id,uint JD); //Specified text, display heading
// extern void DISP_JD(uchar txt_id,uint JD); //Specified text, showing the angle of the two points&#39; latitude and longitude relative to due north
// extern void CLEAN_UI_DATA();

// extern uchar Resolution_UI_DATA(); //Analysis of UI data type

extern void DISP_KISS_DATA() ;  // Analyze and display the other party&#39;s positioning data, and display


extern uchar UI_CALL[10];
// extern fly UI_SSID[3];
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

extern uint UI_Angle_N;	  // Relative North

extern uint UI_Angle_CAR;  // Relative position of vehicle head

extern float UI_ALT_MI;	 // Beacon altitude
 
extern uint Elevation_angle; // Relative elevation angle



#endif