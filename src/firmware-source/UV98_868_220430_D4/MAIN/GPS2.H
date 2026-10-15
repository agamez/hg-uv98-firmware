

#ifndef _GPS2_H_
#define _GPS2_H_

 
/************* External function and variable declaration*****************/

extern void	GPS_INIT();
extern void UART3_RX_GPS();	  // Debug UART3 GPS data
extern bit GPS_LOCKED;

extern uchar GPS_TIME[10];	// time
extern uchar GPS_DATE[13];	// date

extern uchar GPS_WD[15];	// latitude
extern uchar GPS_JD[15];    // longitude
extern uint GPS_NOW_DIR;	// GPS current direction 0-359 degrees
extern uint GPS_NOW_SPEED_KNOT;	// The current speed extracted from GPS is 0-255 nautical miles. 1 nautical mile = 1.852 kilometers.
extern uint GPS_SPEED_KM;	// GPS speed in kilometers


extern uchar  GPS_ALT_ERR;		// GPS has no valid positioning, 1 = no altitude data, then the data is invalid 0 = altitude data is normal
extern uchar GPS_HEIGHT[7];	// Altitude
extern long GPS_ALT_MI;

extern uchar GPS_SIG[3];    // Number of satellites located 00-12

extern uint GPS_WAIT_TIME; // (0-255)*10 seconds




extern void BECON_MODE();	// Timing Beacon
extern long GPS_ALT_MI;
extern uchar SMART_MODE;




extern uint GPS_NOW_DIR;		// GPS current direction 0-359 degrees
extern uchar GPS_NOW_DIR_TXT[4];



extern bit GPS_DIR_NOTHING;	// 0 = stationary, no heading data 1 = moving, heading data available



// extern void LC_MEMORY(); //shutdown and record mileage

extern void LC_CLEAN();  // Mileage reset to 0
extern unsigned  long  TOTAL_LC,AB_LC;    // 0-6500.0km
// extern uchar LC_TIME; //Mileage cycle AZ
extern uchar GET_LC();


extern bit QUICK_SET_EN;	// 0 = Disable fast station setup 1 = Enable fast station setup

// extern void QUICK_SET_FIXED(); //Quickly modify the longitude and latitude of the fixed station

#endif