


#ifndef _KISS2GAOMIN_H_
#define _KISS2GAOMIN_H_

 
/************* External function and variable declaration*****************/
extern void UI_TO_GPWPL();
extern unsigned char GPWPL_BUF[60];	// Waypoint data cache length &lt;50
extern void KISS_TO_GPWPL();		// Convert KISS data to GPWPL waypoint format

extern void MY_BEACON_TO_GPWPL();		// Convert your own beacon directly into GPWPL waypoint format


/*****************************************/


#endif