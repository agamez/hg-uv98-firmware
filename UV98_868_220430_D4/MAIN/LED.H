#define   uchar unsigned char	   // 0-255 single byte, use uchar instead of char
#define   uint unsigned int		   // 0-65536 double bytes use uint instead of int

#ifndef _LED_H_
#define _LED_H_


/************* External function and variable declaration*****************/



extern void LED_SHOW(uchar LED,uchar n);	 // The specified LED flashes several times

#endif