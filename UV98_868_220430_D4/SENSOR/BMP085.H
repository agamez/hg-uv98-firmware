   

#ifndef _BMP085_H_
#define _BMP085_H_


#define   uchar unsigned char	   // 0-255 single byte, use uchar instead of char
#define   uint unsigned int		   // 0-65536 double bytes use uint instead of int

 
extern uchar QY[10];       

// extern long BMP280_QY;

extern uchar bmp085Convert();
// extern void Init_BMP085();

extern bit BMP085_LINK;	// 0=Not installed 1=Installed

/*****************************************/
#endif