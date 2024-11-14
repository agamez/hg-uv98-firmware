   

#ifndef _BMP085_H_
#define _BMP085_H_


#define   uchar unsigned char	   //0-255单字节	用uchar 代替char
#define   uint unsigned int		   //0-65536双字节	用uint 代替int

 
extern uchar QY[10];       

//extern long BMP280_QY;

extern uchar bmp085Convert();
//extern void Init_BMP085();

extern bit BMP085_LINK;	//0=没安装	1=安装

/*****************************************/
#endif