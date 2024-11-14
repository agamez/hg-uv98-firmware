
#ifndef _IO_H_
#define _IO_H_


/************* 引脚端口 **************/


sbit LED_STU =P2^6;


sbit FX604_TX=P1^5;

sbit PTT 	 =P2^3;
sbit GPS_EN	 =P2^7;	 //GPRS开关


sbit BL_PWR  =P3^6;	  //蓝牙电源
sbit BL_RST  =P3^5;	  //蓝牙复位
sbit BL_STU  =P3^2;	  //蓝牙连接状态

 
sbit	BMP280_SCL=P4^1;      //IIC时钟引脚定义
sbit	BMP280_SDA=P2^2;      //IIC数据引脚定义

 
sbit CMX865A_SCS	=P4^4;	//	CBUS	 定义CBUS的片选信号端口 
sbit CMX865A_SCLK	=P1^3;	//	CBUS	 定义CBUS的时钟信号端口
//sbit CMX865A_RST	=P0^3;	//	CBUS
sbit CMX865A_MOSI	=P1^4;	//	CBUS	 定义CBUS的MOSI端口 
sbit CMX865A_MISO	=P1^2;	//	CBUS	 定义CBUS的MISO端口
sbit CMX865A_PW	=P0^7;	//	PW	  



#endif