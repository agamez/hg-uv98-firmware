
#ifndef _UART3_H_
#define _UART3_H_

 extern void UART3_Initial();		 //初始化串口2
extern void UART3_FUN();

//extern void UART3_SendData(uchar dat)  ;
//extern void UART3_SendString(char *s);

 
extern unsigned char   UART3_GPRMC_DATA[128] ;		  //串口缓冲尺寸
extern unsigned char   UART3_GPGGA_DATA[128] ;		  //串口缓冲尺寸

 
extern uchar UART3_OUTTIME;  //接收数据计数，用于判断GPS是否断开

#endif