

#ifndef _UART2_H_
#define _UART2_H_

 
extern void UART2_Initial();		 //初始化串口2
extern void UART2_FUN();

extern void UART2_SendData(uchar dat)  ;
extern void UART2_SendString(char *s);

extern unsigned char UART2_RX_TIME;	//UARTx  接收数据计时


extern void UART2_DEBUG(unsigned int n);	   //调试输出整数

#endif