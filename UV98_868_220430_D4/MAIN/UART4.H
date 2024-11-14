
#ifndef _UART4_H_
#define _UART4_H_

 
extern void UART4_Initial();		 //初始化串口4
extern void UART4_FUN();

extern void UART4_SendData(uchar dat) ;
extern void UART4_SendString(char *s);

extern unsigned char UART4_RX_TIME;	//UARTx  接收数据计时


extern void UART4_TX_EEROM() ;
extern void UART4_TX_liebiao();


extern unsigned char  UART4_BUF_DATA[600] ;		  //串口缓冲尺寸
extern unsigned int  UART4_BUF_LENTH;




extern uchar UART1_SET()  ;




#endif