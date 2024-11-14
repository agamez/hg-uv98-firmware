
#ifndef _UARTx_H_
#define _UARTx_H_

 
 
extern unsigned char  UARTx_BUF[600] ;		  //串口缓冲尺寸

extern uint UARTx_BUF_LENTH;	//串口收发数据长度

extern uchar UART_X_CMD(uchar UARTx) ;

extern unsigned char CHECK_AT_CMD(const unsigned char *cmd);	   //对比AT指令


  
#endif