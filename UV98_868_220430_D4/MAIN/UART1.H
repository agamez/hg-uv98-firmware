

#ifndef _UART1_H_
#define _UART1_H_

 
extern void UART1_Initial();	// 初始化串口
extern void UART1_FUN();	//处理串口接收到的数据 ,解析数据

extern void UART1_SendData(uchar dat) ;
extern void UART1_SendString(char *s);
//extern bit UART1_RX_BUSY;	   //串口1接收数据等待处理

extern unsigned char UART1_BUF_DATA[600];	//接收数据缓存
//extern unsigned char UART1_BUF_LENTH;	//串口收发数据长度

//extern void UART1_select_p3031();
//extern void UART1_select_p3637();

extern void UART1_DEBUG(unsigned int n);   //调试输出整数
extern void UART1_DEBUG2(unsigned int n);	   //调试输出整数
	
//extern void UART1_DEBUG_AA(char *s );

//extern unsigned char CHECK_AT_CMD(const unsigned char *cmd);	   //对比AT指令


extern unsigned char UART1_RX_TIME;	//UARTx  接收数据计时


#endif