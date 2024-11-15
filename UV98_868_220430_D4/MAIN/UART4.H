
#ifndef _UART4_H_
#define _UART4_H_

 
extern void UART4_Initial();		 // Initialize serial port 4
extern void UART4_FUN();

extern void UART4_SendData(uchar dat) ;
extern void UART4_SendString(char *s);

extern unsigned char UART4_RX_TIME;	// UARTx Receive Data Timing


extern void UART4_TX_EEROM() ;
extern void UART4_TX_liebiao();


extern unsigned char  UART4_BUF_DATA[600] ;		  // Serial port buffer size
extern unsigned int  UART4_BUF_LENTH;




extern uchar UART1_SET()  ;




#endif