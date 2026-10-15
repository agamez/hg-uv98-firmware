
#ifndef _UARTx_H_
#define _UARTx_H_

 
 
extern unsigned char  UARTx_BUF[600] ;		  // Serial port buffer size

extern uint UARTx_BUF_LENTH;	// Length of serial port data sent and received

extern uchar UART_X_CMD(uchar UARTx) ;

extern unsigned char CHECK_AT_CMD(const unsigned char *cmd);	   // Compare AT commands


  
#endif