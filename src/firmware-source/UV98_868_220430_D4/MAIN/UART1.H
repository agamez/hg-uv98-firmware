

#ifndef _UART1_H_
#define _UART1_H_

 
extern void UART1_Initial();	// Initialize the serial port
extern void UART1_FUN();	// Process the data received by the serial port and parse the data

extern void UART1_SendData(uchar dat) ;
extern void UART1_SendString(char *s);
// extern bit UART1_RX_BUSY; //Serial port 1 receives data and waits for processing

extern unsigned char UART1_BUF_DATA[600];	// Receive data buffer
// extern unsigned char UART1_BUF_LENTH; //Serial port send and receive data length

// extern void UART1_select_p3031();
// extern void UART1_select_p3637();

extern void UART1_DEBUG(unsigned int n);   // Debug output integer
extern void UART1_DEBUG2(unsigned int n);	   // Debug output integer
	
// extern void UART1_DEBUG_AA(char *s );

// extern unsigned char CHECK_AT_CMD(const unsigned char *cmd); //Compare AT commands


extern unsigned char UART1_RX_TIME;	// UARTx Receive Data Timing


#endif