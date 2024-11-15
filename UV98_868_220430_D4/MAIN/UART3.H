
#ifndef _UART3_H_
#define _UART3_H_

 extern void UART3_Initial();		 // Initialize serial port 2
extern void UART3_FUN();

// extern void UART3_SendData(float data) ;
// extern void UART3_SendString(char *s);

 
extern unsigned char   UART3_GPRMC_DATA[128] ;		  // Serial port buffer size
extern unsigned char   UART3_GPGGA_DATA[128] ;		  // Serial port buffer size

 
extern uchar UART3_OUTTIME;  // Receive data count, used to determine whether GPS is disconnected

#endif