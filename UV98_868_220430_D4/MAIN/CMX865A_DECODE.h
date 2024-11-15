

#ifndef _CMX865A_DECODE_H_
#define _CMX865A_DECODE_H_


/************* External function and variable declaration*****************/

  
extern uchar CMX865A_HDLC_RX() ;   // Exclusive decoding method

extern uchar CMX865A_HDLC_RX_2();  // Interrupt decoding method

extern void CMX_RX_Initial();	// Timer interrupt

extern void CMX_RX_INT();	// Timed interrupt, 5ms interrupt once

extern unsigned char CMX_RX_BUSY;

/*****************************************/
#endif