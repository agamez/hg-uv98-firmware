

#ifndef _CMX865A_SPI_H_
#define _CMX865A_SPI_H_


/************* External function and variable declaration*****************/



extern void CMX865A_Init();

extern void  CMX865A_TX_TONE(unsigned char tone);  // tone=1 1200  0=2200hz
extern void CMX865A_833US();	  // Use TIME1 as timer, delay 833us



extern void CMX865A_TX_DATA( unsigned char dat ) ;
extern uchar CMX865A_RX_DATA();
extern uchar CMX865A_DET();


extern uint CMX865A_READ_E6() ; // E6

extern uchar CMX865A_READ_E5();


extern void CMX865A_TX_ON();
extern void CMX865A_TX_OFF() ;
extern void CMX865A_TONE_TEST() ;


								
extern void TONE1200();
extern void TONE2200()	;
extern void TONE_OFF()	;


extern void DISP_HDLC(uchar dat);	// Convert to high and low symbol display, for debugging


extern void CMX865A_PWUP()	;   // Power-down mode


extern void CMX_RX_ON();


/*****************************************/
#endif