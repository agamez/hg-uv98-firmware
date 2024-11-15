

#ifndef _KISS2ASC_H_
#define _KISS2ASC_H_


/************* External function and variable declaration*****************/
 // Convert the received KISS data to ASCII UI format, format 0 = original data 1 = insert the local call sign



extern void KISS_TO_ASCII( unsigned char *nbuff,unsigned char format) ;

/*****************************************/


#endif