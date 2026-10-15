

#ifndef _CRC16_H_
#define _CRC16_H_


/************* 16-bit CRC check**************/
extern unsigned char  FCS_LO,FCS_HI;						// Checksum after verification
// void GetCrc16(const uchar *pData); //Calculate the 16-bit CRC check data of unknown length data, stop when encountering 0X00, and output FCS_LO, FCS_HI
extern void GetCrc16_LEN(const unsigned char *pData,unsigned char nLength);	  // Calculate the 16-bit CRC of the given length data and output FCS_LO, FCS_HI

/*****************************************/
#endif