

#ifndef _CRC16_H_
#define _CRC16_H_


/************* 16位CRC校验 **************/
extern unsigned char  FCS_LO,FCS_HI;						//校验后的校验值
//void GetCrc16(const uchar *pData);	  				//计算未知长度数据的16位CRC校验数据，遇到0X00停止 ，输出 FCS_LO,FCS_HI
extern void GetCrc16_LEN(const unsigned char *pData,unsigned char nLength);	  // 计算给定长度数据的16位CRC，输出 FCS_LO,FCS_HI

/*****************************************/
#endif