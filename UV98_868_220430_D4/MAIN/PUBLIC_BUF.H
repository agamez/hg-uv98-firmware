#ifndef	_PUBLIC_BUF_H_
#define	_PUBLIC_BUF_H_
 
 
extern unsigned char KISS_DATA[200];	//解码后的KISS数据，不含C0 00 ..C0,  长度<128
extern unsigned char KISS_LEN;			//解码后的KISS数据长度
extern unsigned char SN_RX_BUFFER[250];	//UI数据缓冲,  长度<128
 
//
extern uchar ASC_TEMP[300];		//临时文本数据
 
extern uint   TIME_1S;
 
extern uchar GPS_LINK;	   //GPS是否连接

extern uchar A20_OUT_EN;


#endif

