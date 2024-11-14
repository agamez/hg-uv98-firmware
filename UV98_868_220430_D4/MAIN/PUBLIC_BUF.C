#include "STC8A8K64D4.H"
#include "PUBLIC_BUF.H"


 
 
unsigned char SN_RX_BUFFER[250]=0;	//解码后的KISS数据，不含C0 00 ..C0,  长度<128
 
unsigned char KISS_DATA[200];	//解码后的KISS数据，不含C0 00 ..C0,  长度<128
unsigned char KISS_LEN;			//解码后的KISS数据长度

 
uchar ASC_TEMP[300];		//临时文本数据
 
uint   TIME_1S;
 
uchar GPS_LINK;	   //GPS是否连接

uchar A20_OUT_EN;
