

#ifndef _tostring_H_
#define _tostring_H_
//公用函数和公用变量声明


extern uchar wan, qian, bai, shi, ge;
extern void tostring(uint val); //转换成3个字节

extern uchar str_txt[10];


extern void disp_Hex2Ascii2(uchar dat);

extern void READ_CPU_ID();	 //读取CPU序列号


extern void DEBUG_KISS(uchar *p, uint len);

extern uchar CPU_ID[16];	//读取CPU序列号

extern uchar TEMP_Call[10];	//临时呼号
extern void READ_TEMP_CALL(uint call_eerom_add, uint SSID_eerom_add);	 //  读出临时呼号




extern void disp_Hex2Ascii3(uint temp)	 ;



#endif