

#ifndef _CMX865A_DECODE_H_
#define _CMX865A_DECODE_H_


/*************  外部函数和变量声明 *****************/

  
extern uchar CMX865A_HDLC_RX() ;   // 独占解码方式

extern uchar CMX865A_HDLC_RX_2();  // 中断解码方式

extern void CMX_RX_Initial();	//定时中断

extern void CMX_RX_INT();	//定时中断 ,5ms中断一次

extern unsigned char CMX_RX_BUSY;

/*****************************************/
#endif