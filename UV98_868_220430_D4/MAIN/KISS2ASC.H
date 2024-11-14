

#ifndef _KISS2ASC_H_
#define _KISS2ASC_H_


/*************  外部函数和变量声明 *****************/
 //电台接收到的  KISS数据转换ASCII UI格式,,format  0=原始数据  1=插入本机呼号



extern void KISS_TO_ASCII( unsigned char *nbuff,unsigned char format) ;

/*****************************************/


#endif