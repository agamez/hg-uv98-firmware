#define   uchar unsigned char	   //0-255单字节	用uchar 代替char
#define   uint unsigned int		   //0-65536双字节	用uint 代替int

#ifndef _LED_H_
#define _LED_H_


/*************  外部函数和变量声明 *****************/



extern void LED_SHOW(uchar LED,uchar n);	 //指定的LED闪烁几次

#endif