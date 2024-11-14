

#ifndef _PTT_FUN_H_
#define _PTT_FUN_H_

#define   uchar unsigned char	   //0-255单字节	用uchar 代替char
#define   uint unsigned int		   //0-65536双字节	用uint 代替int
/************* 16位CRC校验 **************/
extern uchar PTT_IN_DOWN(); //检查PTT按下
extern void PTT_IN_UP(); //等待按键松开

/*****************************************/
#endif