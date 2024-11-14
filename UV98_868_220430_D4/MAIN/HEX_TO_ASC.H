#ifndef _HEX_TO_ASC_H_
#define _HEX_TO_ASC_H_

extern unsigned char ASC_BUF[8];

extern void HEX_TO_ASC(int a);	 //2字节HEX转成ASC文本，首位0消隐，文本保存在 ASC_BUF
//extern void HEX_TO_ASC(unsigned int a);	 //2字节HEX转成ASC文本，首位0消隐，文本保存在 ASC_BUF

#endif