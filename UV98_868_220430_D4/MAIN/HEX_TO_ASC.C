#include "STC15Fxxxx.H"
#include "HEX_TO_ASC.H"
	
//unsigned char ASC_BUF[8];
//
//void HEX_TO_ASC(int a)	    //2字节HEX转成ASC文本，首位0消隐，文本保存在 ASC_BUF
//{
//	unsigned char i;
//	unsigned char zero;
//
//	zero=1;
//	if(a<0)	{ 	a=-a;  zero=0;}	 //负数转成正数，置负数标记
//
//	for(i=0;i<8;i++)	 {ASC_BUF[i]=0x00;}
//
//	if (zero==1){ASC_BUF[0]=' ';}	else{  ASC_BUF[0]='-';}
//   	if (a==0){ASC_BUF[1]='0'; return;}
//
//	ASC_BUF[1]=(a/10000+0x30);
//	ASC_BUF[2]=(a/1000%10+0x30);
//	ASC_BUF[3]=(a/100%10+0x30);
//	ASC_BUF[4]=(a/10%10+0x30);
//	ASC_BUF[5]=(a%10+0x30);
//
//	while(ASC_BUF[1]=='0')	 //首位0消隐
//	{
//	for(i=1;i<6;i++) 	{ ASC_BUF[i]=	ASC_BUF[i+1]; }
//	}
//}
//
//void HEX_TO_ASC(unsigned int a)	    //2字节HEX转成ASC文本，首位0消隐，文本保存在 ASC_BUF
//{
//	unsigned char i;
//
//	for(i=0;i<8;i++)	 {ASC_BUF[i]=0x00;}
//
//   	if (a==0){ASC_BUF[0]='0'; return;}
//
//	ASC_BUF[0]=(a/10000+0x30);
//	ASC_BUF[1]=(a/1000%10+0x30);
//	ASC_BUF[2]=(a/100%10+0x30);
//	ASC_BUF[3]=(a/10%10+0x30);
//	ASC_BUF[4]=(a%10+0x30);
//
//	while(ASC_BUF[0]=='0')	 //首位0消隐
//	{
//	for(i=0;i<5;i++) 	{ ASC_BUF[i]=	ASC_BUF[i+1]; }
//	}
//}
//
