
#include "STC15Fxxxx.H"
 #include "IO.H"
 #include "PTT_FUN.H"
 
#include "DELAY.H"
 
 
 #define   uchar unsigned char	   //0-255单字节	用uchar 代替char
#define   uint unsigned int		   //0-65536双字节	用uint 代替int



sbit PTT_IN		=P3^3;	 //手动PTT
  
uchar PTT_IN_DOWN() //检查PTT按下
{
  	if (PTT_IN==1)	{return 0; }  //按键没按下，退出
	Delay_time_25ms(4);
	if (PTT_IN==1) {return 0; }	   //防抖
	//确定按键已按下
	return 1;
}



void PTT_IN_UP() //等待按键松开
{
	while(1)			   //等待按键松开后，重启CPU
	{
		if (PTT_IN==1)  
		{
		Delay_time_25ms(4);	 //延时100MS
		if (PTT_IN==1) 	 	{	return;	}  	//按键确实松开
		}
	}
}