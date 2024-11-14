#include "STC12C5A32S2.H"
#define   uchar unsigned char	   //0-255单字节	用uchar 代替char
#define   uint unsigned int		   //0-65536双字节	用uint 代替int


#include "IO.H"
#include "DELAY.H"
#include "LED.H"

void LED_SHOW(uchar LED,uchar n);	 //指定的LED闪烁几次



/*------------------------------------------------------------------*/
void LED_SHOW(uchar LED,uchar n)	 //哪个LED 闪烁，闪烁 N次数
{
	n=n*2;


	switch (LED)
	{
		case 1:
			 while (n--)
			    {
				LED_RED=!LED_RED;
				Delay_time_25ms(1);  	//单位50ms
			   	}
				LED_RED=1;
			break;
		case 2:
			 while (n--)
			    {
				LED_GREEN=!LED_GREEN;
				Delay_time_25ms(1);  	//单位50ms
			   	}
				LED_GREEN=1;
			break;
		default:
			  while (n--)
			    {
				LED_RED=!LED_RED;
				LED_GREEN=!LED_GREEN;
				LED_BLUE=!LED_BLUE;
				Delay_time_25ms(1);  	//单位50ms
			   	}
				LED_GREEN=1;
				LED_RED=1;
				LED_BLUE=1;
			break;
	}

   
}


