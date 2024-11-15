#include "STC12C5A32S2.H"
#define   uchar unsigned char	   // 0-255 single byte, use uchar instead of char
#define   uint unsigned int		   // 0-65536 double bytes use uint instead of int


#include "IO.H"
#include "DELAY.H"
#include "LED.H"

void LED_SHOW(uchar LED,uchar n);	 // The specified LED flashes several times



/*------------------------------------------------------------------*/
void LED_SHOW(uchar LED,uchar n)	 // Which LED flashes, flashes N times
{
	n=n*2;


	switch (LED)
	{
		case 1:
			 while (n--)
			    {
				LED_RED=!LED_RED;
				Delay_time_25ms(1);  	// Unit: 50ms
			   	}
				LED_RED=1;
			break;
		case 2:
			 while (n--)
			    {
				LED_GREEN=!LED_GREEN;
				Delay_time_25ms(1);  	// Unit: 50ms
			   	}
				LED_GREEN=1;
			break;
		default:
			  while (n--)
			    {
				LED_RED=!LED_RED;
				LED_GREEN=!LED_GREEN;
				LED_BLUE=!LED_BLUE;
				Delay_time_25ms(1);  	// Unit: 50ms
			   	}
				LED_GREEN=1;
				LED_RED=1;
				LED_BLUE=1;
			break;
	}

   
}


