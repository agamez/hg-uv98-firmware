
#include "STC15Fxxxx.H"
 #include "IO.H"
 #include "PTT_FUN.H"
 
#include "DELAY.H"
 
 
 #define   uchar unsigned char	   // 0-255 single byte, use uchar instead of char
#define   uint unsigned int		   // 0-65536 double bytes use uint instead of int



sbit PTT_IN		=P3^3;	 // Manual PTT
  
uchar PTT_IN_DOWN() // Check PTT Press
{
  	if (PTT_IN==1)	{return 0; }  // No button pressed, exit
	Delay_time_25ms(4);
	if (PTT_IN==1) {return 0; }	   // Stabilization
	// Confirm that the button is pressed
	return 1;
}



void PTT_IN_UP() // Wait for the button to be released
{
	while(1)			   // Wait for the button to be released and then restart the CPU
	{
		if (PTT_IN==1)  
		{
		Delay_time_25ms(4);	 // Delay 100MS
		if (PTT_IN==1) 	 	{	return;	}  	// The button is actually released
		}
	}
}