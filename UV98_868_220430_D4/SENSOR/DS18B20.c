
#include "STC15Fxxxx.H"
#include "SENSOR\DS18B20.h"
#include "tostring.H"
#include "MAIN\UART1.H"



#include  <INTRINS.H> // Keil library

#define   uchar unsigned char	   // 0-255 single byte, use uchar instead of char
#define   uint unsigned int		   // 0-65536 double bytes use uint instead of int
#include "HEX_TO_ASC.H"

long DS18B20_WENDU;
uchar  DS18B20_TEMP[10];

sbit DQ = P2^1;                     // DS18B20 data port is P3.3
uint TPH;                           // High byte to store temperature value
uint TPL;                           // Stores the low byte of the temperature value

void DelayXus(uchar n);
uchar DS18B20_Reset();
void DS18B20_WriteByte(uchar dat);
uchar DS18B20_ReadByte();


/*************************************************** 
	DS18B20 temperature reading program
****************************************************/ 
uchar DS18B20_READTEMP() 
{ 
    uchar i,n;
	uint temp;
	bit   ZERO; // 0 = above zero, 1 = below zero

	if (DS18B20_Reset()==0)	 {return 0;	 }
	
	// DS18B20_Reset(); //Device reset
    DS18B20_WriteByte(0xCC);        // Skip ROM command
    DS18B20_WriteByte(0x44);        // Start conversion command
    while (!DQ);                    // Wait for the conversion to complete

	if (DS18B20_Reset()==0)	{return 0; 	 }
	
    // DS18B20_Reset(); //Device reset
    DS18B20_WriteByte(0xCC);        // Skip ROM command
    DS18B20_WriteByte(0xBE);        // Read Scratchpad Memory Command
    TPL = DS18B20_ReadByte();       // Read temperature low byte
    TPH = DS18B20_ReadByte();       // Read temperature high byte

// UART1_SendData(0xaa);	UART1_SendData(TPH);		UART1_SendData(TPL);

	for(i=0;i<6;i++)   {DS18B20_TEMP[i]=0x00;}

	if ((TPH&0x80)==0x80)	  // Celsius&lt;0
	{
		ZERO=1;
		DS18B20_WENDU=-(~((TPH*256)+TPL)+1)*0.0625*10; 	 // Keep 1 decimal place
		// -0.1 ~ -54.5
		DS18B20_TEMP[0]='-';
	  	}
	else
	{					  // Celsius&gt;=0
		ZERO=0;
		DS18B20_WENDU=((TPH*256)+TPL)*0.0625*10; 	   // Keep 1 decimal place
		// 0~124.5
		DS18B20_TEMP[0]=' ';
	}
 	if (ZERO==1) { temp=(uint)-DS18B20_WENDU; }else{temp=(uint)DS18B20_WENDU;}	 
	



	tostring(temp);

	DS18B20_TEMP[1]=wan;
	DS18B20_TEMP[2]=qian;
	DS18B20_TEMP[3]=bai;
	DS18B20_TEMP[4]=shi;
	DS18B20_TEMP[5]='.';
	DS18B20_TEMP[6]=ge;
	DS18B20_TEMP[7]=0x00;

	n=0;
	while(DS18B20_TEMP[1]=='0')	 // The first 0 is blanked, at least one bit is retained
	{
	for(i=1;i<8;i++) 	{ DS18B20_TEMP[i]=DS18B20_TEMP[i+1]; }
	n++; if (n>2){break;}
	}

UART1_SendString("DS18B20: ");	UART1_SendString(DS18B20_TEMP);	UART1_SendString("\r\n");

	return 1;
}

/**************************************
Delay X microseconds (STC12C5A60S2@12M)
Different working environments require adjustment of this function
This delay function is calculated using a 1T instruction cycle, which is different from the traditional 12T MCU.
**************************************/
void DelayXus(uchar n)		  // 22.1184M
{	 	
	while (n--)
	{
	_nop_();_nop_();_nop_();_nop_();    _nop_();_nop_();_nop_();_nop_();  	_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();    _nop_();_nop_();_nop_();_nop_();  	_nop_();_nop_();
	_nop_();       _nop_();  

// 
// _nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_(); _nop_();_nop_();
// _nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_(); _nop_();_nop_();
// _nop_(); _nop_();
// 


	}
}






/**************************************
Reset DS18B20 and detect whether the device exists

**************************************/
uchar DS18B20_Reset()
{
	CY = 1;
    DQ = 0;                     // Send a low level reset signal
    DelayXus(240);              // Delay at least 480us
    DelayXus(240);              // Delay at least 480us

    DQ = 1;                     // Release the data line
    DelayXus(60);               // Wait 60us
    CY = DQ;                    // Detect presence pulse
    DelayXus(240);              // Wait for the device to release the data line
	DelayXus(180);
	if (CY==0) 		{return 1;}	   // Detect presence pulse
	return 0;
}

/**************************************
Read 1 byte of data from DS18B20
**************************************/
uchar DS18B20_ReadByte()
{
    uchar i;
    uchar dat = 0;

    for (i=0; i<8; i++)             // 8-bit counter
    {	
		dat >>= 1;
        DQ = 0;                     // Start time slice
        DelayXus(1);                // Delay wait 10us
        DQ = 1;                     // Ready to receive
        DelayXus(1);                // Receiving delay 10us
        if (DQ) dat |= 0x80;        // Reading Data
        DelayXus(60);               // Wait for the 60us time slice to end
    }

    return dat;
}

/**************************************
Write 1 byte of data to DS18B20
**************************************/
void DS18B20_WriteByte(uchar dat)
{
    char i;

    for (i=0; i<8; i++)             // 8-bit counter
    {
        DQ = 0;                     // Start time slice
        DelayXus(1);                // Delay wait 10us
        dat >>= 1;                  // Send data
        DQ = CY;
        DelayXus(60);               // Wait for the 60us time slice to end
        DQ = 1;                     // Recovery cable
        DelayXus(1);                // Recovery delay 10us
    }
}
