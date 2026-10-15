
/**************** BMP085 sensor***************/

#include "STC15Fxxxx.H"
#include  <INTRINS.H> // Keil library
#define   uchar unsigned char	   // 0-255 single byte, use uchar instead of char
#define   uint unsigned int		   // 0-65536 double bytes use uint instead of int

#include "tostring.H"

#include "BMP085.H"
#include "bmp280.h"


#define	  BMP085_SlaveAddress   0xee	  // Defines the slave address of the device in the IIC bus
#define   OSS 0	// Oversampling Setting (note: code is not set up to use other OSS values)


sbit	SCL=P4^2;      // IIC clock pin definition
sbit	SDA=P4^3;      // IIC data pin definition
 	
// unsigned char bmp085_ge,bmp085_shi,bmp085_bai,bmp085_qian,bmp085_wan,bmp085_shiwan; //display variables
// unsigned char  bmp085_tp_ge,bmp085_tp_shi,bmp085_tp_bai;

// unsigned char QY_SHIWAN,QY_WAN,QY_QIAN,QY_BAI,QY_SHI;

uchar QY[10];       
 
unsigned int dis_data;                              // variable

short ac1,ac2,ac3;
unsigned short ac4,ac5,ac6;
short b1,b2; 
short mb,mc,md;

void delay(unsigned int k);
void conversion(long temp_data);

void  Single_Write(uchar SlaveAddress,uchar REG_Address,uchar REG_data);   // Single write data
uchar Single_Read(uchar REG_Address);                                      // Single read internal register data
void  Multiple_Read(uchar,uchar);                                          // Continuously read internal register data
// ------------------------------------
void Delay5us();
void Delay5ms();
void BMP085_Start();
void BMP085_Stop();
void BMP085_SendACK(bit ack);
bit  BMP085_RecvACK();
void BMP085_SendByte(uchar ndat);
uchar BMP085_RecvByte();
void BMP085_ReadPage();
void BMP085_WritePage();



uchar bmp085Convert();
void Init_BMP085();
// -----------------------------------

bit BMP085_LINK;	// 0=Not installed 1=Installed


// *********************************************************
// void conversion(long temp_data)
// {
// 
// bmp085_shiwan=temp_data/100000+0x30;
// temp_data=temp_data%100000; //Remainder operation
// 
// bmp085_wan=temp_data/10000+0x30 ;
// temp_data=temp_data%10000; //Remainder operation
// 
// bmp085_qian=temp_data/1000+0x30 ;
// temp_data=temp_data%1000; // remainder operation
// 
// bmp085_bai=temp_data/100+0x30   ;
// temp_data=temp_data%100; // remainder operation
// 
// bmp085_shi=temp_data/10+0x30    ;
// temp_data=temp_data%10; // remainder operation
// 
// bmp085_ge=temp_data+0x30;
// }
// 
// *******************************/
		
 
/**************************************
Delay 5 microseconds (STC90C52RC@12M)
Different working environments require adjustment of this function. Note that it needs to be modified when the clock is too fast.
When using a 1T MCU, please adjust this delay function

1TCPU x=5US*11.0592=55
1TCPU x=5US*22.1184=110
1TCPU x=5US*33.1776=165
**************************************/
// void Delay5us()
// {
// flying i ,n;
// 
// for(i=0;i<55;i++)	{n++;}		//11.0592M
// 
// for(i=0;i<110;i++)	{n++;}		//22.1184M
// 
// for(i=0;i<165;i++)	{n++;}		//33.1776M
// }
void Delay5us()		// @22.1184MHz
{
	unsigned char i;

	_nop_();
	i = 25;
	while (--i);
}

/**************************************
Delay 5 milliseconds (STC90C52RC@12M)
Different working environments require adjustment of this function
When using a 1T MCU, please adjust this delay function
**************************************/
// ;Delay 5ms ;65536-11.0592M oscillation MHZ/12*5000US =65536-4608
void Delay5ms()
{	/*	*/
// TR1=0;
// TF1=0;
// TH1=(65536-4608)/256;
// TL1=(65536-4608)%256;
// TR1=1;
// while (!TF1);
	

// CCON = 0; // Clear CF, CR, CCF0, CCF1
// CH = (65536-4608)/256; //Set the initial value of PCA reference timer.
// CL = (65536-4608)%256; //;65536-11.0592M oscillation MHZ/12*5000US =65536-4608
// CR = 1; //Start PCA timer.
// while (!CF);


	CCON = 0;					// Clear CF, CR, CCF0, CCF1
	CH = (65536-9216)/256;		// The PCA reference timer is initialized.
	CL = (65536-9216)%256;		// ;65536-22.1184M oscillation MHZ/12*5000US =65536-9216
	CR = 1;						// Start the PCA timer.
	while (!CF);

// CCON = 0; // Clear CF, CR, CCF0, CCF1
// CH = (65536-13824)/256; //Set the initial value of PCA reference timer.
// CL = (65536-13824) %256; //65536-33.1776M oscillation MHZ/12*5000US =65536-13824
// CR = 1; //Start PCA timer.
// while (!CF);


}



/**************************************
Start signal
**************************************/
void BMP085_Start()
{
    SDA = 1;                    // Pull up the data line
    SCL = 1;                    // Pull the clock line high
    Delay5us();                 // Delay
    SDA = 0;                    // Generate falling edge
    Delay5us();                 // Delay
    SCL = 0;                    // Pull the clock line low
}

/**************************************
Stop signal
**************************************/
void BMP085_Stop()
{
    SDA = 0;                    // Pull the data line low
    SCL = 1;                    // Pull the clock line high
    Delay5us();                 // Delay
    SDA = 1;                    // Generate rising edge
    Delay5us();                 // Delay
   
}

/**************************************
Sending an answer signal
Ingress parameter: ack (0:ACK 1:NAK)
**************************************/
void BMP085_SendACK(bit ack)
{
    SDA = ack;                  // Write response signal
    SCL = 1;                    // Pull the clock line high
    Delay5us();                 // Delay
    SCL = 0;                    // Pull the clock line low
    Delay5us();                 // Delay
}

/**************************************
Receive response signal
**************************************/
bit BMP085_RecvACK()
{
    SCL = 1;                    // Pull the clock line high
    Delay5us();                 // Delay
    CY = SDA;                   // Read response signal
		
	if (SDA==1)	{BMP085_LINK=0;}  // If there is no response, the BMP085 is not installed.
	else{BMP085_LINK=1;}

    SCL = 0;                    // Pull the clock line low
    Delay5us();                 // Delay

    return CY;
}

/**************************************
Send a byte of data to the IIC bus
**************************************/
void BMP085_SendByte(uchar dat)
{
    uchar i;

    for (i=0; i<8; i++)         // 8-bit counter
    {
        dat <<= 1;              // Shift out the highest bit of data
        SDA = CY;               // Data transmission port
        SCL = 1;                // Pull the clock line high
        Delay5us();             // Delay
        SCL = 0;                // Pull the clock line low
        Delay5us();             // Delay
    }
    BMP085_RecvACK();
}

/**************************************
Receive one byte of data from the IIC bus
**************************************/
uchar BMP085_RecvByte()
{
    uchar i;
    uchar dat = 0;

    SDA = 1;                    // Enable internal pull-up and prepare to read data.
    for (i=0; i<8; i++)         // 8-bit counter
    {
        dat <<= 1;
        SCL = 1;                // Pull the clock line high
        Delay5us();             // Delay
        dat |= SDA;             // Read Data
        SCL = 0;                // Pull the clock line low
        Delay5us();             // Delay
    }
    return dat;
}
/*
// Single byte write BMP085 internal data **********************************

void Single_Write(uchar SlaveAddress,uchar REG_Address,uchar REG_data)
{
    BMP085_Start(); // Start signal
    BMP085_SendByte(SlaveAddress); // Send device address + write signal
    BMP085_SendByte(REG_Address); // Internal register address
    BMP085_SendByte(REG_data); // Internal register data
    BMP085_Stop(); // Send stop signal
}
*/
/*
// Single byte read BMP085 internal data********************************
uchar Single_Read(uchar REG_Address)
{ uchar REG_data;
    BMP085_Start(); // Start signal
    BMP085_SendByte(BMP085_SlaveAddress); // Send device address + write signal
    BMP085_SendByte(REG_Address); // Send storage unit address
    BMP085_Start(); // Start signal
    BMP085_SendByte(BMP085_SlaveAddress+1); // Send device address + read signal
    REG_data=BMP085_RecvByte(); // Read register data
	BMP085_SendACK(1);   
	BMP085_Stop(); // Stop signal
    return REG_data; 
}
*/
// *********************************************************
// Read the internal data of BMP085, two consecutive
// *********************************************************
short Multiple_read(uchar ST_Address)
{   
	fly msb, lsb;
	short _data;
    BMP085_Start(); // Start signal
    BMP085_SendByte(BMP085_SlaveAddress); // Send device address + write signal
    BMP085_SendByte(ST_Address); // Send storage unit address
    BMP085_Start(); // Start signal
    BMP085_SendByte(BMP085_SlaveAddress+1); // Send device address + read signal

    msb = BMP085_RecvByte(); // BUF[0] storage
    BMP085_SendACK(0); // Response ACK
    lsb = BMP085_RecvByte();     
	BMP085_SendACK(1); // The last data needs to be returned NOACK

    BMP085_Stop(); // Stop signal
    Delay5ms();
    _data = msb << 8;
	_data |= lsb;	
	return _data;
}
// ********************************************************************
long bmp085ReadTemp(void)
{

    BMP085_Start(); // Start signal
    BMP085_SendByte(BMP085_SlaveAddress); // Send device address + write signal
    BMP085_SendByte(0xF4);	          // write register address
    BMP085_SendByte(0x2E);       	// write register data for temp
    BMP085_Stop(); // Send stop signal
	// delay(10);	// max time is 4.5ms
	Delay5ms();
	return (long) Multiple_read(0xF6);
}
// *************************************************************
long bmp085ReadPressure(void)
{
	long pressure = 0;

    BMP085_Start(); // Start signal
    BMP085_SendByte(BMP085_SlaveAddress); // Send device address + write signal
    BMP085_SendByte(0xF4);	          // write register address
    BMP085_SendByte(0x34);       	  // write register data for pressure
    BMP085_Stop(); // Send stop signal
	// delay(10);    	                  // max time is 4.5ms
	Delay5ms();
	pressure = Multiple_read(0xF6);
	pressure &= 0x0000FFFF;
	
	return pressure;	
	// return (long) bmp085ReadShort(0xF6);
}

// **************************************************************

// Initialize BMP085, please refer to pdf and modify as needed**************
void Init_BMP085()
{
	ac1 = Multiple_read(0xAA);
	ac2 = Multiple_read(0xAC);
	ac3 = Multiple_read(0xAE);
	ac4 = Multiple_read(0xB0);
	ac5 = Multiple_read(0xB2);
	ac6 = Multiple_read(0xB4);
	b1 =  Multiple_read(0xB6);
	b2 =  Multiple_read(0xB8);
	mb =  Multiple_read(0xBA);
	mc =  Multiple_read(0xBC);
	md =  Multiple_read(0xBE);
}
// ***********************************************************************
fly bmp085Convert()
{

	flying k;

	long ut;
	long up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;
	long  temperature;
	long  pressure;




 	Heat_BMP085();

	
	
	ut = bmp085ReadTemp(); // Read temperature
// ut = bmp085ReadTemp(); // Read temperature
 	
	up = bmp085ReadPressure(); // Read pressure
// up = bmp085ReadPressure(); // Read pressure


	if( BMP085_LINK==0) // BMP180 not detected
	{
	
	
	
	 return 0;
	}	 

	x1 = ((long)ut - ac6) * ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	temperature = (b5 + 8) >> 4;
	
	// *************
	
// conversion(temperature);
	
	
	
// bmp085_tp_bai=bmp085_bai;
// bmp085_tp_shi=bmp085_shi;
// bmp085_tp_ge=bmp085_ge;
	
	
// HUASHI_BMP085=((bmp085_tp_bai-0x30)*10+(bmp085_tp_shi-0x30))*9/5+32; //Convert to Fahrenheit temperature
// HUASHI_BMP085_BAI=HUASHI_BMP085/100%10+0x30;
// HUASHI_BMP085_SHI=HUASHI_BMP085/10%10+0x30;
// HUASHI_BMP085_GE=HUASHI_BMP085%10+0x30;

/*	*/ 

// 
// SendData(&#39;T&#39;); //Temperature display
// SendData(&#39;:&#39;);
// 
// SendData(bmp085_tp_bai);
// SendData(bmp085_tp_shi);
// SendData(&#39;.&#39;);
// SendData(bmp085_tp_ge);
// 
// SendData(&#39;C&#39;); //Temperature unit
// 
// SendData(0x0d);
// SendData(0x0a);

 

	 
     // *************
	
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((long)ac1 * 4 + x3) + 2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);
	if( b7 < 0x80000000)
	     p = (b7 * 2) / b4 ;
           else  
		    p = (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	pressure = p + ((x1 + x2 + 3791) >> 4);
	
   BMPXX_QY=(uint) (pressure/10);

  tostring(BMPXX_QY);
// conversion(pressure);

// QY_SHIWAN=bmp085_shiwan;
// QY_WAN= bmp085_wan;
// QY_QIAN=bmp085_qian;
// QY_BAI=bmp085_bai;
// QY_SHI=bmp085_shi;
	
	k=0;
	QY[k]=wan;	k++;
	QY[k]=qian;	k++;
	QY[k]=bai;	k++;
	QY[k]=shi;	k++;
	QY[k]='.';	k++;
	QY[k]=ge;	k++;
// QY[k]='p';	k++;
// QY[k]='a';	k++;
	QY[k]=0x00;


// 
// SendData(&#39;P&#39;); //Display pressure
// SendData(&#39;:&#39;);
// 
// SendData(bmp085_shiwan);
// SendData(bmp085_wan);
// SendData(bmp085_qian);
// SendData(bmp085_bai);
// SendData(bmp085_shi);
// SendData(bmp085_ge);
// SendData(0x0d);
// SendData(0x0a);


	 
// Standard weather beacon format, full data
// 217/000g003t058r010P009p040h62b10236 36 bytes long
// 0-2 3 4-6  	7-10  	11-14  15-18	  19-22		  23-26     27-29  		30-35
// 217	/ 000	g003 	t058   r010       P009        p040      h62  		b10236
// Items: wind direction, wind speed, gusts, temperature, rain/last hour, rain/last 24 hours, rain/since midnight, humidity, air pressure
// Length: 3 ,1 , 3 , 4 , 4 , 4 , 4 , 3 , 6
// Unit: degree, imperial, 5 minutes, Fahrenheit, hour, 24 hours, zero o&#39;clock, percentage, Pa*10


// unsigned char xdata weather_is_beacen[]={"217/000g003t058r010P009p040h62b10236 WX_beacen \r\n"};
	return 1;
}


