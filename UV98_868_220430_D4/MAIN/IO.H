
#ifndef _IO_H_
#define _IO_H_


/********************************************************/


sbit LED_STU =P2^6;


sbit FX604_TX=P1^5;

sbit PTT 	 =P2^3;
sbit GPS_EN	 =P2^7;	 // GPRS switch


sbit BL_PWR  =P3^6;	  // Bluetooth Power
sbit BL_RST  =P3^5;	  // Bluetooth Reset
sbit BL_STU  =P3^2;	  // Bluetooth connection status

 
sbit	BMP280_SCL=P4^1;      // IIC clock pin definition
sbit	BMP280_SDA=P2^2;      // IIC data pin definition

 
sbit CMX865A_SCS	=P4^4;	// CBUS defines the chip select signal port of CBUS
sbit CMX865A_SCLK	=P1^3;	// CBUS defines the CBUS clock signal port
// sbit CMX865A_RST = P0^3; // CBUS
sbit CMX865A_MOSI	=P1^4;	// CBUS defines the MOSI port of CBUS
sbit CMX865A_MISO	=P1^2;	// CBUS defines the MISO port of CBUS
sbit CMX865A_PW	=P0^7;	// PW



#endif