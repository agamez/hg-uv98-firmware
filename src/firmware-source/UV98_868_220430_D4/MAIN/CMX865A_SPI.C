#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "CMX865A_SPI.H"

#include "UART1.H"
#include "UART4.H"


#include "DELAY.H"
#include "io.H"
#include "CRC16.H"
#include "DELAY.H"
#include "CMX865A_DECODE.H"
#include  <INTRINS.H> // Keil library



/************* Define the pin port of W5100 CBUS**************/
// sbit CMX865A_SCS =P2^1; // CBUS defines the chip select signal port of CBUS
// sbit CMX865A_SCLK =P2^0; // CBUS defines the CBUS clock signal port
// sbit CMX865A_RST = P0^3; // CBUS
// sbit CMX865A_MOSI =P4^2; // CBUS defines the MOSI port of CBUS
// sbit CMX865A_MISO =P4^1; // CBUS defines the MISO port of CBUS


// float CMX86X_TX_VOL; //1 0-7
// float CMX86X_RX_VOL; //1 0-7


// float CMX86X_TX_VOL; //1 0-7
// float CMX86X_RX_VOL; //1 0-7


#define CMX86X_TX_VOL  	EEPROM_Buffer[0X1D3]	 // 1 0-7
#define CMX86X_RX_VOL  	EEPROM_Buffer[0X1D4]	 // 1 0-7


// #define TX_MODE 0x90   //0X00-0XF0
// #define RX_MODE 0X90   //0X00-0XF0

#define TX_MODE 0x30   // 0X00-0XF0
#define RX_MODE 0X30   // 0X00-0XF0
// 

// Standard Rate Tone 1, 0 Application
// BELL 202	1200	1200\2200	UV 	   0x30
// V.23  	1200	1300\2100	UV 	   0x50
// V.21 H	300	1650\1850	HF		 0x90
// V.21 L	300	980\1180	HF		 0x80

// BELL 103 H	300	2225\2025	HF	 0x70
// BELL 103 L	300	1270\1070	HF	 0x60


// b15 b14  b13  b12
// 1  1  1  1  V.22 bis 2400 bps QAM  High band (Answering modem)
// 1  1  1  0  “  Low band (Calling modem)
// 1  1  0  1  V.22/Bell 212A 1200 bps DPSK  High band (Answering modem)
// 1  1  0  0  “  Low band (Calling modem)
// 1  0  1  1  V.22 600 bps DPSK  High band (Answering modem)
// 1  0  1  0  “  Low band (Calling modem)
// 1  0  0  1  V.21 300 bps FSK  High band (Answering modem)
// 1  0  0  0  “  Low band (Calling modem)
// 0  1  1  1  Bell 103 300 bps FSK  High band (Answering modem)
// 0  1  1  0  “  Low band (Calling modem)
// 0 1 0 1 V.23 FSK 1200 bps
// 0  1  0  0  “  75 bps
// 0  0  1  1  Bell 202 FSK  1200 bps
// 0  0  1  0  “  150 bps
// 0  0  0  1  DTMF / Tones
// 0  0  0  0  Transmitter disabled 1  0  0  0  “  Low band (Calling modem)





// 
// void CMX865A_833US() //Use TIME1 as timer, delay 833us
// {
// CCON = 0; // Clear CF, CR, CCF0, CCF1
// CH = (65536-760*2)/256; //Set the initial value of PCA reference timer.
// CL = (65536-760*2)%256; //;65536-22.1184M oscillation MHZ/12*833US =65536-1535
// CR = 1; //Start PCA timer.
// while (!CF);
// }


void CMX865A_Delay5us()		// @@22.1184MHz
{
// unsigned char i;

    _nop_();
    _nop_();
    _nop_();
    _nop_();

// i = 5;
// while (--i);
}
// ********************************************************************/
// If synchronous data mode is selected, the 8 data bits in the TX data buffer are transmitted serially;
// Data B0 is sent first, so it is reversed. The original data needs to be sent after the high and low bits are swapped.
// In synchronous data mode, data B0 is sent first
void CBUS_WRITE_E3 ( unsigned char dat )	// Sending Data
{
    unsigned char i;

    for ( i = 0; i < 8; i++)
    {
        CMX865A_SCLK = 0; // _nop_(); //send low bit first

        if ( (dat & 0x01) == 0X01 )
        {
            CMX865A_MOSI = 1;
        }
        else
        {
            CMX865A_MOSI = 0;
        }

        dat >>= 1;

// if ( (dat & 0x80)==0X80 )   {CMX865A_MOSI=1; }  else    { CMX865A_MOSI=0; }
// that &lt;&lt;= 1;

        CMX865A_SCLK = 1; // _nop_();
    }

// CMX865A_Delay5us(); //Stop bit delay
}
// In synchronous data mode, data B0 is sent out first
uchar CBUS_READ_E5 ()	 // Read Data
{
    unsigned char i, j;

    j = 0;

    for( i = 0; i < 8; i++ )
    {
        CMX865A_SCLK = 0; // _nop_();
// j <<= 1;				//先接收高位
// if( CMX865A_MISO==1 )  { j |= 0x01;}

        j >>= 1;				// Receive low bit first

        if( CMX865A_MISO == 1 )
        {
            j |= 0x80;
        }

        // _nop_(); // _nop_();
        CMX865A_SCLK = 1; // _nop_();
    }

    return j;
}
// ********************************************************************/


/********************************************************************
Output a byte via the CBUS bus
********************************************************************/
void CBUS_WRITE_REG ( unsigned char dat )
{
    unsigned char i;

    for ( i = 0; i < 8; i++)
    {
        CMX865A_SCLK = 0; // _nop_(); //Send high first

        if ( (dat & 0x80) == 0X80 )
        {
            CMX865A_MOSI = 1;
        }
        else
        {
            CMX865A_MOSI = 0;
        }

        dat <<= 1;
        CMX865A_SCLK = 1; // _nop_();
    }
}

uchar CBUS_READ_REG ()
{
    unsigned char i, j;

    j = 0;

    for( i = 0; i < 8; i++ )
    {
        CMX865A_SCLK = 0; // _nop_(); //Read high bit first

        j <<= 1;

        if( CMX865A_MISO == 1 )
        {
            j |= 0x01;
        }

        CMX865A_SCLK = 1; // _nop_();

    }

    return j;
}

void CMX865A_RESET()
{
    CMX865A_SCS = 0;
    CMX865A_Delay5us();
    CBUS_WRITE_REG(0X01);
    CMX865A_SCS = 1;
    Delay_time_1ms(20);
}



void CMX865A_WRITE_TWO(uchar addr, uchar dat1, uchar dat2 )
{
    CMX865A_SCS = 0; // CMX865A_Delay5us();
    CBUS_WRITE_REG ( addr );
    CBUS_WRITE_REG ( dat1 );
    CBUS_WRITE_REG ( dat2 );
    CMX865A_SCS = 1;
}

uint CMX865A_READ_TWO(uchar addr )
{
    uint dat_msb, dat_lsb;
    CMX865A_SCS = 0; // CMX865A_Delay5us();
    CBUS_WRITE_REG ( addr );
    dat_msb = CBUS_READ_REG ();
    dat_lsb = CBUS_READ_REG ();
    CMX865A_SCS = 1;
    return dat_msb * 256 + dat_lsb;
}




uint CMX865A_READ_E6()  // E6
{
    uint stu;
    stu = CMX865A_READ_TWO(0XE6) ;		   // New data, B10=1 B6=1
// if ((stu&0x0440)==0x0440)	 	{   return 1;	 	}
    return stu;
}


uchar CMX865A_READ_E5()
{
    uchar dat;

    CMX865A_SCS = 0; // CMX865A_Delay5us();
    CBUS_WRITE_REG ( 0XE5 );
    dat = CBUS_READ_E5 ();
    CMX865A_SCS = 1;

    return dat;
}


// Read received data, the status register changes as follows, B5 = 0 B6 = 0 3C 00 Reception completed 3C 40 Reception overflow 3C 60
// float CMX865A_RX_DATA()
// { uchar dat; uint stu;
// 
// 
// stu= CMX865A_READ_TWO(0XE6)  ;
// while((stu&amp;0x0040)!=0x0040) //Receive new data, B6=1
// {
// stu= CMX865A_READ_TWO(0XE6)  ;
// }
// 
// CMX865A_SCS=0; // CMX865A_Delay5us();
// CBUS_WRITE_REG ( 0XE5 );
// dat=CBUS_READ_E5 ();
// CMX865A_SCS=1;
// 
// return that;
// }

// ********************************************************************/
// Write the data to be sent, the status register changes as follows, B12 = 0 B11 = 0 24 60 Ready to send 34 60 Send overflow 3C 60
void CMX865A_TX_DATA( uchar dat )
{
    uint stu;

    CMX865A_SCS = 0; // CMX865A_Delay5us();
    CBUS_WRITE_REG ( 0xE3 );
    CBUS_WRITE_E3 ( dat );
    CMX865A_SCS = 1;

    stu = CMX865A_READ_TWO(0XE6)  ;

    while((stu & 0x1000) != 0x1000)	 // //Wait for sending to complete, B12=1
    {
        stu = CMX865A_READ_TWO(0XE6)  ;
    }
}

// ********************************************************************/
// void  CMX865A_TX_TONE(uchar tone)  //tone=1 1200  0=2200hz
// {
// if (tone==1){CMX865A_WRITE_TWO(0xE1,0x30|(7<<1),0X1B);}else{CMX865A_WRITE_TWO(0xE1,0x30|(7<<1),0X1A);}
// }

void CMX865A_TX_OFF()
{
    CMX865A_WRITE_TWO(0xE1, 0x00 | (CMX86X_TX_VOL << 1), 0X1C);	 // Turn off transmitter
}

// void CMX865A_TONE_TEST()
// {	PTT=1;	 LED_RED=0;  Delay_time_1ms(10);
// 
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(CMX86X_TX_VOL<<1),0X1B);	 //连续发 1	1200HZ
// Delay_time_25ms(40*10);
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(CMX86X_TX_VOL<<1),0X1A);	 //连续发 0	2200hz
// Delay_time_25ms(40*5);
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(CMX86X_TX_VOL<<1),0X1C);
// CMX865A_TX_OFF(); //Turn off the transmitter
// 
// 
// LED_RED=1;		PTT=0;
// }



void TONE1200()
{
    PTT = 1;
    LED_STU = 0;
    Delay_time_1ms(10);
    UART4_SendString("$E01\r\n");

    CMX865A_WRITE_TWO(0xE1, 0x30 | (CMX86X_TX_VOL << 1), 0X1B);	 // Continuous 1 1200HZ

}
void TONE2200()
{
    PTT = 1;
    LED_STU = 0;
    Delay_time_1ms(10);
    UART4_SendString("$E01\r\n");

    CMX865A_WRITE_TWO(0xE1, 0x30 | (CMX86X_TX_VOL << 1), 0X1A);	 // Continuous 0 2200HZ

}

void TONE_OFF()
{
    PTT = 1;
    LED_STU = 0;
    Delay_time_1ms(10);
    CMX865A_WRITE_TWO(0xE1, 0x00 | (CMX86X_TX_VOL << 1), 0X1C);	 // Turn off transmitter
    PTT = 0;
    LED_STU = 1;
    UART4_SendString("$E00\r\n");
}


void CMX865A_TX_ON()
{
// V2.3 FSK 0x50
// bell 202 FSK  0x30
// tx level (0-7) 7=0DB maximum

// TX sets bell 202 FSK mode, tx level=0DB, Tx Synchronous mode, Data bytes from Tx Data Buffer

// Standard Rate Tone 1, 0 Application
// BELL 202	1200	1200\2200	UV 	   0x30
// V.23  	1200	1300\2100	UV 	   0x50
// V.21 H	300	1650\1850	HF		 0x90
// V.21 L	300	980\1180	HF		 0x80

// BELL 103 H	300	2225\2025	HF	 0x70
// BELL 103 L	300	1270\1070	HF	 0x60

// b15 b14  b13  b12
// 1  0  0  1  V.21 300 bps FSK  High band (Answering modem)
// 1  0  0  0  “  Low band (Calling modem)
// 0  1  1  1  Bell 103 300 bps FSK  High band (Answering modem)
// 0  1  1  0  “  Low band (Calling modem)
// 0 1 0 1 V.23 FSK 1200 bps
// 0  0  1  1  Bell 202 FSK  1200 bps
// 0  0  0  0  Transmitter disabled 1  0  0  0  “  Low band (Calling modem)

// CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1C);	 //发送实际数据


    CMX865A_WRITE_TWO(0xE1, TX_MODE | (CMX86X_TX_VOL << 1), 0X1C);	 // Sending actual data

// CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1B);	 //连续发 1	1200HZ
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1A);	 //连续发 0	2200hz
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(1<<1),0X18);	 //连续发交替1 0






// TX sets V.23 FSK mode, tx level=0DB, Tx Synchronous mode, Data bytes from Tx Data Buffer
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1C);	 //发送实际数据
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1B);	 //连续发 1	1200HZ
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1A);	 //连续发 0	2200hz
// CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X18);	 //连续发交替1 0

// CMX865A_WRITE_TWO(0xE1,0x00|(7<<1),0X1C);	 //关闭发送器
}

void CMX865A_PWUP()	   // Power-down mode
{
    CMX_RX_BUSY = 1;
    Delay_time_1ms(10);   // Prevent notifications from being decoded in read state
    CMX865A_WRITE_TWO(0xE0, 0x00, 0X00); // Power-down mode
}


// void read_cmx_set()
// {
// UART1_SendData(CMX86X_TX_VOL+0X30);
// 
// UART1_SendData(CMX86X_RX_VOL+0X30);
// 
// }


void CMX_RX_ON()
{

    CMX_RX_BUSY = 1;
    Delay_time_1ms(10);   // Prevent notifications from being decoded in read state
    CMX865A_WRITE_TWO(0xE2, RX_MODE | (CMX86X_RX_VOL << 1), 0X3E);	// -10.5dB
    CMX_RX_BUSY = 0;

}

// void CMX_RX_OFF()
// {
// CMX865A_WRITE_TWO(0xE2,0x00|(CMX86X_RX_VOL<<1),0X3E);	//-10.5dB
// }

// Initialize the corresponding registers of CMX865A
void CMX865A_Init()
{
// CMX86X_TX_VOL=CMX86X_TX_VOL;	 //1
// CMX86X_RX_VOL=CMX86X_RX_VOL;	 //1

// CMX865A_SCS =CMX865A_SCLK=CMX865A_MOSI=CMX865A_MISO=1; // CBUS defines the chip select signal port of CBUS

    CMX865A_RESET();  // After powering on, reset
    Delay_time_25ms(1);	// Must delay 20ms to wait for the clock and VBIAS to start
    CMX865A_WRITE_TWO(0xE0, 0x11, 0X80); // b8=b7=1 Turn on the power and reset
    Delay_time_25ms(1);	// Must delay 20ms to wait for the clock and VBIAS to start

// TXAN=OUT TXA=ON, turn off closed loop test, 1200MODEM, turn on power, enable IRQ interrupt output
// B7: Setting this bit to 1 resets the internal circuitry of the CMX865, clearing all bits of transmission and
// Status register, receive mode register, programming register and B13-0

// CMX865A_WRITE_TWO(0xE0,0x01,0X00); //Normal state
// CMX865A_WRITE_TWO(0xE0,0x09,0X00); //Start loop test

    CMX865A_WRITE_TWO(0xE0, 0x11, 0X40); // Normal state 11.0592M Filter on


// CMX865A_WRITE_TWO(0xE0,0x15,0X40); //Normal state 11.0592M Close filter

// CMX865A_WRITE_TWO(0xE0,0x09,0X40); //Start loop test
// ********************************************************************/
// CMX865A_TX_ON();
    CMX865A_TX_OFF();
// ********************************************************************/
// RX sets bell 202 FSK mode, Rx level=0DB, Rx Synchronous mode, 8 data bits
// CMX865A_WRITE_TWO(0xE2,0x30|(7<<1),0X3E);	//0dB

    CMX_RX_ON();
// CMX865A_WRITE_TWO(0xE2,0x30|(CMX86X_RX_VOL<<1),0X3E);	//-10.5dB



// RX sets V.23 FSK mode, Rx level=0DB, Rx Synchronous mode, 8 data bits
// CMX865A_WRITE_TWO(0xE2,0x50|(7<<1),0X3E);
// CMX865A_WRITE_TWO(0xE2,0x30|(7<<1),0X0E);	 //关闭接收器
// ********************************************************************/
}
