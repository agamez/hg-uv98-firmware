
#include <intrins.h>

#include "STC8A8K64D4.H"
#include "DELAY.H"
#include "AT24C512.h"
#include "IO.H"
#include "UART2.H"
#include "tostring.H"


sbit AT24_WP = P4 ^ 2;
sbit AT24_SCL = P2 ^ 5;
sbit AT24_SDA = P2 ^ 4;
 

unsigned char  AT24C512_RW_BUF[256];

 

/**************************************
Delay 5US*11.0592=55
**************************************/
void AT24_Delay()
{
    uchar i;

    for(i = 0; i < 10; i++)
    {
        _nop_();
        _nop_();
        _nop_();
    }
}

/**************************************
Start signal
**************************************/

void AT24C512_Start()
{
    AT24_SDA = 1;                    // Pull up the data line
    AT24_Delay();                 // Delay
    AT24_SCL = 1;                    // Pull the clock line high
    AT24_Delay();                 // Delay
    AT24_SDA = 0;                    // Generate falling edge
    AT24_Delay();                 // Delay
    AT24_SCL = 0;                    // Pull the clock line low
    AT24_Delay();                 // Delay
}
/**************************************
Stop signal
**************************************/

void AT24C512_Stop()
{
    AT24_SDA = 0;                    // Pull the data line low
    AT24_Delay();                 // Delay
    AT24_SCL = 1;                    // Pull the clock line high
    AT24_Delay();                 // Delay
    AT24_SDA = 1;                    // Generate rising edge
    AT24_Delay();                 // Delay
}

/**************************************
Sending an answer signal
Ingress parameter: ack (0:ACK 1:NAK)
**************************************/
void AT24CXX_ACK(bit ACK)
{
    AT24_SDA = ACK;                  // Write response signal
    AT24_SCL = 1;                    // Pull the clock line high
    AT24_Delay();                 // Delay
    AT24_SCL = 0;                    // Pull the clock line low
    AT24_Delay();                 // Delay
}


/**************************************
Receive response signal
**************************************/
bit AT24_RecvACK()
{
    AT24_SCL = 1;                    // Pull the clock line high

    AT24_SDA = 1;
    AT24_Delay();                 // Delay
    CY = AT24_SDA;                   // Read response signal

// if (AT24_SDA==1) {AT24_LINK=0;} //No response, then AT24 is not installed
// else{AT24_LINK=1;}

    AT24_SCL = 0;                    // Pull the clock line low
    AT24_Delay();                 // Delay

    return CY;
}

/**************************************
Send a byte of data to the IIC bus
**************************************/
uchar AT24CXX_write_byte(unsigned char value)
{
    unsigned char i;

    for (i = 0; i < 8; i++)     // 8-bit counter
    {
        value <<= 1;            // Shift out the highest bit of data
        AT24_SDA = CY;        // Data transmission port
        AT24_Delay();            // Delay
        AT24_SCL = 1;          // Pull the clock line high
        AT24_Delay();            // Delay
        AT24_SCL = 0;          // Pull the clock line low
        AT24_Delay();             // Delay
    }

    return AT24_RecvACK();		 // 0 = Answered 1 = No answer
}


/**************************************
Receive one byte of data from the IIC bus
**************************************/
uchar AT24CXX_read_byte()
{
    uchar i;
    uchar dat = 0;

    AT24_SDA = 1;            // Enable internal pull-up and prepare to read data.

    for (i = 0; i < 8; i++)    // 8-bit counter
    {
        dat <<= 1;
        AT24_SCL = 1;         // Pull the clock line high
        AT24_Delay();           // Delay
        dat |= AT24_SDA;     // Read Data
        AT24_SCL = 0;         // Pull the clock line low
        AT24_Delay();            // Delay
    }

    return dat;
}


void AT24CXX_WRITE(unsigned int addr, unsigned char wdata)
{
    AT24_WP = 0;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);
    AT24CXX_write_byte(wdata);
    AT24C512_Stop();
    AT24_WP = 1;
    Delay_time_1ms(5);	// Writing data requires a delay, otherwise the system will crash. Writing does not require a delay
}



unsigned char AT24CXX_READ(unsigned int addr)
{
    unsigned char i;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);

    AT24C512_Start();
    AT24CXX_write_byte(0xa1);
    i = AT24CXX_read_byte();
    AT24C512_Stop();

    return i;
}
// 128-byte page write
unsigned char AT24CXX_WRITE_N(unsigned int addr, unsigned char  *buf, uint len)
{
    uint i;

    AT24_WP = 0;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);

    for(i = 0; i < len; i++)
    {
        AT24CXX_write_byte(*buf);
        buf++;
    }

    AT24C512_Stop();
    AT24_WP = 1;

    Delay_time_1ms(5);	// //Writing data must be delayed

    return i;
}

// 128-byte page read
void  AT24CXX_READ_N(unsigned int addr, unsigned char  *buf, uint len)
{
    uint i;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);

    AT24C512_Start();
    AT24CXX_write_byte(0xa1);

    for(i = 0; i < len; i++)
    {
        *buf = AT24CXX_read_byte();
        buf++;
        AT24CXX_ACK(i == (len - 1));
    }

    AT24C512_Stop();

}




// 
// void AT2C512_TEST()
// {
// uint i; volatile temp[20];
// 
// AT24CXX_WRITE(0x0000,0x0E); //Write a byte to the specified address
// UART1_SendData(AT24CXX_READ(0x0000)); //Read a byte from the specified address
// for(i=0;i<256;i++)	 {UART1_SendData(AT24CXX_READ(0x0000+i));  }
// AT24CXX_WRITE_N(0x0000,AT24C512_W_BUF,128); //Write 128 bytes to the specified address
// AT24CXX_READ_N(0x0000,AT24C512_R_BUF,128); //Read 128 bytes from the specified address
// for(i=0;i<128;i++)	 {  UART1_SendData(AT24C512_R_BUF[i]); }
// 
// AT24CXX_READ_N(0,AT24C512_RW_BUF,20);
// DEBUG_KISS(AT24C512_RW_BUF,20);
// 
// for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(0+16+i); temp[i+1]=0; }  //呼号
// DEBUG_KISS(temp,9);
// 
// 
// }
// 

void AT2C512_CLEAN()   // Clear all records 0-511, clear index
{
    uint i, n;
    UART2_SendString("Wait...\r\n");

    for(i = 0; i < 128; i++)
    {
        AT24C512_RW_BUF[i] = 0xff;
    }

    for(n = 0; n < 512; n++)
    {
        AT24CXX_WRITE_N(n * 128, AT24C512_RW_BUF, 128);		   // Write 128 bytes to the specified address
    }

    UART2_SendString("DATA Clean OK\r\n");
}

// 0x0000-0x6400 (0-25600), each beacon occupies 256 bytes, a total of 100 records, index 0-99 is stored at 0x8000
// 0x6500-0x7180 , list storage area, one index per 16 bytes, a total of 100 indexes
// BH4TDV-XX 99 Beacon storage address index 0-99
// 0xFE00 Last 512 storage setting data
