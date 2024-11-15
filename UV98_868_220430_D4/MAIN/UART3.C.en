#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
// #include "DELAY.H"

#include "UART1.H"
#include "UART2.H"
#include "UART3.H"
// #include "UART4.H"

#include "GPS2.H"
#include "IO.H"


// #include "BEACON.H"
// #include "bmp280.h"
#include "PUBLIC_BUF.H"


#include<string.h>


#define BAUD    9600			   // GPS rate 9600

#define S3RI  0x01              // S3CON.0
#define S3TI  0x02              // S3CON.1
// #define S3RB8 0x04              //S3CON.2
// #define S3TB8 0x08              //S3CON.3
#define S3_S0 0x02              // P_SW2.1

unsigned char   UART3_GPRMC_DATA[128] ;		  // Serial port buffer size
unsigned char   UART3_GPGGA_DATA[128] ;		  // Serial port buffer size

unsigned int 	UART3_BUF_LENTH;	// Length of serial port data sent and received

unsigned char UART3_RX_BUSY;	    // Serial port 3 receives data and waits for processing
bit UART3_TX_BUSY;		// Serial port 3 sends idle



uchar UART3_OUTTIME; // Receive data count, used to determine whether GPS is disconnected


unsigned char UART3_CHECK_FLAG(uchar *p, uchar *p1)			// Comparing Strings
{
// GPS  "$GPRMC"  "$GBRMC"
// BD   "$GPGGA"  "$GBGGA"

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;	  	// Compare $

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		// Compare G

    p++;
    p1++;				   	// Ignore P or B

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		// Compare to RMC or GGA

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		// Compare $

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		// Compare $

    return 1;
}
// 
// *----------------------------
// Send serial data
// ----------------------------*/
// void UART3_SendData(float data)
// {
// while (UART3_TX_BUSY); //Wait for the previous data to be sent
// UART3_TX_BUSY = 1;
// S3BUF = dat; //Write data to UART2 data register
// }
// 
// *----------------------------
// Send string
// ----------------------------*/
// void UART3_SendString(char *s)
// {
// while (*s) //Detect the end of the string
// {
// UART3_SendData(*s++); //Send current character
// }
// }
// 



// Serial port 3 interrupts receiving data
void UART3() interrupt 17 using 1
{
    // Sending completed // Clear S3TI bit // Clear busy flag
    if (S3CON & S3TI)
    {
        S3CON &= ~S3TI;
        UART3_TX_BUSY = 0;
        return;
    }

    if (S3CON & S3RI)
    {
        S3CON &= ~S3RI;         // Clear the S3RI bit

        if (UART3_RX_BUSY == 2)
        {
            return ;   // If the serial port 3 data is not processed, no new data will be received
        }

        // Receiving GPS data
        if (UART3_BUF_LENTH > 120)
        {
            UART3_BUF_LENTH = 0;	   // The received data is too long, please receive again
            return;
        }

        // The received data is too long, please receive it again
        if ( UART3_RX_BUSY == 0  	)
        {
            UART3_GPRMC_DATA[UART3_BUF_LENTH] = S3BUF;
            UART3_BUF_LENTH++;		 // Receive 1 byte of data

            if (S3BUF == 0x0A)	 // Receive 1 row of data //Receive GPS data and retrieve the specified data row in the data frame
            {
                UART3_GPRMC_DATA[UART3_BUF_LENTH] = 0x00;		// Add end symbol


                if (UART3_CHECK_FLAG(UART3_GPRMC_DATA, "$GPRMC") == 1)	 	// Check whether it is GPRMC data
                {
                    UART3_RX_BUSY = 1;
                }	// The serial port receives GPRMC, and the next time it receives GPGGA

                UART3_BUF_LENTH = 0;
                return ; 		// Clear the length mark and prepare for the next reception
            }
        }

        // -------------------------------------------------

        if ( UART3_RX_BUSY == 1  	)
        {
            UART3_GPGGA_DATA[UART3_BUF_LENTH] = S3BUF;
            UART3_BUF_LENTH++;		 // Receive 1 byte of data

            if (S3BUF == 0x0A)	 // Receive 1 row of data //Receive GPS data and retrieve the specified data row in the data frame
            {
                UART3_GPGGA_DATA[UART3_BUF_LENTH] = 0x00;		// Add end symbol

                if (UART3_CHECK_FLAG(UART3_GPGGA_DATA, "$GPGGA") == 1)	// Check whether it is GPRMC data
                {
                    UART3_RX_BUSY = 2;	 // The serial port receives GPRMC again and waits for processing
                }

                UART3_BUF_LENTH = 0;
                return ; 		// Clear the length mark and prepare for the next reception
            }
        }
    }
}

// BeiDou
// $GBRMC,,V,,,,,,,,,,N,V*3B
// $GBVTG,,,,,,,,,N*22
// $GBGGA,,,,,,0,00,99.99,,,,,,*5A
// $GBGSA,A,1,,,,,,99.99,99.99,99.99,4*3A
// $GBGSV,1,1,00,0*77
// $GBGLL,,,,,,W,N*76



void UART3_FUN()
{
    if (UART3_RX_BUSY == 2)	 // Serial port 3 receives GPRMC GPGGA data flag
    {
        if (GPS_EN == 1)
        {
            if(EEPROM_Buffer[0X15] == 1)
            {
                UART2_SendString(UART3_GPRMC_DATA);	 // Debug output GPRMC
                UART2_SendString(UART3_GPGGA_DATA);	 // Debug output GPRMC
// UART1_SendString(UART3_GPRMC_DATA); //debug output GPRMC
// UART1_SendString(UART3_GPGGA_DATA); //debug output GPRMC
            }

            UART3_RX_GPS();	    // Processing GPS data
        }

        UART3_OUTTIME = 0;
        UART3_RX_BUSY = 0;	// //Timeout clears serial port 3 and starts receiving again
        GPS_LINK = 1; // GPS connected
    }

    // If no GPS data is received after 4 seconds, it is determined that the GPS is not connected. // Turn off the GPS light // Clear the timer to 0
    if (UART3_OUTTIME > 5)
    {
        UART3_OUTTIME = 0;
        GPS_LOCKED = 0;
        LED_STU = 1;
        GPS_LINK = 0; // GPS disconnected
    }
}



void UART3_Initial()
{
    P_SW2 &= ~S3_S0;            // S3_S0=0 (P0.0/RxD3, P0.1/TxD3)
// P_SW2 |= S3_S0; //S3_S0=1 (P5.0/RxD3_2, P5.1/TxD3_2)

    S3CON = 0x50;               // 8-bit variable baud rate
// Timer 3 is used as the baud rate generator for serial port 3
    T3L = (65536 - (FOSC / 4 / BAUD)); // Set the baud rate reload value
    T3H = (65536 - (FOSC / 4 / BAUD)) >> 8;
    T4T3M |= 0x02;              // Timer 3 is in 1T mode
    T4T3M |= 0x08;              // Timer 3 starts counting

    UART3_TX_BUSY = 0;
    UART3_RX_BUSY = 0;
    UART3_BUF_LENTH = 0;	// Data length reset = 0 // Clear serial port 2 buffer data

    IE2 |= 0x08;                 // Enable serial port 3 interrupt
// EA = 1;
}

