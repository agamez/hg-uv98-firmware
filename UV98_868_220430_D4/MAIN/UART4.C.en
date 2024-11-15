#include "STC8A8K64D4.H"
#include "UART4.H"
#include "UART2.H"

#include "UARTx.H"
// #include "UART2.H"
#include "PUBLIC_BUF.H"

#include  <string.H> // Keil library

#define FOSC    22118400L		   // Clock 22.1184M
// #define FOSC 11059200L //Clock 22.1184M
#define BAUD    115200			   // GPRS communication rate 115200

#define S4RI  0x01              // S4CON.0
#define S4TI  0x02              // S4CON.1
#define S4RB8 0x04              // S4CON.2
#define S4TB8 0x08              // S4CON.3
#define S4_S0 0x04              // P_SW2.2


// bit UART4_RX_BUSY; //Serial port 4 receives data and waits for processing
bit UART4_TX_BUSY;	   // Serial port 4 sends idle


unsigned char  UART4_BUF_DATA[600] ;		  // Serial port buffer size
unsigned int  UART4_BUF_LENTH;

unsigned char UART4_RX_TIME;	// UARTx Receive Data Timing

/*----------------------------
Send serial data
----------------------------*/
void UART4_SendData(uchar dat)
{
    while (UART4_TX_BUSY);      // Waiting for the previous data to be sent

    UART4_TX_BUSY = 1;
    S4BUF = dat;                // Write data to UART4 data register
}

/*----------------------------
Sending a string
----------------------------*/
void UART4_SendString(char *s)
{
    while (*s)                  // Detect the end of a string
    {
        UART4_SendData(*s++);      // Send current character
    }

    TIME_1S = 0; // The timing is cleared to 0 to prevent continuous output of GPS status


}



/*----------------------------
UART4 Interrupt Service Routine
-----------------------------*/
void UART4() interrupt 18 using 1
{
    if (S4CON & S4TI)
    {
        S4CON &= ~S4TI;    	   // Clear busy flag // Clear S4TI bit
        UART4_TX_BUSY = 0;
        return;
    }

    if (S4CON & S4RI)
    {
        S4CON &= ~S4RI;         // Clear the S4RI bit

// if (UART4_RX_BUSY == 1)
// {
// return; //If the received data is not processed, the new data will be ignored and the data length will be reset to 0
// }

        UART4_RX_TIME = 0;		// Receive timing clear
        UART4_BUF_DATA[UART4_BUF_LENTH++] = S4BUF;
        UART4_BUF_DATA[UART4_BUF_LENTH] = 0x00;			 // Receive 1 byte of data // Add the end symbol

        if (UART4_BUF_LENTH > 550)
        {
            UART4_BUF_LENTH = 0;
            return;
        }

        // The data is greater than 128, the receiving data length, overflow error processing //data length reset = 0
    }
}


void UART4_FUN() // If the radio channel is idle, the serial port receives KISS data.
{
    uint i;

    if ((UART4_BUF_LENTH != 0) && (UART4_RX_TIME > 10))
    {
// UART4_RX_BUSY = 1; //Stop receiving


// for(i=0;i<UART4_BUF_LENTH;i++)   	 {   UART2_SendData(UART4_BUF_DATA[i]);    }  //调试


        for(i = 0; i < UART4_BUF_LENTH; i++)
        {
            UARTx_BUF[i] =  UART4_BUF_DATA[i] ;
            UARTx_BUF[i + 1] = 0;
        }

        UARTx_BUF_LENTH = UART4_BUF_LENTH;
        UART4_BUF_LENTH = 0;
        UART4_RX_TIME = 0;
// UART4_RX_BUSY = 0; //Allow re-receiving
        UART_X_CMD(4);
    }
}


void UART4_Initial()
{
// P_SW2 &amp;= ~S4_S0; //S4_S0=0 (P0.2/RxD4, P0.3/TxD4)
    // P_SW2 |= S4_S0;             //S4_S0=1 (P5.2/RxD4_2, P5.3/TxD4_2)
    S4CON = 0x50;               // 8-bit variable baud rate

    T4L = (65536 - (FOSC / 4 / 9600)); // Set the baud rate reload value
    T4H = (65536 - (FOSC / 4 / 9600)) >> 8;
    T4T3M |= 0x20;              // Timer 4 is in 1T mode
    T4T3M |= 0x80;              // Timer 4 starts counting

    UART4_TX_BUSY = 0;
// UART4_RX_BUSY = 0;
    UART4_BUF_LENTH = 0;	// Data length reset = 0 // Clear serial port 2 buffer data

    IE2 |= 0x10;                 // Enable serial port 4 interrupt
// EA = 1;
}

