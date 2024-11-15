#include "STC8A8K64D4.H"
#include "PUBLIC_BUF.H"


#include "UART1.H"
#include "UARTx.H"
#include "UART2.H"

#include "tostring.H"


#define S2RI  0x01              // S2CON.0
#define S2TI  0x02              // S2CON.1
// #define S2RB8 0x04              //S2CON.2
// #define S2TB8 0x08              //S2CON.3
// #define S2_S0 0x01              //P_SW2.0

unsigned char   UART2_BUF_DATA[600] ;		  // Serial port buffer size
unsigned int 	UART2_BUF_LENTH;	// Length of serial port data sent and received

// bit UART2_RX_BUSY; //Serial port 2 receives data and waits for processing
bit UART2_TX_BUSY;		// Serial port 2 sends idle


unsigned char UART2_RX_TIME;	// UARTx Receive Data Timing

/*----------------------------
Send serial data
----------------------------*/
void UART2_SendData(uchar dat)
{
    while (UART2_TX_BUSY);               // Waiting for the previous data to be sent

    UART2_TX_BUSY = 1;
    S2BUF = dat; // Combined login data; //Write data to UART2 data register
}

/*----------------------------
Sending a string
----------------------------*/
void UART2_SendString(char *s)
{
    while (*s)                  // Detect the end of a string
    {
        UART2_SendData(*s++);         // Send current character
    }
}

/*----------------------------
UART2 Interrupt Service Routine
-----------------------------*/
void UART2() interrupt 8 using 1
{
    if (S2CON & S2TI)
    {
        S2CON &= ~S2TI;    // Clear S2TI bit // clear busy flag
        UART2_TX_BUSY = 0;
        return;
    }

    if (S2CON & S2RI)
    {
        S2CON &= ~S2RI;         // Clear the S2RI bit

// if (UART2_RX_BUSY == 1)
// {
// return; //If the received data is not processed, the new data will be ignored and the data length will be reset to 0
// }

        UART2_RX_TIME = 0;		// Receive timing clear
        UART2_BUF_DATA[UART2_BUF_LENTH++] = S2BUF;
        UART2_BUF_DATA[UART2_BUF_LENTH] = 0x00;	 // Receive 1 byte of data // Add the end symbol

        if (UART2_BUF_LENTH > 550)
        {
            UART2_BUF_LENTH = 0;
            return;
        }

        // The data is greater than 128, the receiving data length, overflow error processing //data length reset = 0
    }
}






void UART2_DEBUG(unsigned int n)	   // Debug output integer
{
    tostring(n);

    UART2_SendString(str_txt);
    UART2_SendString("\r\n");

}



void UART2_FUN() // If the radio channel is idle, the serial port receives KISS data.
{
    uint i;

    if ((UART2_BUF_LENTH != 0) && (UART2_RX_TIME > 10))
    {

        // for(i=0;i<UART2_BUF_LENTH;i++)   	 {   UART2_SendData(UART2_BUF_DATA[i]);    }  //调试
        for(i = 0; i < UART2_BUF_LENTH; i++)
        {
            UARTx_BUF[i] =  UART2_BUF_DATA[i] ;
            UARTx_BUF[i + 1] = 0;
        }

        UARTx_BUF_LENTH = UART2_BUF_LENTH;

        UART2_BUF_LENTH = 0;
        UART2_RX_TIME = 0;

        UART_X_CMD(2);
    }
}




// 
// void UART2_select_p1011() //Bluetooth
// { P_SW2 &amp;= ~S2_S0; //S2_S0=0 (P1.0/RxD2, P1.1/TxD2)
// }
// 
// void UART2_select_p4647()	  //LCD
// {	P_SW2 |= S2_S0;      //S2_S0=1 (P4.6/RxD2_2, P4.7/TxD2_2)
// }


void UART2_Initial()
{
// P_SW2 &amp;= ~S2_S0; //S2_S0=0 (P1.0/RxD2, P1.1/TxD2)
// //  P_SW2 |= S2_S0;             //S2_S0=1 (P4.6/RxD2_2, P4.7/TxD2_2)

// UART2_select_p4647();

    S2CON = 0x50;               // 8-bit variable baud rate
// The baud rate does not need to be set, it is the same as serial port 1
// T2L = (65536 - (FOSC/4/BAUD)); //Set baud rate reload value
// T2H = (65536 - (FOSC/4/BAUD))&gt;&gt;8;
// AUXR = 0x14; //T2 is 1T mode, and start timer 2

    UART2_TX_BUSY = 0;
    UART2_BUF_LENTH = 0;	// Data length reset = 0 // Clear serial port 2 buffer data

    IE2 |= 0x01;                 // Enable serial port 2 interrupt
// EA = 1;

}
