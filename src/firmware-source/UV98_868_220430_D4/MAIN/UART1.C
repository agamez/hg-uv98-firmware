#include "STC8A8K64D4.H"
#include "UART1.H"
#include "UARTx.H"
#include "UART2.H"

#include "tostring.H"


#define BAUD    9600			   // Serial port/Bluetooth communication rate 9600



#define S1_S0 0x40              // P_SW1.6
#define S1_S1 0x80              // P_SW1.7


// #define ISP_ON IAP_CONTR= 0x60; //ISP download
// #define RESET IAP_CONTR= 0x20; //System reset

unsigned char UART1_BUF_DATA[600];	// Receive data buffer
unsigned int  UART1_BUF_LENTH;	// Length of serial port data sent and received

bit UART1_TX_BUSY;		// Serial port 3 sends idle

unsigned char UART1_RX_TIME;	// UARTx Receive Data Timing

/*----------------------------
Send serial data
----------------------------*/
void UART1_SendData(uchar dat)
{
    while (UART1_TX_BUSY);    // Waiting for the previous data to be sent

    UART1_TX_BUSY = 1;
    SBUF = dat;                // Write data to UART2 data register
}

/*----------------------------
Sending a string
----------------------------*/
void UART1_SendString(char *s)
{
    while (*s)                  // Detect the end of a string
    {
        UART1_SendData(*s++);         // Send current character
    }
}




void UART1_DEBUG(unsigned int n)	   // Debug output integer
{
    tostring(n);
 
    UART1_SendString(str_txt);
    UART1_SendString("\r\n");
}



// *****************************************
// UART1 serial port interrupt
// *****************************************
void UART1() interrupt 4 using 1
{
    if(TI)
    {
        TI = 0;	   // Note the crash caused by TI interrupt //Serial port sends a byte flag //Clear busy flag
        UART1_TX_BUSY = 0;
        return;
    }

    if(RI)
    {
        RI = 0;



        UART1_RX_TIME = 0;		// Receive timing clear
        UART1_BUF_DATA[UART1_BUF_LENTH++] = SBUF;
        UART1_BUF_DATA[UART1_BUF_LENTH] = 0x00;		// Add end symbol //Receive 1 byte of data

        if (UART1_BUF_LENTH > 590)
        {
            UART1_BUF_LENTH = 0;
            return;
        }

        // The data is greater than 128, the receiving data length, overflow error processing //data length reset = 0
    }
}





void UART1_FUN()	// Process the data received by the serial port and parse the data
{
    uint i;

    if ((UART1_BUF_LENTH != 0) && (UART1_RX_TIME > 5))
    {

// for(i=0;i<UART1_BUF_LENTH;i++)   	 {   UART2_SendData(UART1_BUF_DATA[i]);    }  //调试
        for(i = 0; i < UART1_BUF_LENTH; i++)
        {
            UARTx_BUF[i] =  UART1_BUF_DATA[i] ;
            UARTx_BUF[i + 1] = 0;
        }

        UARTx_BUF_LENTH = UART1_BUF_LENTH;
        UART1_BUF_LENTH = 0;
        UART1_RX_TIME = 0;

        UART_X_CMD(1);
    }

}


// 
// void UART1_select_p3031()
// {
// ACC = P_SW1;
// ACC &amp;= ~(S1_S0 | S1_S1); //S1_S0=0 S1_S1=0
// P_SW1 = ACC; //(P3.0/RxD, P3.1/TxD)
// }
// 
// void UART1_select_p3637()
// {
// ACC = P_SW1;
// ACC &amp;= ~(S1_S0 | S1_S1); //S1_S0=1 S1_S1=0
// ACC |= S1_S0;               //(P3.6/RxD_2, P3.7/TxD_2)
// P_SW1 = ACC;
// }

void UART1_Initial()
{
// ACC = P_SW1;
// ACC &amp;= ~(S1_S0 | S1_S1); //S1_S0=0 S1_S1=1
// ACC |= S1_S1; //(P1.6/RxD_3, P1.7/TxD_3)
// P_SW1 = ACC;

    SCON = 0x50;                // 8-bit variable baud rate
    T2L = (65536 - (FOSC / 4 / BAUD)); // Set the baud rate reload value
    T2H = (65536 - (FOSC / 4 / BAUD)) >> 8;
    AUXR = 0x14;                // T2 is 1T mode, and start timer 2
    AUXR |= 0x01;               // Select Timer 2 as the baud rate generator for serial port 1

    UART1_TX_BUSY = 0;
    UART1_BUF_LENTH = 0;	// Data length reset = 0 // Clear serial port 2 buffer data

    ES = 1;                     // Enable serial port 1 interrupt
}




