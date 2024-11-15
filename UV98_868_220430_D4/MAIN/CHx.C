
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "DELAY.H"
#include "CHx.H"

#include "UART4.H"
#include "UART2.H"
// #include "BEACON.H"
#include "CMX865A_CODE.H"
#include "io.H"
#include "BT.H"

#include "PUBLIC_BUF.H"



// Modify the ptt method
// No need to control PTT line
// Launch control by command

// $E01\r\n CH A PTT connected
// $E02\r\n CH B PTT connected
// $E00\r\n PTT disconnected

// #define   PTT_CH_A_ON  UART2_SendString("$E01\r\n");	UART4_SendString("$E01\r\n"); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);
// #define PTT_CH_B_ON UART2_SendString(&quot;$E02\r\n&quot;); UART4_SendString(&quot;$E02\r\n&quot;); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);

#define   PTT_CH_A_ON  	PTT=1; UART4_SendString("$E01\r\n"); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN); 	UART4_SendString("$E00\r\n"); PTT=0;
#define   PTT_CH_B_ON   PTT=1; UART4_SendString("$E02\r\n"); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);	UART4_SendString("$E00\r\n");PTT=0;

// #define PTT_OFF UART2_SendString(&quot;$E00\r\n&quot;); UART4_SendString(&quot;$E00\r\n&quot;);


void BEACON_TX_CHX(uchar mode)	 // Transmit own beacon MODE=0 Relay forwarding MODE=1
{
    uchar CHx;

    Delay_time_25ms(1);// Delay 50ms to give the board time to process


    if (mode == 0)
    {
        CHx = EEPROM_Buffer[0X1D7];
    }
    else
    {
        CHx = EEPROM_Buffer[0X1D6];
    }

    // CH A sends RF beacon
    if (CHx == 0)
    {
        PTT_CH_A_ON	  	BT_OUT(2);	   // Bluetooth output
        return;
    }

    // CH B sends RF beacon
    if (CHx == 1)
    {
        PTT_CH_B_ON	 	BT_OUT(2);	   // Bluetooth output
        return;
    }

    // CH A+B sends RF beacon
    if (CHx == 2)
    {
        PTT_CH_A_ON
        Delay_time_25ms(40);
        PTT_CH_B_ON
        BT_OUT(2);
        return; // Bluetooth output
    }


    // Only Bluetooth sends data, CH A+B does not send RF beacons
    if (CHx == 3)
    {
        BT_OUT(2);
        UART4_SendString("$F01\r\n");
        return; // Bluetooth output only
    }



}

