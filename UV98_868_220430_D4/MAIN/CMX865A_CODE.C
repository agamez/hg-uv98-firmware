#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

// #include "UART1.H"
#include "DELAY.H"
#include "io.H"
#include "BEACON.H"
#include "BT.H"


#include "CRC16.H"
#include "CMX865A_CODE.H"
#include "CMX865A_SPI.H"

// #include "tostring.H"

#include "UART2.H"
#include "UART4.H"

#include "PUBLIC_BUF.H"


#include  <INTRINS.H> // Keil library


// ************* Send Register
bit HDLC_TX_BIT;
uchar HDLC_TX_COUNT;			// Reset the count of 5 consecutive 1s
uchar HDLC_TX_BUF[300];
uint HDLC_TX_IDX;	  // Current bit index

// ************* Receive Register
// float HDLC_RX_BUF[300];

// uchar HDLC_TX_BIT_OLD; //Last bit status
// uchar BIT_1_COUNT; //Count of consecutive 1s
// uchar END_7E; //7E end mark
// uchar HDLC_TEMP; //temporary decoding data
// uint TOTAL_IDX; //Total number of digits

// ************* Output KISS data register

// float APRS_KISS_BUF[300];


// Each frame has a flag code 01111110 before and after, which is used to indicate the start and end of the frame and for frame synchronization.
// The flag code is not allowed to appear inside the frame to avoid ambiguity.
// In order to ensure the uniqueness of the identification code while taking into account the transparency of the data within the frame, the &quot;0-bit insertion method&quot; can be used to solve the problem.
// This method monitors all fields except the flag code at the sending end.
// When five consecutive &quot;1&quot;s are found, a &quot;0&quot; is inserted after them, and then the subsequent bit stream is sent.
// At the receiving end, all fields except the start flag code are also monitored.
// When five &quot;1&quot;s are found in succession, if the next bit is &quot;0&quot;, it will be automatically deleted to restore the original bit stream;
// If six consecutive &quot;1&quot;s are found, it may be that the inserted &quot;0&quot; has an error and turned into a &quot;1&quot;, or it may be that the end flag code of the frame has been received.
// The latter two cases can be further distinguished by the frame check sequence in the frame.
// The principle of the &quot;0-bit insertion method&quot; is simple and very suitable for hardware implementation.


// uchar HDLC_SendByte (uchar inbyte,uchar aprs_flag) //aprs_flag 1 = front and back flag 7E, 0 = data 7E
// {
// flying k, bt; float HDLC_TEMP;
// 
// for (k=0;k<8;k++)
// {                                                         //do the following for each of the 8 bits in the byte
// bt = inbyte &amp; 0x01; // take one bit
// if (bt == 0)
// {
// HDLC_TX_BIT=!HDLC_TX_BIT; //Invert the output
// HDLC_TX_COUNT=0; //Reset the count of 5 consecutive 1s
// }
// else
// {			HDLC_TX_COUNT++; 	 	}
// 
// HDLC_TEMP&lt;&lt;=1; HDLC_TEMP|=HDLC_TX_BIT;
// 
// if ((HDLC_TX_COUNT == 5)&amp;&amp;(aprs_flag == 0) ) //5 consecutive &quot;1&quot;s, and not the aprs_flag before and after identification code, then insert 1 &quot;0&quot;
// {
// HDLC_TX_BIT=!HDLC_TX_BIT; //5 consecutive &quot;1&quot;, the output is inverted, that is, insert a 0
// HDLC_TEMP&lt;&lt;=1; HDLC_TEMP|=HDLC_TX_BIT;
// HDLC_TX_COUNT=0; //Reset the count of 5 consecutive 1s
// }
// 
// inbyte = inbyte&gt;&gt;1; //Get the next bit
// }
// 
// CMX865A_TX_DATA(HDLC_TEMP);	   FX604_TX=!FX604_TX;
// return 	HDLC_TEMP;
// }

// C0 00
// 82 A0 6A 62 90 62 60
// 9C 9E 86 82 98 98 73
// 03 F0
// 21 //21F0 20
// 33 31 33 34 2E 38 30 4E
// 2F
// 31 32 30 32 30 2E 39 39 45
// 3E //3E45 1E
// 36 6F
// C0

void HDLC_WIRITE_TX_BIT(uchar dat)	   // Write one bit
{
    uint byte_idx;
    uchar bit_idx;

    HDLC_TX_IDX++;
    byte_idx = HDLC_TX_IDX / 8;
    bit_idx = HDLC_TX_IDX % 8;

    if (dat == 1)
    {
        HDLC_TX_BUF[byte_idx] |= (0x80 >> bit_idx);
    }
}

void HDLC_SendByte2 (uchar inbyte, uchar aprs_flag) // aprs_flag 1 = pre and post flag 7E, 0 = data 7E or other data
{
    uchar k, bt;

    for (k = 0; k < 8; k++)
    {
        // do the following for each of the 8 bits in the byte
        bt = inbyte & 0x01;    // Take one

        if (bt == 0)
        {
            HDLC_TX_BIT = !HDLC_TX_BIT;  	// Output inversion
            HDLC_TX_COUNT = 0;			// Reset the count of 5 consecutive 1s
        }
        else
        {
            HDLC_TX_COUNT++;
        }


        HDLC_WIRITE_TX_BIT(HDLC_TX_BIT);

        if ((HDLC_TX_COUNT == 5) && (aprs_flag == 0) )		 // If there are five consecutive &quot;1&quot;s and they are not the identification codes before or after aprs_flag, insert a &quot;0&quot;
        {
            HDLC_TX_COUNT = 0;	 	 // Reset the count of 5 consecutive 1s
            HDLC_TX_BIT = !HDLC_TX_BIT;  // If there are five consecutive &quot;1s&quot;, the output is inverted, that is, a 0 is inserted
            HDLC_WIRITE_TX_BIT(HDLC_TX_BIT);
        }

        inbyte = inbyte >> 1; // Take the next one
    }
}

// void DISP_HDLC(uchar dat) //Convert to high and low symbol display, for debugging
// { fly i;
// for (i=0;i<8;i++)
// {
// if ((dat&amp;0x80)==0x80){UART1_SendData(&#39;-&#39;);}else{UART1_SendData(&#39;_&#39;);UART1_SendData(&#39; &#39;);}
// that&lt;&lt;=1; // 取下一位
// }
// UART1_SendData(&#39;|&#39;);
// }

// 
// void CMX865A_HDLC_TX(uchar *pData,uchar nlen) //HDLC data
// { uint i; uint HDLC_LEN; //buffer length
// 
// KISS_TO_ASCII(SN_RX_BUFFER,0); //The KISS data received by the radio, after RF decoding, convert to ASCII UI format and obtain the UI data length UI_DIGI_LEN
// UART4_SendString(SN_RX_BUFFER );
// UART1_SendString(SN_RX_BUFFER); //Serial port 1 monitoring
// 
// BT_OUT(); //Bluetooth output
// 
// Delay_time_25ms(2); //Delay 50ms to give the main board time to process
// 
// for (i=0;i<250;i++)   {HDLC_TX_BUF[i]=0;   }	  //
// HDLC_TX_IDX=0;  HDLC_TX_BIT=0;
// 
// GetCrc16_LEN(pData,nlen); //Calculate the checksum of data // SendData(FCS_LO); //58 //SendData(FCS_HI); //D5
// 
// 
// PTT=1;	   LED_STU=0; 	Delay_time_1ms(10);
// 
// CMX865A_TX_ON(); //Start PTT
// 
// 
// //coding
// for (i=0;i<(8*EEPROM_Buffer[0X07]);i++) {HDLC_SendByte2(0x7E,1);  }//发送50个前置FLAG标志数据 约等于PTT延时8x6.6=53ms
// for (i=0;i<(8*8);i++) {HDLC_SendByte2(0x7E,1);  }//发送50个前置FLAG标志数据 约等于PTT延时8x6.6=53ms
// 
// 
// for (i=0;i<nlen;i++) {HDLC_SendByte2(*pData,0); pData++;}	//编码
// HDLC_SendByte2(FCS_LO,0); HDLC_SendByte2(FCS_HI,0); //Send checksum data, low first then high
// for (i=0;i<2;i++)   {HDLC_SendByte2(0x7E,1);   }	  //发送后置FLAG标志数据
// 
// HDLC_LEN= HDLC_TX_IDX/8+1; //must be +2 or more
// for (i=0;i<HDLC_LEN;i++)   {CMX865A_TX_DATA(HDLC_TX_BUF[i]);  }	//发送
// 
// UART1_DEBUG(HDLC_LEN);
// 
// CMX865A_TX_OFF();
// PTT=0; LED_STU=1; //Turn off PTT
// 
// 
// for (i=0;i<HDLC_LEN;i++) 	{	DISP_HDLC(HDLC_TX_BUF[i]);  	}
// 
// UART1_SendString(&quot;TATAL: &quot;); UART1_DEBUG(HDLC_LEN);
// 
// 
// 
// UART1_SendData(0xC0); UART1_SendData(0x00);
// for (i=0;i<HDLC_LEN;i++)   {UART1_SendData(HDLC_TX_BUF[i]);   }// 调试
// UART1_SendData(FCS_LO);	  	UART1_SendData(FCS_HI);
// UART1_SendData(0xC0);
// 
// 
// 
// UART1_SendData(0xC0); UART1_SendData(0x00); //Bluetooth serial port outputs KISS data
// for (i=0;i<KISS_LEN;i++)    {  UART1_SendData(KISS_DATA[i]);}
// UART1_SendData(FCS_LO);  	UART1_SendData(FCS_HI);
// UART1_SendData(0xC0);
// 
// KISS_TO_ASCII(SN_RX_BUFFER,0); //Convert KISS data to ASCII UI format and get UI data length UI_DIGI_LEN
// KISS_TO_ASCII(SN_RX_BUFFER,1); //The KISS data received by the radio, after RF decoding, convert to ASCII UI format and obtain the UI data length UI_DIGI_LEN
// UART1_SendString(SN_RX_BUFFER); //Serial port 1 monitoring
// 
// }





void CMX865A_HDLC_TX(uchar *pData, uchar nlen)	  // HDLC Encoding
{
    uint i;
    uint HDLC_LEN;	  // Buffer length

// Delay_time_25ms(1); //Delay 50ms to give the main board time to process
// CMX865A_Init();


    for (i = 0; i < 250; i++)
    {
        HDLC_TX_BUF[i] = 0;      // 
    }

    HDLC_TX_IDX = 0;
    HDLC_TX_BIT = 1;

    GetCrc16_LEN(pData, nlen);	// Calculate the checksum of data // SendData(FCS_LO); //58 // SendData(FCS_HI); //D5


// PTT=1;
// LED_STU=0;	 //Delay_time_1ms(10);
// CMX_RX_OFF(); //Turn off receiving
    CMX865A_TX_ON();  	// Start PTT

    // coding
    for (i = 0; i < (8 * EEPROM_Buffer[0X07] + 30); i++)
    {
        HDLC_SendByte2(0x7E, 1);     // Sending 50 pre-flag data is approximately equal to PTT delay 8x6.6=53ms
    }

    for (i = 0; i < nlen; i++)
    {
        HDLC_SendByte2(*pData, 0);
        pData++;
    }

    HDLC_SendByte2(FCS_LO, 0);
    HDLC_SendByte2(FCS_HI, 0);	   	// Send checksum data, low first then high

    for (i = 0; i < 2; i++)
    {
        HDLC_SendByte2(0x7E, 1);      // Send post-FLAG mark data
    }

    // send
    HDLC_LEN =  HDLC_TX_IDX / 8 + 1; // Must be +2 or above

    for (i = 0; i < HDLC_LEN; i++)
    {
        CMX865A_TX_DATA(HDLC_TX_BUF[i]);     // Sending Data
    }

// UART1_DEBUG(HDLC_LEN);

    CMX865A_TX_OFF();
// CMX_RX_ON();

// PTT=0; // LED_STU=1; //Turn off PTT




// UART2_SendString(&quot;$E00\r\n&quot;); UART4_SendString(&quot;$E00\r\n&quot;); Delay_time_25ms(1);
// UART2_SendString(&quot;$E00\r\n&quot;); UART4_SendString(&quot;$E00\r\n&quot;); Delay_time_25ms(1);

}

