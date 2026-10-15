#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"


#include "UART1.H"
#include "UART2.H"

#include "DELAY.H"
#include "io.H"

#include "CMX865A_SPI.H"
#include "CMX865A_DECODE.H"
#include "CRC16.H"

#include "tostring.H"

// #include "BEACON.H"
#include "KISS_Analysis.H"
#include "PUBLIC_BUF.H"



#define MAX_LEN 200	 // The maximum length of KISS data allowed to be received

#include  <INTRINS.H> // Keil library


bit HDLC_RX_BIT;
uchar HDLC_RX_COUNT;			// Reset the count of 5 consecutive 1s

// ************* Receive Register
uchar HDLC_RX_BUF[400];
// float APRS_KISS_BUF[300];

uchar HDLC_RX_BIT_OLD;// The state of the last bit
uchar BIT_1_COUNT; // Count of consecutive 1s
uchar END_7E; 	   // 7E End Marker
uchar HDLC_RX_TEMP;   // Temporary decoded data
uint  TOTAL_IDX;   // Total number of digits

// ************* Output KISS data register
uint HDLC_RX_IDX;	  // Current bit index
uint HDLC_RX_LEN;	  // Buffer length


uchar CMX_RX_BUSY;


uint RX_OK_COUNT;

sbit sig_in  = P1 ^ 6;


// Each frame has a flag code 01111110 before and after, which is used to indicate the start and end of the frame and for frame synchronization.
// The flag code is not allowed to appear inside the frame to avoid ambiguity.
// In order to ensure the uniqueness of the identification code while taking into account the transparency of the data within the frame, the &quot;0-bit insertion method&quot; can be used to solve the problem.
// This method monitors all fields except the flag code at the sending end.
// When five consecutive &quot;1&quot;s are found, a &quot;0&quot; is inserted after them, and then the subsequent bit stream is sent.
// At the receiving end, all fields except the start flag code are also monitored.
// When five &quot;1&quot;s are found in succession, if the next bit is &quot;0&quot;, it will be automatically deleted to restore the original bit stream;
// If six consecutive &quot;1&quot;s are found, it may be that the inserted &quot;0&quot; has an error and turned into a &quot;1&quot;, or it may be that the frame end flag code has been received.
// The latter two cases can be further distinguished by the frame check sequence in the frame.
// The principle of the &quot;0-bit insertion method&quot; is simple and very suitable for hardware implementation.




uchar HDLC_READ_BIT()	 // Read a 0 or 1
{
    uint byte_idx;
    uchar bit_idx;
    HDLC_RX_IDX++;
    byte_idx = HDLC_RX_IDX / 8;
    bit_idx = HDLC_RX_IDX % 8;

    if ((HDLC_RX_BUF[byte_idx] & (0x80 >> bit_idx)) != 0)
    {
        return 1;
    }

    return  0 ;
}

// Retrieve the first FLAG=7E
uchar HDLC_START_7E()
{
    uchar COUNT_7E;
    uchar BIT_STU;// 7E Count

    HDLC_RX_TEMP = 0x00;
    HDLC_RX_BIT_OLD = HDLC_READ_BIT();	 // Recording start input level

// UART1_SendString("7E start IDX:  ");UART1_DEBUG(HDLC_RX_IDX);
// if (HDLC_RX_BIT_OLD==1){	UART1_SendString("srat 1\r\n"); }else{	UART1_SendString("srat 0\r\n");}

    BIT_1_COUNT = 0;	// Count of consecutive 1s
    COUNT_7E = 0;	  // 7E Count

    while (1)
    {
        if (HDLC_RX_IDX > (TOTAL_IDX - 2))
        {
            return 0x00;   // Search too long
        }

        BIT_STU	= HDLC_READ_BIT();

        if (BIT_STU != HDLC_RX_BIT_OLD) // Level flip occurs
        {
            HDLC_RX_BIT_OLD = BIT_STU; // Recording input level
            HDLC_RX_TEMP  >>= 1;			  // Shift right 1 bit, default is 0

            if  (BIT_1_COUNT == 6)		 // ;Current = 0, the first 6 1s, the first FLAG = 7E is found
            {
                COUNT_7E++;		 	// 3 consecutive detections of FLAG=7E

                if (COUNT_7E > 2)
                {
                    BIT_1_COUNT = 0;	   // Retrieval success, return
                    return 0x01;
                }

// UART1_SendString("7E DAT: ");   UART1_SendData(Hex2Ascii2(HDLC_RX_TEMP>>4)); UART1_SendData(Hex2Ascii2(HDLC_RX_TEMP));	  UART1_SendString("\r\n");
// BIT_1_COUNT=0;	return 0x01;
            }

            BIT_1_COUNT = 0;	 // Count of consecutive 1s
        }
        else			 // No flip, write 1
        {
            HDLC_RX_TEMP  >>= 1;	// Shift right 1 bit first
            HDLC_RX_TEMP |= 0x80; 	  // Write 1

            BIT_1_COUNT++;		  // Count of consecutive 1s +1
// if (BIT_1_COUNT==7) { return 0x05;} //Error, 7 consecutive &quot;1&quot;, may be a long tone or non-APRS data // Re-detect
        }
    }

    return 0x00;
}



uchar HDLC_RX_BYTE()
{
    unsigned char bit_count;
    uchar BIT_STU;

    END_7E = 0;	 // Data 7E flag cleared
    bit_count = 0; // Receive 8 bits

    while (bit_count < 8)
    {
        if (HDLC_RX_IDX > (TOTAL_IDX - 2))
        {
            return 0x00;   // Search too long
        }

        BIT_STU	= HDLC_READ_BIT();

        if (BIT_STU != HDLC_RX_BIT_OLD) // Level flip occurs
        {
            HDLC_RX_BIT_OLD = BIT_STU; // Recording input level

            if (BIT_1_COUNT != 5)	// If it is not 5 consecutive 1s, then receive one bit. If it is preceded by 5 consecutive &quot;1s&quot;, then ignore this bit.
            {
                if (BIT_1_COUNT == 6)
                {
                    END_7E = 1;	   // 7E flag detected
                }

                HDLC_RX_TEMP  >>= 1;	// Shift right 1 bit, default is 0
                bit_count++;
            }

            BIT_1_COUNT = 0;	  // Count of consecutive 1s
        }
        else			        // No flip, write 1
        {
            HDLC_RX_TEMP  >>= 1;	// Shift right 1 bit first
            HDLC_RX_TEMP |= 0x80; 	// Write 1

            BIT_1_COUNT++;		  // Count of consecutive 1s +1

            if (BIT_1_COUNT == 7)
            {
                return 0x05;   // An error occurred, 7 consecutive &quot;1&quot;, which may be a long tone or non-APRS data // Re-detect
            }

            bit_count++;
        }
    }

    return 	1;
}

// uchar HDLC_CRC() //Check the last 2 bits of data
// {
// GetCrc16_LEN(KISS_DATA,HDLC_RX_LEN-2); //Calculate the checksum of data
// disp_Hex2Ascii2(FCS_LO);   disp_Hex2Ascii2(FCS_HI);
// 
// if (FCS_LO!=KISS_DATA[HDLC_RX_LEN-2])	  { return 0;   }
// if (FCS_HI!=KISS_DATA[HDLC_RX_LEN-1])	  { return 0;   }
// KISS_LEN=HDLC_RX_LEN-2;
// return 1; // The comparison check code is correct and the decoding is successful
// }
// 
// 
// fly HDLC_DECODE( )
// { //uint i; //uchar stu;
// 
// if (HDLC_START_7E()!=0x01) { return 0; } //Find the first FLAG=7E at the front of the data packet. If the 7E start marker is not found, exit
// 
// while (1) //Wait for all 7E in the packet header to be received
// {
// if (HDLC_RX_BYTE()!=0x01) { return 0; } //If there is an abnormal error such as disconnection, noise, long sound, etc., the program will be exited
// if (HDLC_RX_TEMP!=0x7e)	{break; 	}
// }
// 
// HDLC_RX_LEN=0; //Receive the first KISS data
// 
// while (1) //Middle of data packet
// {
// KISS_DATA[HDLC_RX_LEN++]=HDLC_RX_TEMP;
// if (HDLC_RX_BYTE()!=0x01) { return 0; } //If there is an abnormal error such as disconnection, noise, long sound, etc., the program will be exited
// if (HDLC_RX_LEN&gt;120) { return 0;} //The receiving length is too long. If two radio stations are transmitting at the same time, the data will overlap and an abnormal error will be thrown.
// if (END_7E==1){break;} //End mark 7E
// }
// 
// if (HDLC_CRC()==1) { return 1;} //Check data last 2 bits //Check data error
// return 0;
// }
// 


uchar HDLC_CRC()		 // Check the last 2 digits of the data
{
    GetCrc16_LEN(KISS_DATA, KISS_LEN - 2);	 // Calculate the checksum of the data
// disp_Hex2Ascii2(FCS_LO);   disp_Hex2Ascii2(FCS_HI);

    if (FCS_LO != KISS_DATA[KISS_LEN - 2])
    {
        return 0;
    }

    if (FCS_HI != KISS_DATA[KISS_LEN - 1])
    {
        return 0;
    }

    KISS_LEN = KISS_LEN - 2;
    return 1;	// The comparison check code is correct and the decoding is successful.
}


uchar HDLC_DECODE( )
{
    if (HDLC_START_7E() != 0x01)
    {
        return 0;    // Look for the first FLAG=7E at the front of the data packet. If the 7E start marker is not found, exit.
    }

    while (1)		// Wait for 7E in the packet header to be received completely
    {
        if (HDLC_RX_BYTE() != 0x01)
        {
            return 1;     // If there are abnormal errors such as disconnection, noise, long sound, etc., the system will pop up
        }

        if (HDLC_RX_TEMP != 0x7e)
        {
            break;
        }
    }

    KISS_LEN = 0;		// Receive the first KISS data

    while (1)		// Middle of packet
    {
        KISS_DATA[KISS_LEN++] = HDLC_RX_TEMP;

        if (HDLC_RX_BYTE() != 0x01)
        {
            return 2;      // If there are abnormal errors such as disconnection, noise, long sound, etc., the system will pop up
        }

        if (KISS_LEN > 190)
        {
            return 3;   // The receiving length is too long. For example, if two radio stations are transmitting at the same time, the data partially overlaps and an abnormal error pops up.
        }

        if (END_7E == 1)
        {
            break ;   // Ending sign 7E
        }
    }

    if 	(HDLC_CRC() == 1)
    {
        return 5;   // Check the last 2 digits of the data //Check data error
    }

// UART1_SendString(&quot;KISS RX: &quot;); DEBUG_KISS(KISS_DATA,KISS_LEN);

    return 4;
}

void DISP_HDLC(uchar dat)	// Convert to high and low symbol display, for debugging
{
    uchar i;

    for (i = 0; i < 8; i++)
    {
        if ((dat & 0x80) == 0x80)
        {
            UART2_SendData('-');
        }
        else
        {
            UART2_SendData('_');
            UART2_SendData(' ');
        }

        dat <<= 1; // Take the next one
    }
}





uchar CMX865A_HDLC_RX()			// Exclusive decoding method
{
    uchar DCD;
    uint i;
    uint stu;
    uint over_err;
    uint fram_err;

// DCD=0;
// HDLC_RX_LEN=0;
// while (CMX865A_DET()==1) //Detect the signal and start recording data
// {
// DCD=1;
// HDLC_RX_BUF[HDLC_RX_LEN++]=CMX865A_RX_DATA();
// if (HDLC_RX_LEN&gt;295){break;} //Limit data, 300 bytes at most, if it is too long, it will be jumped
// }
// 
// if (DCD==0){return 0;} //No signal jump out



    stu = CMX865A_READ_E6() ;		  // E6 //New data, B10=1 B6=1

    if ((stu & 0x0400) != 0x0400)
    {
        return 0;
    }

    HDLC_RX_LEN = 0;
    over_err =	fram_err = 0;

    while(1)
    {

        if ((stu & 0x0001) == 0x0001)
        {
            sig_in = 0;
        }
        else
        {
            {
                sig_in = 1;
            }
        }

        if ((stu & 0x0010) == 0x0010)
        {
            fram_err++;   // 
        }

        if ((stu & 0x0020) == 0x0020)
        {
            over_err++;   // 
        }


        if ((stu & 0x0040) == 0x0040)	 // Receive new data, B6=1
        {
            HDLC_RX_BUF[HDLC_RX_LEN++] = CMX865A_READ_E5();

            if (HDLC_RX_LEN > 295)
            {
                break;   // Limit data to 300 bytes at most, if it is too long, it will be skipped
            }
        }

        stu = CMX865A_READ_E6() ;

        if ((stu & 0x0400) != 0x0400)
        {
            break;
        }


    }




// UART1_SendData(0 yes);
// for (i=0;i<len;i++)   {	UART1_SendData(HDLC_RX_BUF[i]);  }// 调试
// for (i=0;i<30;i++)   {	UART1_SendData(0X00);  }// 调试

// UART1_SendString(&quot;========\r\n&quot;); //Format debugging with number
// for (i=0;i<40;i++)
// {
// UART1_SendString(&quot;(&quot;); UART1_DEBUG2(i*5*8); UART1_SendString(&quot;): &quot;);
// 
// for (n=0;n<5;n++) 	{	DISP_HDLC(HDLC_RX_BUF[i*5+n]);  	}
// UART1_SendString(&quot; \r\n&quot;);
// }
// UART1_SendString(&quot;========\r\n&quot;);


    UART2_SendString("TATAL:  ");
    UART2_DEBUG(HDLC_RX_LEN);
    UART2_SendString("over:  ");
    UART2_DEBUG(over_err);
    UART2_SendString("err:  ");
    UART2_DEBUG(fram_err);


    RX_OK_COUNT++;	// Decoding success count
    UART2_SendString("RX COUNT:  ");
    UART2_DEBUG(RX_OK_COUNT);

    for (i = 0; i < HDLC_RX_LEN; i++)
    {
        DISP_HDLC(HDLC_RX_BUF[i]);
    }


// return 0;

    TOTAL_IDX = HDLC_RX_LEN * 8; // Total index length
    HDLC_RX_IDX = 0;	  		 // Start checking 7E index position // UART1_SendString(&quot;TATAL: &quot;); UART1_DEBUG(TOTAL_IDX);

    for (i = 0; i < 10; i++) 	 // Up to 10 consecutive decodings
    {
        stu = HDLC_DECODE();
        UART2_SendString("err:  ");
        UART2_SendData(stu + 0x30);
        UART2_SendString("\r\n");

        if (stu == 5)
        {
            return 1;	   // Decoding success
        }
    }


    return 0;
}




uchar CMX865A_HDLC_RX_2()			// Interrupt decoding method
{
    uint i;
    uchar stu;

    stu = 0;

    if (CMX_RX_BUSY == 1)
    {
// LED_STU=0;

        RX_OK_COUNT++;	// Decoding success count
// UART2_SendString(&quot;TABLE: &quot;); UART2_DEBUG(HDLC_RX_LEN);
// UART2_SendString("RX COUNT:  ");	UART2_DEBUG(RX_OK_COUNT);
// for (i=0;i<HDLC_RX_LEN;i++) 	{	DISP_HDLC(HDLC_RX_BUF[i]);  	}


        TOTAL_IDX = HDLC_RX_LEN * 8; // Total index length
        HDLC_RX_IDX = 0;	 // Start checking at index position 7E

        for (i = 0; i < 10; i++) 	 // Up to 10 consecutive decodings
        {
            stu = HDLC_DECODE();

// UART2_SendString(&quot;err: &quot;); UART2_SendData(stu+0x30); UART2_SendString(&quot;\r\n&quot;);
            if (stu == 5)
            {
                break;	   // Decoding success
            }
        }

        HDLC_RX_LEN = 0;	// The receiving length is cleared to 0
        CMX_RX_BUSY = 0;	// Re-receive
    }

// UART1_SendData(0 yes);
// for (i=0;i<len;i++)   {	UART1_SendData(HDLC_RX_BUF[i]);  }// 调试
// for (i=0;i<30;i++)   {	UART1_SendData(0X00);  }// 调试

// UART1_SendString(&quot;========\r\n&quot;); //Format debugging with number
// for (i=0;i<40;i++)
// {
// UART1_SendString(&quot;(&quot;); UART1_DEBUG2(i*5*8); UART1_SendString(&quot;): &quot;);
// 
// for (n=0;n<5;n++) 	{	DISP_HDLC(HDLC_RX_BUF[i*5+n]);  	}
// UART1_SendString(&quot; \r\n&quot;);
// }
// UART1_SendString(&quot;========\r\n&quot;);

    return stu;
}

// Read received data, the status register changes as follows, B5 = 0 B6 = 0 3C 00 Reception completed 3C 40 Reception overflow 3C 60

void CMX_RX_INT()	// Timed interrupt, 5ms interrupt once
{
    uint DCD;

// return;

    if (PTT == 1)
    {
        return;   // If in the transmitting state, no data is received
    }

    if (CMX_RX_BUSY == 1)
    {
        return;   // Waiting for data processing, no data is received
    }

    DCD = CMX865A_READ_E6(); // E6 //New data, B10=1 B6=1

    if ((DCD & 0x0400) != 0x0400)	 // B10=0 signal disappears. Data length &lt;20, then it is invalid and received again. Otherwise, data reception is completed.
    {
        if (HDLC_RX_LEN < 20)
        {
            HDLC_RX_LEN = 0;
        }
        else
        {
            CMX_RX_BUSY = 1;
        }

        return ;
    }

    // B10=1
    if ((DCD & 0x0040) != 0x0040)
    {
        return ;	 	   // B6=0 No data to receive
    }

    // B6=1
    HDLC_RX_BUF[HDLC_RX_LEN++] = CMX865A_READ_E5();

    if (HDLC_RX_LEN > 295)
    {
        CMX_RX_BUSY = 1;   // Limit data to 300 bytes at most, if it is too long, it will be skipped
    }
}

void CMX_RX_Initial()	// Timer interrupt
{
    CMX_RX_BUSY = 0;
    HDLC_RX_LEN = 0;	// The receiving length is cleared to 0


    RX_OK_COUNT = 0;	// Decoding success count
}
