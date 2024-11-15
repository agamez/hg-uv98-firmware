#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "UART1.H"
#include "UART2.H"
#include "DELAY.H"

#include "CRC16.H"
#include "APRS_RF.H"
#include "io.H"
#include "KISS_DECODE.H"

#include "CMX865A_SPI.H"
#include "PUBLIC_BUF.H"
#include "tostring.H"


/********* APRS codec***********/
// sbit FX604_M1=P4^5;
// sbit FX604_M0=P2^7;
// sbit PTT = P2^6;
// sbit FX604_DET=P2^4;
// sbit FX604_RX=P2^3;
// sbit FX604_TX=P2^2;

bit aprs_flag;	 // Check if flag=7E is set
bit old_input;	 // Pin inversion status
bit input_rev;	 // Whether the level is inverted


void Delay_417us();					 // Codec Timing
void DELAY_833US();	  // Use as a timer, delay 833us

unsigned char APRS_STATUS;			// APRS decoding status, =03 normal, other indications are unexpected disconnection, constant sound, noise, data synchronization abnormality and other errors
unsigned char one_count;	 		// Codec, count of consecutive 1s
unsigned char HDLC_DATA;			// APRS decoding gets 1 byte

uchar  DATA_7E;	// 0 = front and back flags, 1 = data 7E

unsigned char RF_DECODE();			 // RF APRS Decoding

uchar check_crc_a();		 // Check the last 2 digits of the data

/*****************************************/
void DELAY_TX_833US();	  // Use TIME1 as timer, delay 833us


sbit FX604_RX  = P1 ^ 6;



void DELAY_833US()	  // Use TIME1 as timer, delay 833us
{
    CCON = 0;					// Clear CF, CR, CCF0, CCF1
    CH = (65536 - 760 * 2) / 256;		// The PCA reference timer is initialized.
    CL = (65536 - 760 * 2) % 256;		// ;65536-22.1184M oscillation MHZ/12*833US =65536-1535
    CR = 1;						// Start the PCA timer.
}

void Delay_417us()	  		// Use TIME1 as timer, delay 417us
{
    CCON = 0;					// Clear CF, CR, CCF0, CCF1
    CH = (65536 - 384 * 2) / 256;		// The PCA reference timer is initialized.
    CL = (65536 - 384 * 2) % 256;		// ;65536-11.0592M oscillation MHZ/12*50000US =65536-46080
    CR = 1;						// Start the PCA timer.

    while (!CF);
}





uchar check_crc_a()		 // Check the last 2 digits of the data
{
    GetCrc16_LEN(KISS_DATA, (KISS_LEN - 2));	 // Calculate the checksum of the data

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



// Retrieve the first FLAG=7E
void APRS_check_flag()
{
    unsigned char i;
    uint stu;

    APRS_STATUS = 0x00;
    HDLC_DATA = 0x00;

    if ((CMX865A_READ_E6() & 0x0001) == 0x0001)
    {
        FX604_RX = 0;
    }
    else
    {
        FX604_RX = 1;
    }

    old_input = FX604_RX;	 // Recording input level

    one_count = 0;	  // Count of consecutive 1s
    i = 0;

    while (1)
    {
        Delay_417us();

        HDLC_DATA  >>= 1;			// Shift right 1 bit, default is 0

        if  (one_count == 6)		  	// ;Current = 0, the first 6 1s, the first FLAG = 7E is found
        {
            i++;		 	// 3 consecutive detections of FLAG=7E

            if (i == 3)
            {
                APRS_STATUS = 0x03;
                one_count = 0;
                return;	 // The search is successful, return
            }    	// Count of consecutive 1s
        }

        one_count = 0;	    	// Count of consecutive 1s
        input_rev = 0;		 	// Level flip mark

        while (input_rev == 0)
        {
            DELAY_833US();		// Delay 833us

            while (!CF)
            {
                stu = CMX865A_READ_E6();

                if ((stu & 0x0400) != 0x0400)
                {
                    APRS_STATUS = 0x04;    // Abnormal error jump //E6 //New data, B10=1 B6=1
                    return;
                }

                if ((stu & 0x0001) == 0x0001)
                {
                    FX604_RX = 0;
                }
                else
                {
                    FX604_RX = 1;
                }

                if (FX604_RX != old_input)
                {
                    old_input = FX604_RX;	 // Recording input level
                    input_rev = 1;	 // Level flip occurs
                    break;
                }
            }

            if (input_rev == 0)	  	// No flip, write 1
            {
                HDLC_DATA  >>= 1;	// Shift right 1 bit first
                HDLC_DATA |= 0x80; 	  // Write 1

                one_count++;		  // Count of consecutive 1s +1

                if (one_count == 7)
                {
                    APRS_STATUS = 0x05;
                    return;
                }

                // ;An error occurred, 7 consecutive &quot;1&quot;, may be a long tone or non-APRS data //; Re-detect
            }
        }
    }
}




void APRS_RECDATA()
{
    unsigned char bit_count;
    uint stu;

    DATA_7E = 0;	 // Data 7E flag cleared

    bit_count = 0; // Receive 8 bits

    while (bit_count < 8)
    {
        DELAY_833US();	    // Delay 833us

        while (!CF)
        {
            stu = CMX865A_READ_E6();

            if ((stu & 0x0400) != 0x0400)
            {
                APRS_STATUS = 0x04;    // Abnormal error jump //E6 //New data, B10=1 B6=1
                return;
            }

            if ((stu & 0x0001) == 0x0001)
            {
                FX604_RX = 0;
            }
            else
            {
                FX604_RX = 1;
            }

            if (FX604_RX != old_input)
            {
                old_input = FX604_RX;	 // Recording input level
                input_rev = 1;	 // Level flip occurs
                break;
            }
            else
            {
                input_rev = 0;   // Level not flipped mark
            }
        }

        if (input_rev == 0)	  	// No flip, write 1
        {
            HDLC_DATA  >>= 1;	// Shift right 1 bit first
            HDLC_DATA |= 0x80; 	// Write 1

            one_count++;		  // Count of consecutive 1s +1

            if (one_count == 7)
            {
                APRS_STATUS = 0x05;
                return;
            }

            // ; An error occurred, 7 consecutive &quot;1&quot;, which may be a long tone or non-APRS data
            // ; Retest
            bit_count++;
        }
        else		   // Level flip occurs
        {
            Delay_417us();

            if (one_count == 6)
            {
                DATA_7E = 1;	   // It is the 7E logo
            }

            if (one_count == 5)	// If there are 5 consecutive &quot;1&quot;s in front, this bit will be ignored.
            {

            }
            else
            {
                HDLC_DATA  >>= 1;	// Shift right 1 bit, default is 0
                bit_count++;
            }

            one_count = 0;	  // Count of consecutive 1s


        }
    }
}


unsigned char RF_DECODE()	   // RF Decoding KISS Data
{
    KISS_LEN = 0;				// Receive the first KISS data

    APRS_check_flag();		 // Find the first FLAG=7E at the front of the data packet

    if (APRS_STATUS != 0x03)
    {
        return 1;    // If there are abnormal errors such as disconnection, noise, long sound, etc., the system will pop up
    }

    if (HDLC_DATA != 0x7e)
    {
        return 2;   // If there are abnormal errors such as disconnection, noise, long sound, etc., the system will pop up
    }

    // Wait for the 7E at the front of the data packet to be received

    while (1)		// Wait for 7E in the packet header to be received completely
    {
        APRS_RECDATA();

        if (HDLC_DATA != 0x7e)
        {
            break;
        }

        if (APRS_STATUS != 0x03)
        {
            return 3;     // If there are abnormal errors such as disconnection, noise, long sound, etc., the system will pop up
        }
    }


    KISS_LEN = 0;				// Receive the first KISS data
    KISS_DATA[KISS_LEN] = HDLC_DATA;
    KISS_LEN++;



    while (1)		// Middle of packet
    {
        APRS_RECDATA();

        if (APRS_STATUS != 0x03)
        {
            return 4;      // If there are abnormal errors such as disconnection, noise, long sound, etc., the system will pop up
        }

        KISS_DATA[KISS_LEN] = HDLC_DATA;
        KISS_LEN++;	   // Deposit data

        if (KISS_LEN > 120)
        {
            return 5;   // The receiving length is too long. For example, if two radio stations are transmitting at the same time, the data partially overlaps and an abnormal error pops up.
        }

        if (DATA_7E == 1)
        {
            break ;   // Ending sign 7E
        }
    }

    KISS_LEN = KISS_LEN - 1;		 // Total length of received data, minus the last 7E byte received, including 2-byte or 1-byte checksum

// KISS_LEN=KISS_LEN-2; //Total length of received data, minus the last 2 7E bytes received, including 2-byte or 1-byte checksum
    if 	   (check_crc_a() == 1)
    {
        return 6;   // Check the last 2 digits of the data
    }

// if (check_crc_b()==1) {return 1;} // check the last digit of the data + 7E
    return 7;				   // Check data error
}
