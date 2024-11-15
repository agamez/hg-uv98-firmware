#include "STC8A8K64D4.H"
#include "PUBLIC_BUF.H"



#include "UART2.H"

#include "DELAY.H"



#include "KISS_Analysis.H"
#include "BEACON_SAVE.H"
#include "GPS2.H"
// #include "BEACON.H"

#include "AT24C512.h"
#include "tostring.H"



#include  <string.H> // Keil library




#include  <INTRINS.H> // Keil library



#define nop() _nop_()



#define MAX_LEN 100

void WRITE_BECAON(uint idx)
{
    uint i, k, len;
    uint add;
    uchar temp[20];

    // If not initialized, the initial address is 0
// if (AT24CXX_READ(0x8000)>99) {AT24CXX_WRITE(0x8000,0);  }
// idx=AT24CXX_READ(0x8000);

    // Write Record
    for (i = 0; i < 256; i++)
    {
        AT24C512_RW_BUF[i] = 0x00;     // Clear all
    }

    // Write the first 128 bytes of data, including date, time, distance, relative north direction, and relative vehicle head direction

    if (GPS_LOCKED == 1)
    {
        AT24C512_RW_BUF[0x19] = 'A';

        for (i = 0; i < 8; i++)
        {
            AT24C512_RW_BUF[i] = GPS_TIME[i];   // Write time, up to 8 bytes
        }

        for (i = 8; i < 16; i++)
        {
            AT24C512_RW_BUF[i] = GPS_DATE[i - 6];   // Write date, up to 8 bytes
        }
    }
    else
    {
        AT24C512_RW_BUF[0x19] = 'V';

        for (i = 0; i < 16; i++)
        {
            AT24C512_RW_BUF[i] = '-';   // Date written
        }

        AT24C512_RW_BUF[0x02] = ':';
        AT24C512_RW_BUF[0x05] = ':';
        AT24C512_RW_BUF[0x0a] = ':';
        AT24C512_RW_BUF[0x0d] = ':';
    }

    for (i = 0; i < 9; i++)
    {
        AT24C512_RW_BUF[0x10 + i] = UI_CALL[i];   // Write call sign, up to 9 bytes
    }

    for (i = 0; i < 8; i++)
    {
        AT24C512_RW_BUF[0x20 + i] = UI_WD[i];   // Write latitude, up to 8 bytes
    }

    AT24C512_RW_BUF[0x27] = UI_WD_DIR;

    for (i = 0; i < 6; i++)
    {
        AT24C512_RW_BUF[0x29 + i] = UI_JULI[i];   // Writing distance, up to 6 bytes
    }

    for (i = 0; i < 9; i++)
    {
        AT24C512_RW_BUF[0x30 + i] = UI_JD[i];   // Write longitude, up to 9 bytes
    }

    AT24C512_RW_BUF[0x38] = UI_JD_DIR;

    // ==============================================
    for (i = 0; i < 6; i++)
    {
        AT24C512_RW_BUF[0x40 + i] = UI_ALT[i];   // Write altitude, up to 6 bytes
    }

    for (i = 0; i < 6; i++)
    {
        AT24C512_RW_BUF[0x47 + i] = UI_SPEED[i];   // Write speed, up to 6 bytes
    }

    // ==============================================

    // --------------------------------------------------------
    for (i = 58; i < 61; i++)
    {
        AT24C512_RW_BUF[i] = UI_DIR[i - 58];   // Write heading, up to 3 bytes
    }



    AT24C512_RW_BUF[0X3F] =	 UI_ICON  ; // icon
    // --------------------------------------------------------
    k = 0;
    tostring(UI_Angle_N);	   // Relative north direction, up to 3 bytes
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    for (i = 0; i < 3; i++)
    {
        AT24C512_RW_BUF[0x1a + i] = temp[i];
    }

    // --------------------------------------------------------
    k = 0;
    tostring(UI_Angle_CAR);		 // Relative vehicle head position, up to 3 bytes
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    for (i = 0; i < 3; i++)
    {
        AT24C512_RW_BUF[0x1d + i] = temp[i];
    }

    if (KISS_LEN > 127)
    {
        len = 127;
    }
    else
    {
        len = KISS_LEN;
    }

    for (i = 0; i < len; i++)
    {
        AT24C512_RW_BUF[i + 128] = (KISS_DATA[i]);   // Write the original KISS data, the length must be less than 128
    }

    // 0x0000-0x6400 (0-25600), each beacon occupies 256 bytes, a total of 100 records, index 0-99 is stored at 0x8000
    add = idx * 256;	 // UART1_SendString(&quot;ADD: &quot;); UART1_DEBUG(idx);
    AT24CXX_WRITE_N(add, AT24C512_RW_BUF, 128);	 	// Write the first 128 bytes of memory in 2 passes, 128 bytes each
    AT24CXX_WRITE_N(add + 128, AT24C512_RW_BUF + 128, 128); // Write the last 128 bytes of memory
// UART1_SendString("DATA WRITE LEN:  ");	DEBUG_KISS(AT24C512_RW_BUF,256);
// for (i = 0; i<256; i++)   {  AT24C512_RW_BUF[i]=AT24CXX_READ(add+i);  }
// UART1_SendString("DATA READ LEN:  ");	DEBUG_KISS(AT24C512_RW_BUF,256);
}

void IDX_UPDATA_A(uint idx)	   // All indexes are moved backwards, and the latest beacon address is filled in the first place
{
    uint	i;
    AT24CXX_READ_N(0x6500, AT24C512_RW_BUF, 128);	// Read 128 bytes from the specified address

    for (i = 120; i > 0; i--)
    {
        AT24C512_RW_BUF[i] = AT24C512_RW_BUF[i - 1];  // Index all moved backwards
    }

    AT24C512_RW_BUF[0] = idx;
    AT24CXX_WRITE_N(0x6500, AT24C512_RW_BUF, 128);	 // Write 128 bytes to the specified address

// UART1_SendString(&quot;DATA IDX A: &quot;); DEBUG_KISS(AT24C512_RW_BUF,100); //debugging
}

uchar IDX_UPDATA_B(uint idx)	   // All indexes are moved backwards, and the latest beacon address is filled in the first place
{
    uint i;
    uchar temp;
    AT24CXX_READ_N(0x6500, AT24C512_RW_BUF, 128);	// Read 128 bytes from the specified address

    if (AT24C512_RW_BUF[0] == idx)	 // If the first comparison is consistent, there is no need to move backward
    {
        AT24C512_RW_BUF[0] = idx;
        AT24CXX_WRITE_N(0x6500, AT24C512_RW_BUF, 128);	 // Write 128 bytes to the specified address

// UART1_SendString(&quot;DATA IDX B: &quot;); DEBUG_KISS(AT24C512_RW_BUF,100); //debugging
    }
    else
    {
// AT24CXX_READ_N(0x6500,AT24C512_RW_BUF,128); //Read 128 bytes from the specified address

        for (i = 0; i < MAX_LEN; i++) 											 // ------100
        {
            if (AT24C512_RW_BUF[i] == idx)
            {
                temp = i;
                break;
            }

// if (i==MAX_LEN-1){ UART2_SendString(&quot;idx out: \r\n&quot;); return 2;} // Index number not found, error occurred //------ 99
            if (i == MAX_LEN - 1)
            {
                return 2;   // Index number not found, error occurred //------ 99
            }

        }


        for (i = temp; i > 0; i--)
        {
            AT24C512_RW_BUF[i] = AT24C512_RW_BUF[i - 1];  // All indexes move backwards by one row
        }

        AT24C512_RW_BUF[0] = idx;
        AT24CXX_WRITE_N(0x6500, AT24C512_RW_BUF, 128);	 // Write 128 bytes to the specified address

// UART1_SendString(&quot;DATA IDX C: &quot;); DEBUG_KISS(AT24C512_RW_BUF,100); //debugging
    }

    return 1;
}



uchar BEACON_SAVE()    // Write beacon list and store beacons
{
    uint i  ;
    uint idx, add;
    uchar temp[20];
    uchar *p;


    for (idx = 0; idx < MAX_LEN; idx++)
    {
        add = idx * 256;		 // UART1_DEBUG(add);

        for (i = 0; i < 9; i++)
        {
            temp[i] = AT24CXX_READ(add + 16 + i);    // Call Sign
            temp[i + 1] = 0;
        }

// UART1_SendString(&quot;CALL : &quot;); DEBUG_KISS(temp,9);
        if (temp[0] == 0xff)
        {
            IDX_UPDATA_A(idx);     // If there is no row, the beacon is directly filled in and the index is filled in the first row
            WRITE_BECAON(idx);
            return 0;
        }


        // //Read the call sign
        // for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(add+16+i); temp[i+1]=0; } 	 //呼号

        p = strstr(UI_CALL, temp);	// UI_CALL Compare call signs

        if  (p != NULL)			 	// Comparison successful, replace beacon segment
        {
            if( IDX_UPDATA_B(idx) == 1)
            {
                WRITE_BECAON(idx);
            }

            return 1;
            // If the update index is successful, the beacon segment is replaced
        }

        // The comparison was unsuccessful. Continue to compare.

        if (idx == MAX_LEN - 1) 	 	 // After all comparisons are completed, if no comparison is found, delete the last record
        {
            idx = AT24CXX_READ(0x6500 + MAX_LEN - 1);
            IDX_UPDATA_A(idx);
            WRITE_BECAON(idx);
            return 2;
        }
    }

    return 3;
}




// 0xFE00 Last 512 storage setting data