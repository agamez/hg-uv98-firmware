
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "DELAY.H"
#include "IO.H"

#include "DIGI.H"

#include "UART2.H"
#include "BEACON.H"
#include "CMX865A_CODE.H"
#include "UARTx.H"

#include "CHx.H"

#include "PUBLIC_BUF.H"
#include  <string.h>



/************* Digital relay**************/

void DIGI_FUN();

uchar 	CHECK_CODE()
{
    uchar *p;
    uchar temp[10];
    unsigned char  i ;

    for(i = 0; i < 6; i++)
    {
        temp[i] = EEPROM_Buffer[REMOTE_SN + i];
        temp[i + 1] = 0;
    }

    p = strstr(SN_RX_BUFFER, temp);

    if (p != NULL)
    {
        for(i = 0; i < 8; i++)
        {
            temp[i] = (*(p + i));
            temp[i + 1] = 0;
        }


        if ((temp[6] == 'A') && (temp[7] == '1'))
        {
            EEPROM_Buffer[DIGI1_EN] = 1;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 1 ON\r\n");
            return 1;
        }

        if ((temp[6] == 'A') && (temp[7] == '0'))
        {
            EEPROM_Buffer[DIGI1_EN] = 0;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 1 OFF\r\n");
            return 1;
        }

        if ((temp[6] == 'B') && (temp[7] == '1'))
        {
            EEPROM_Buffer[DIGI2_EN] = 1;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 2 ON\r\n");
            return 1;
        }

        if ((temp[6] == 'B') && (temp[7] == '0'))
        {
            EEPROM_Buffer[DIGI2_EN] = 0;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 2 OFF\r\n");
            return 1;
        }

        if ((temp[6] == 'R') && (temp[7] == '0'))
        {
            UART2_SendString("NOW RST\r\n");
            IAP_CONTR = 0x20;
            return 1;
        }

        return 1;
    }  // 

    return 0;
}








// Relay forwarding processing, return 0 = wrong name, 1 = count is 0, 2 = forwarding is allowed
unsigned char DIGI_NAME_CHECK(uchar *dig_name, uchar kiss_idx)
{
    unsigned char  SSID, i;
    uchar temp;
    uchar path_name[8];  	 // Path address 6 digits, including spaces

    // If the digital trunk NAME is on and the path contains NAME-N, and N&gt;0, then forward
// ----------------------------------------
    SSID =	 ((KISS_DATA[kiss_idx + 6] & 0x1E) >> 1) ;		// Find SSID

    if (SSID == 0)
    {
        return 1;    // The number of forwardings is 0, exit
    }

// ----------------------------------------
    for (i = 0; i < 6; i++)
    {
        path_name[i] = ' ';   // Filling spaces
    }

    for(i = 0; i < 6; i++) 			// Read out the trunk name
    {
        temp = *(dig_name + i);	 // 

        if (temp == 0x00)
        {
            break;
        }
        else
        {
            path_name[i] = temp;
        }
    }

    for(i = 0; i < 6; i++)
    {
        if (path_name[i] != KISS_DATA[kiss_idx + i] >> 1)
        {
            return 0;   // Compare Names
        }
    }

// ----------------------------------------

    // SSID>0
    SSID --; 	   // SSID Count - 1
    SSID <<= 1;	   // Shift left 1 bit
    KISS_DATA[kiss_idx + 6] &= 0xE1;	// Clear SSID, SSID=0 //The end mark of the end path remains unchanged
    KISS_DATA[kiss_idx + 6]  |= SSID;	// Write a new SSID

    if (SSID == 0)
    {
        KISS_DATA[kiss_idx + 6]  |= 0x80;   // When SSID=0, set SSID.7=1, which means the forwarding of this path is completed.
    }

    return 2;
}



// KISS data, insert the call sign before the specified DIGI and convert it into new KISS data
void DIGI_CALL_COV(uchar kiss_idx)
{
    unsigned char i, k, new_idx;

    uchar MY_CALL[7];

    // If the total length of the data is less than (128-9), the 14th bit of KISS is shifted right by 7 bytes, leaving 7 bytes to insert the relay name
    if  (KISS_LEN > 150)
    {
        return;   // If the total length of the KISS data is not enough to insert a 7-byte relay name, exit
    }


    if (EEPROM_Buffer[0x08] == 0x00)
    {
        return;   // Exit if no callsign
    }

    // --------------------------------

    // Leave 7 bytes before the specified index
    new_idx = KISS_LEN;
    k = KISS_LEN - kiss_idx + 1;

    for (i = 0; i < k; i++)
    {
        KISS_DATA[new_idx + 7] = KISS_DATA[new_idx];
        new_idx--;
    }

    // --------------------------------

    for (i = 0; i < 6; i++)
    {
        MY_CALL[i]	= 0x40;    // Source Call Sign Filler
    }

    for (i = 0; i < 6; i++)  		// Source Call Sign Filler
    {
        if (EEPROM_Buffer[0x08 + i] == 0x00)
        {
            break;
        }

        MY_CALL[i] = EEPROM_Buffer[0x08 + i] << 1;
    }

    MY_CALL[6] = (EEPROM_Buffer[0x0F] << 1) | 0xE0 ;
    k++;	  // Insert high bit = 1, Source SSID + path terminator = 0

    // --------------------------------
    // Leave 7 bytes blank to insert the local machine name
    for (i = 0; i < 7; i++)
    {
        KISS_DATA[kiss_idx + i] = MY_CALL[i];
    }

    // --------------------------------
    KISS_LEN = (KISS_LEN + 7) ;	 // KISS data length + 7
    // --------------------------------
// GetCrc16_LEN(KISS_DATA,KISS_LEN); // Calculate the 16-bit CRC of the given length data.
// APRS_TX(KISS_DATA,KISS_LEN); //Send RF

// UART2_SendData(0XC0); UART2_SendData(0X00);
// for (i=0;i<KISS_LEN;i++)    {  UART2_SendData(KISS_DATA[i]);	   }  //串口调试
// UART2_SendData(0XC0);
// UART2_SendData(FCS_LO)	;  	  UART2_SendData(FCS_HI)	;
}





// Relay sends processed KISS data
void DIGI_RF_TXD()
{
    uchar i;

    if (EEPROM_Buffer[0x01D5] > 0)
    {
        for (i = 0; i < EEPROM_Buffer[0x01D5]; i++)
        {
            Delay_time_25ms(40);
        }
    }

// CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);
    Delay_time_25ms(6);

    BEACON_TX_CHX(1);

}


void DIGI_FUN()
{
    uchar i, idx;

    CHECK_CODE();

// Relay field position, every 7 bytes is 1 segment, KISS data is checked from the 3rd segment
    if ((EEPROM_Buffer[DIGI1_EN] == 0) && (EEPROM_Buffer[DIGI2_EN] == 0))
    {
        return;
    }

    if ((KISS_DATA[13] & 0x01) == 1)
    {
        return;	   // If there is no relay path, exit
    }


    idx = 13;

    for (i = 0; i < 6; i++)	 // A maximum of 6 paths can be queried
    {

        if (EEPROM_Buffer[DIGI1_EN] != 0) // Digital trunk WIDE1 is on, and the path includes WIDE1-N, N&gt;0, then forward
        {
            if (DIGI_NAME_CHECK(EEPROM_Buffer + DIGI1_NAME, idx + 1) == 2)	// Compare relay name 1, forwarding times
            {
                DIGI_CALL_COV(idx + 1); 	   // Insert the local name, insert it at the front
                DIGI_RF_TXD();
                return;
            }
        }

        if (EEPROM_Buffer[DIGI2_EN] != 0) // Digital trunk WIDE2 is on, and the path contains WIDE2-N, N&gt;0, then forward
        {
            if (DIGI_NAME_CHECK(EEPROM_Buffer + DIGI2_NAME, idx + 1) == 2) // Compare relay name 2 and forwarding times
            {
                DIGI_CALL_COV(idx + 1); 	   // Insert the local name, insert it at the front
                DIGI_RF_TXD();
                return;
            }
        }

        if ((KISS_DATA[idx + 7] & 0x01) == 1)
        {
            return;	   // If there is no relay path, exit
        }

        idx = idx + 7;

    }


    return;

}


void dig_Initial()
{
    unsigned char code  TX_PATH1[8] = {"WIDE1 "};			// Transmit Path 1
    unsigned char code  TX_PATH2[8] = {"WIDE2 "};			// Transmit Path 2
    unsigned char code  DIG_PATH1[8] = {"WIDE1 "};			// Relay name 1, priority detection
    unsigned char code  DIG_PATH2[8] = {"WIDE2 "};			// Relay name 2, second priority detection


    unsigned char code  digi_on_code[6] = {"123456"};
// unsigned char code  digi_off_code[6]={"654321"};

    uchar i;

    // TX  WIDE1-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(PATH1_NAME + i, TX_PATH1[i]);   // 
    }

    EEPROM_write_one(PATH1_NAME + 6, 0x00); // End symbol
    EEPROM_write_one(PATH1_COUNT, 1 ); 	// 0=off, 1-9 forwarding times

    // TX  WIDE2-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(PATH2_NAME + i, TX_PATH2[i]);   // 
    }

    EEPROM_write_one(PATH2_NAME + 6, 0x00); // End symbol
    EEPROM_write_one(PATH2_COUNT, 0 ); 	// 0=off, 1-9 forwarding times

    // YOU WIDE1-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(DIGI1_NAME + i, DIG_PATH1[i]);   // 
    }

    EEPROM_write_one(DIGI1_NAME + 6, 0x00); // End symbol
    EEPROM_write_one(DIGI1_EN, 1 ); 	// 0 = close, 1 = forward

    // YOU WIDE2-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(DIGI2_NAME + i, DIG_PATH2[i]);   // 
    }

    EEPROM_write_one(DIGI2_NAME + 6, 0x00); // End symbol
    EEPROM_write_one(DIGI2_EN, 1 ); 	// 0 = close, 1 = forward




    // ********************************************************DIGI switch password
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(REMOTE_SN + i, digi_on_code[i]);
    }

    EEPROM_write_one(REMOTE_SN + 6, 0x00); // end symbol
// for(i=0;i<6;i++){EEPROM_write_one(0x0148+i, digi_off_code[i]);}
// EEPROM_write_one(0x0148+6, 0x00); //end symbol
}

// void READ_DIGI_STU()
// { fly i;
// --------------------------------------------------------
// UART2_SendString(&quot;AT+PATH1_NAME=&quot;); //0-9
// for(i=0;i<6;i++)
// {
// if (EEPROM_Buffer[PATH1_NAME+i]==0){break;}	UART2_SendData(EEPROM_Buffer[PATH1_NAME+i]);
// }
// UART2_SendString(&quot;\r\n&quot;);
// 
// UART2_SendString(&quot;AT+PATH1_COUNT=&quot;); UART2_SendData(EEPROM_Buffer[PATH1_COUNT]%10+0x30); UART2_SendString(&quot;\r\n&quot;);
// --------------------------------------------------------
// UART2_SendString(&quot;AT+PATH2_NAME=&quot;); //0-9
// for(i=0;i<6;i++)
// {
// if (EEPROM_Buffer[PATH2_NAME+i]==0){break;} 	UART2_SendData(EEPROM_Buffer[PATH2_NAME+i]);
// }
// UART2_SendString(&quot;\r\n&quot;);
// 
// UART2_SendString(&quot;AT+PATH2_COUNT=&quot;); UART2_SendData(EEPROM_Buffer[PATH2_COUNT]%10+0x30); UART2_SendString(&quot;\r\n&quot;);
// --------------------------------------------------------
// UART2_SendString(&quot;AT+DIGI1_NAME=&quot;);
// for(i=0;i<6;i++)
// {
// if (EEPROM_Buffer[DIGI1_NAME+i]==0){break;}   UART2_SendData(EEPROM_Buffer[DIGI1_NAME+i]);
// }
// UART2_SendString(&quot;\r\n&quot;);
// 
// if (EEPROM_Buffer[DIGI1_EN]==0) { UART2_SendString(&quot;AT+DIGI1=OFF&quot;);} else {UART2_SendString(&quot;AT+DIGI1=ON&quot;);} UART2_SendString(&quot;\r\n&quot;); //
// --------------------------------------------------------
// UART2_SendString(&quot;AT+DIGI2_NAME=&quot;);
// for(i=0;i<6;i++)
// {
// if (EEPROM_Buffer[DIGI2_NAME+i]==0){break;} 	UART2_SendData(EEPROM_Buffer[DIGI2_NAME+i]);
// }
// UART2_SendString(&quot;\r\n&quot;);
// 
// if (EEPROM_Buffer[DIGI2_EN]==0) { UART2_SendString(&quot;AT+DIGI2=OFF&quot;);} else {UART2_SendString(&quot;AT+DIGI2=ON&quot;);} UART2_SendString(&quot;\r\n&quot;); //
// WIDE1 is enabled
// 
// 
// --------------------------------------------------------
// 
// //Relay remote password
// UART2_SendString(&quot;AT+CODE=&quot;); for(i=0;i&lt;6;i++){UART2_SendData(EEPROM_Buffer[REMOTE_SN+i]);} UART2_SendString(&quot;\r\n&quot;); //
// UART2_SendString(&quot;AT+OFFSN=&quot;); for(i=0;i&lt;6;i++){UART2_SendData(EEPROM_Buffer[0X0148+i]);} UART2_SendString(&quot;\r\n&quot;); //
// 
// }
// 
// 
// 
// fly SET_PATH_DIGI()
// {
// uchar i,len;
// 
// //================================================ //Set the name of path 1
// if (CHECK_AT_CMD("AT+PATH1_NAME=")==1)
// {
// len=strlen(UARTx_BUF)-14-2;
// if (len&gt;6) { return 0;} //Relay name is too long, error
// if (len==0) { return 0;} //Relay name not set, error
// for(i=0;i<len;i++) 	{ EEPROM_Buffer[PATH1_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[PATH1_NAME+i+1]=0;  }
// EEPROM_UPDATA(); return 1;
// }
// //================================================ //Set the number of TXpath1 times 0-9 0=off
// if (CHECK_AT_CMD("AT+PATH1_COUNT=")==1)
// {	len=strlen(UARTx_BUF)-15-2;
// if (len==1)	{  	EEPROM_Buffer[PATH1_COUNT]=(UARTx_BUF[15]-0x30);EEPROM_UPDATA();  	return 1;	  }
// return 0;
// }
// 
// //================================================ //Set the name of path 2
// if (CHECK_AT_CMD("AT+PATH2_NAME=")==1)
// {
// len=strlen(UARTx_BUF)-14-2;
// 
// if (len&gt;6) { return 0;} //Relay name is too long, error
// if (len==0) { return 0;} //Relay name not set, error
// for(i=0;i<len;i++) 	{ EEPROM_Buffer[PATH2_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[PATH2_NAME+i+1]=0;  }
// EEPROM_UPDATA(); return 1;
// }
// 
// ================================================= //Set the number of TXpath1 times 0-9 0=off
// if (CHECK_AT_CMD("AT+PATH2_COUNT=")==1)
// {	len=strlen(UARTx_BUF)-15-2;
// if (len==1)	{  	EEPROM_Buffer[PATH2_COUNT]=(UARTx_BUF[15]-0x30);EEPROM_UPDATA();  	return 1;	  }
// return 0;
// }
// ================================================= //Set the name of relay 1
// if (CHECK_AT_CMD("AT+DIGI1_NAME=")==1)
// {
// len=strlen(UARTx_BUF)-14-2;
// if (len&gt;6) { return 0;} //Relay name is too long, error
// if (len==0) { return 0;} //Relay name not set, error
// for(i=0;i<len;i++) 	{ EEPROM_Buffer[DIGI1_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[DIGI1_NAME+i+1]=0;  }
// EEPROM_UPDATA(); return 1;
// }
// ================================================= //Set the name of relay 2
// if (CHECK_AT_CMD("AT+DIGI2_NAME=")==1)
// {
// len=strlen(UARTx_BUF)-14-2;
// if (len&gt;6) { return 0;} //Relay name is too long, error
// if (len==0) { return 0;} //Relay name not set, error
// for(i=0;i<len;i++) 	{ EEPROM_Buffer[DIGI2_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[DIGI2_NAME+i+1]=0;  }
// EEPROM_UPDATA(); return 1;
// }
// //==============================================
// if (CHECK_AT_CMD("AT+DIGI1=ON")==1){ EEPROM_Buffer[DIGI1_EN]=1;	EEPROM_UPDATA(); return 1;}
// if (CHECK_AT_CMD("AT+DIGI1=OFF")==1){ EEPROM_Buffer[DIGI1_EN]=0;	EEPROM_UPDATA(); return 1;}
// if (CHECK_AT_CMD("AT+DIGI2=ON")==1){ EEPROM_Buffer[DIGI2_EN]=1;	EEPROM_UPDATA(); return 1;}
// if (CHECK_AT_CMD("AT+DIGI2=OFF")==1){ EEPROM_Buffer[DIGI2_EN]=0;	EEPROM_UPDATA(); return 1;}
// //==============================================
// 
// if (CHECK_AT_CMD("AT+TEST=ON")==1){  BEACON_GPS_TXD(); return 2;}
// ================================================ //Relay remote password
// if (CHECK_AT_CMD("AT+CODE=")==1)
// {
// len=strlen(UARTx_BUF)-8-2;
// if (len!=6) { return 0;} //Relay ON password must be 6 digits
// for(i=0;i<len;i++) 	{ EEPROM_Buffer[0X01D0+i]= UARTx_BUF[8+i]; EEPROM_Buffer[0X01D0+i+1]=0;  }
// EEPROM_UPDATA(); return 1;
// 
// }
// 
// if (CHECK_AT_CMD("AT+OFFSN=")==1)
// {
// len=strlen(UARTx_BUF)-9-2;
// if (len!=6) { return 0;} //The relay OFF password must be 6 digits
// for(i=0;i<len;i++) 	{ EEPROM_Buffer[0X01D8+i]= UARTx_BUF[9+i]; EEPROM_Buffer[0X01D8+i+1]=0;  }
// EEPROM_UPDATA(); return 1;
// }
// 
// return 0;
// }
// 
// 



/*
// Gateway radio RF beacon, KISS format, excluding C0 00 ..C0, excluding checksum, adding end character 0x00 at the end, end character 0X00 is not sent
unsigned char code Gate_beacen[]=
{ 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, // target address + SSID
	 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF2, // Source address + SSID
	 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, // Path + SSID
	 0x03, 0xF0, 											// 03f0
	 0x21, // Type
	 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, // longitude and latitude
	 0x2F, // separator &quot;/&quot;
	 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, // longitude and latitude
	 0x3E, // Icon type
	 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74, // information
	 0x00, }; // The compiler will not add 00H to the end of the non-text string, you need to add it manually, and the end character 0X00 is not sent



	82 A0 88 A4 62 64 E0
	84 90 68 A8 88 AND 6A
	YES 92 88 8A 62 40 63
	03 F0
	3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0

	WIDE1-2  WIDE2-1
	C0 00
	82 A0 88 A4 62 64 E0
	84 90 68 A8 88 AND 6A
	YES 92 88 8A 62 40 64
	YES 92 88 8A 64 40 63
	03 F0
	3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E

	6F 72 67 2F C0


	BH4TDV-9  WIDE1

C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


BH4TDV-5  WIDE1

C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 63 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0


WIDE1-1
C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


WIDE1-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 65 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0



WIDE1-2  WIDE2-1
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 63 03 F0 3D 33 31

32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E

6F 72 67 2F C0



WIDE1-2  WIDE2-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 65
03 F0 3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64

72 6F 69 64 2E 6F 72 67 2F C0
BH4TDV-9  WIDE1

C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


BH4TDV-5  WIDE1

C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 63 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0


WIDE1-1
C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


WIDE1-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 65 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0



WIDE1-2  WIDE2-1
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 63 03 F0 3D 33 31

32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E

6F 72 67 2F C0



WIDE1-2  WIDE2-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 65
03 F0 3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64

72 6F 69 64 2E 6F 72 67 2F C0

*/




