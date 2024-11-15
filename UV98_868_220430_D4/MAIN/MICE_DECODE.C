#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "UART1.H"
#include "tostring.H"

#include "GPS2.H"
#include "GPS_JULI.H"

// #include "BEACON.H"

#include "KISS_Analysis.H"
#include "MICE_DECODE.H"

#include "PUBLIC_BUF.H"



/********* KISS data conversion ASCII UI format***********/

/* Convert the KISS data received by the radio to ASCII UI format*/

uchar MICE_IDX;	// Compressed data longitude actual location index


uchar KISS_MICE_STU;  // Received data 0 = uncompressed format 1 = compressed format


// uchar MICE_CHECK1()//Check whether the KISS data is compressed data,
// {
// uint i; flying tempo;
// 
// for(i=0;i<KISS_LEN;i++)
// {
// if ((KISS_DATA[i]==0x03)&&(KISS_DATA[i+1]==0xF0))
// {
// temp=	KISS_DATA[i+2];
// 
// if (temp==0x27){MICE_IDX=i+2;	 	return i+2;}
// if (temp==0x60){MICE_IDX=i+2;	 	return i+2;}
// return 0;
// }
// }
// return 0;
// }

// Check if the KISS data is compressed data,
// Return 0 = the path is too long or not compressed data
// Returns&gt;0, the starting index of the compressed data in the KISS data
uchar  MICE_CHECK()
{
    uchar idx, count;

    idx = 6;
    count = 0;

    while ((KISS_DATA[idx] & 0x01) != 1)	// Retrieve the end-of-path symbol
    {
        idx += 7;
        count++;		  // Each path field has 7 bytes

        if (count > 8)
        {
            return 0;   // Path too long, error
        }
    }

    idx++;
    idx++;
    idx++;	  // Skip 03 F0

    // Is it compressed data? //Record the starting position index of the longitude compressed data
    if ((KISS_DATA[idx] == 0x27))
    {
        MICE_IDX = idx;
        return idx;
    }

    if ((KISS_DATA[idx] == 0x60))
    {
        MICE_IDX = idx;
        return idx;
    }

    return 0;
}

uchar DECODE_WD()  // Decompress latitude
{
    uchar i;
    uchar MSG_BIT, CUSTOM_BIT; // Message Type ABC
    uchar long_offset;
    uchar TEMP_DATA[7];

    MSG_BIT = CUSTOM_BIT = 0;

    for (i = 0; i < 6; i++)
    {
        TEMP_DATA[i] = KISS_DATA[i] >> 1;
    }

    // ====================================================== BIT1
    if ((TEMP_DATA[0] >= 'P') && (TEMP_DATA[0] <= 'Y'))
    {
        UI_WD[0] = TEMP_DATA[0] - 32;
        MSG_BIT   |= 0X40;
    }

    if ((TEMP_DATA[0] >= 'A') && (TEMP_DATA[0] <= 'J'))
    {
        UI_WD[0] = TEMP_DATA[0] - 17;
        CUSTOM_BIT |= 0X40;
    }

    if (TEMP_DATA[0] == 'K')
    {
        UI_WD[0] = ' ';
        CUSTOM_BIT |= 0X40;
    }

    if (TEMP_DATA[0] == 'Z')
    {
        UI_WD[0] = ' ';
        MSG_BIT   |= 0X40;
    }

    if (TEMP_DATA[0] == 'L')
    {
        UI_WD[0] = ' ';
    }

    if (TEMP_DATA[0] < ':')
    {
        UI_WD[0] = TEMP_DATA[0];
    }

    // ==============================================BIT2

    if ((TEMP_DATA[1] >= 'P') && (TEMP_DATA[1] <= 'Y'))
    {
        UI_WD[1] = TEMP_DATA[1] - 32;
        MSG_BIT   |= 0X40;
    }

    if ((TEMP_DATA[1] >= 'A') && (TEMP_DATA[1] <= 'J'))
    {
        UI_WD[1] = TEMP_DATA[1] - 17;
        CUSTOM_BIT |= 0X40;
    }

    if (TEMP_DATA[1] == 'K')
    {
        UI_WD[1] = ' ';
        CUSTOM_BIT |= 0X40;
    }

    if (TEMP_DATA[1] == 'Z')
    {
        UI_WD[1] = ' ';
        MSG_BIT   |= 0X40;
    }

    if (TEMP_DATA[1] == 'L')
    {
        UI_WD[1] = ' ';
    }

    if (TEMP_DATA[1] < ':')
    {
        UI_WD[1] = TEMP_DATA[1];
    }

    // ==================================================BIT3
    if ((TEMP_DATA[2] >= 'P') && (TEMP_DATA[2] <= 'Y'))
    {
        UI_WD[2] = TEMP_DATA[2] - 32;
        MSG_BIT   |= 0X40;
    }

    if ((TEMP_DATA[2] >= 'A') && (TEMP_DATA[2] <= 'J'))
    {
        UI_WD[2] = TEMP_DATA[2] - 17;
        CUSTOM_BIT |= 0X40;
    }

    if (TEMP_DATA[2] == 'K')
    {
        UI_WD[2] = ' ';
        CUSTOM_BIT |= 0X40;
    }

    if (TEMP_DATA[2] == 'Z')
    {
        UI_WD[2] = ' ';
        MSG_BIT   |= 0X40;
    }

    if (TEMP_DATA[2] == 'L')
    {
        UI_WD[2] = ' ';
    }

    if (TEMP_DATA[2] < ':')
    {
        UI_WD[2] = TEMP_DATA[2];
    }

    // ==============================================BIT4
    if (TEMP_DATA[3] >= 'P')
    {
        UI_WD[3] = TEMP_DATA[3] - 32;     // Latitude NS
        UI_WD_DIR = 'N';
    }

    if (TEMP_DATA[3] == 'Z')
    {
        UI_WD[3] = ' ';
        UI_WD_DIR = 'N';
    }

    if (TEMP_DATA[3] == 'L')
    {
        UI_WD[3] = ' ';
        UI_WD_DIR = 'S';
    }

    if (TEMP_DATA[3] < ':')
    {
        UI_WD[3] = TEMP_DATA[3];
        UI_WD_DIR = 'S';
    }

    // ==============================================
    UI_WD[4] = '.';

    // ==================================================BIT5
    if (TEMP_DATA[4] >= 'P')
    {
        UI_WD[5] = TEMP_DATA[4] - 32;     // Longitude +100
        long_offset = 100;
    }

    if (TEMP_DATA[4] == 'Z')
    {
        UI_WD[5] = ' ';
        long_offset = 100;
    }

    if (TEMP_DATA[4] == 'L')
    {
        UI_WD[5] = ' ';
        long_offset = 0;
    }

    if (TEMP_DATA[4] < ':')
    {
        UI_WD[5] = TEMP_DATA[4];
        long_offset = 0;
    }

    // ==============================================BIT6
    if (TEMP_DATA[5] >= 'P')
    {
        UI_WD[6] = TEMP_DATA[5] - 32;     // LongitudeWE
        UI_JD_DIR = 'W';
    }

    if (TEMP_DATA[5] == 'Z')
    {
        UI_WD[6] = ' ';
        UI_JD_DIR = 'W';
    }

    if (TEMP_DATA[5] == 'L')
    {
        UI_WD[6] = ' ';
        UI_JD_DIR = 'E';
    }

    if (TEMP_DATA[5] < ':')
    {
        UI_WD[6] = TEMP_DATA[5];
        UI_JD_DIR = 'E';
    }

    // ==============================================


    UI_WD[7] = UI_WD_DIR; 	// Latitude analysis completed

    UI_WD[8] = 0x00; 	// Latitude analysis completed
    // ==============================================
    return long_offset;

// if (MSG_BIT==7) {uart1_sendstring("M0:OFF DUTY");}
// if (MSG_BIT==6) {uart1_sendstring(&quot;M1:En Route&quot;);}
// if (MSG_BIT==5) {uart1_sendstring("M2:In Service");}
// if (MSG_BIT==4) {uart1_sendstring("M3:Returning");}
// if (MSG_BIT==3) {uart1_sendstring("M4:Committed");}
// if (MSG_BIT==2) {uart1_sendstring("M5:Special");}
// if (MSG_BIT==1) {uart1_sendstring("M6:Priority");}
// if (MSG_BIT==0) {uart1_sendstring("Emergency");}

// if (CUSTOM_BIT==7) {uart1_sendstring("C0:Custom-0");}
// if (CUSTOM_BIT==6) {uart1_sendstring("C1:Custom-1");}
// if (CUSTOM_BIT==5) {uart1_sendstring("C2:Custom-2");}
// if (CUSTOM_BIT==4) {uart1_sendstring("C3:Custom-3");}
// if (CUSTOM_BIT==3) {uart1_sendstring("C4:Custom-4");}
// if (CUSTOM_BIT==2) {uart1_sendstring("C5:Custom-5");}
// if (MICE_MSG_ABC==1) {uart1_sendstring("C6:Custom-6");}
// if (CUSTOM_BIT==0) {uart1_sendstring("Emergency");}
}

uchar KISS_TO_MICE( )
{
    unsigned char i, k, ssid;
    unsigned char temp;

    uchar long_offset;
    uint bit4, bit5, bit6, bit10, bit11, bit12;
    uchar alt_stu;
    long s;

// KISS_MICE_STU; //Received data 0 = uncompressed format 1 = compressed format

    if (MICE_CHECK() == 0)
    {
        KISS_MICE_STU = 0;
        return 0;
    }



    // fly DEST_BYTE[7]={&quot;S32U6T&quot;}; 3325.64 N W
    // float DEST_BYTE[7]={&quot;C8LZZL&quot;};

    long_offset = DECODE_WD(); // Decompress latitude
    // ==============================================
    // BH1PEZ-7>SYUUQ1,WIDE1-1,WIDE2-1:`,<|l+Z[/`"4D}Welcome to QSO with me!._(
    temp = KISS_DATA[MICE_IDX + 1] - 28 ; // The resolution range of longitude is 0-179 degrees

    if (long_offset == 100)
    {
        temp += 100;
    }

    if ((temp >= 180) && (temp <= 189))
    {
        temp -= 80;
    }

    if ((temp >= 190) && (temp <= 199))
    {
        temp -= 190;
    }

    UI_JD[0] = temp / 100 + 0X30;
    UI_JD[1] = temp / 10 % 10 + 0X30;
    UI_JD[2] = temp % 10 + 0X30;
    // ==============================================
    // Encoding process Long_Min+=28; if (Long_Min&lt;38){Long_Min+=60;}
    temp = KISS_DATA[MICE_IDX + 2] - 28 ; // Parsing longitude 0-59

    if (temp >= 60)
    {
        temp -= 60;
    }

    UI_JD[3] = temp / 10 + 0X30;
    UI_JD[4] = temp % 10 + 0X30;
    // ==============================================
    UI_JD[5] = '.';
    // ==============================================
    temp = KISS_DATA[MICE_IDX + 3] - 28 ; // Parsing longitude decimals 0-99
    UI_JD[6] = temp / 10 + 0X30;
    UI_JD[7] = temp % 10 + 0X30;

    UI_JD[8] = UI_JD_DIR;
    UI_JD[9] = 0x00;
    // --------------------------------------------
    // 60 30 2F 74 1C 29 20 3E 2F 22 34 59 7D
    // 304 2KM

    bit4 = KISS_DATA[MICE_IDX + 4];	 // 4th Byte Speed
    bit5 = KISS_DATA[MICE_IDX + 5];	 // 5th Byte Speed

    s = (bit4 - 28) * 10 + (bit5 - 28) / 10;	 // (4th byte - 28)*10 + (5th byte - 28)/10

    if (s >= 800)
    {
        s -= 800;
    }

    s = s * 1.852 * 10;	 // Convert to Metric

    tostring(s);
    UI_SPEED[0] = wan;
    UI_SPEED[1] = qian;
    UI_SPEED[2] = bai;
    UI_SPEED[3] = shi;
    UI_SPEED[4] = '.';
    UI_SPEED[5] = ge;
    UI_SPEED[6] = 0x00;

// k=0;
// while(UI_SPEED[0]==&#39;0&#39;) //The first 0 is blanked, at least one bit is retained
// {
// for(i=0;i<7;i++) 	{ UI_SPEED[i]=UI_SPEED[i+1]; }
// k++; if (k>2){break;}
// }

// --------------------------------------------
// (5th byte - 28)%10*100 + (6th byte - 28)

    bit5 = KISS_DATA[MICE_IDX + 5];	 // The 5th and 6th bytes are the heading
    bit6 = KISS_DATA[MICE_IDX + 6];

    s = (bit5 - 28) % 10 * 100 + (bit6 - 28)   ;	 // 

    if (s >= 400)
    {
        s -= 400;
    }

    tostring(s);
    UI_DIR[0] = bai;
    UI_DIR[1] = shi;
    UI_DIR[2] = ge;
    UI_DIR[3] = 0x00;
// --------------------------------------------
// BI3NFU-7&gt;SYUYQ0,WIDE1-1,WIDE2-1:`.F8l!f[/`&quot;4W}144.640_$
// SYUYQ0 =3959.10N E

// 22 36 72 7D 273m &quot;6r

// 22 35 44 136m &quot;5D

// The Mic-E status text field can contain the station's altitude. The altitude is
// expressed in the form xxx},
// where xxx is in meters relative to 10km below
// 
// mean sea level (the deepest ocean), to base 91.
// For example, to compute the xxx characters for an altitude of 200 feet:

    alt_stu = 0;

// GPS_ALT_MI=-20; //22 33 5E

    if (KISS_DATA[MICE_IDX + 12] == '}')		 // Compress altitude data, the unit is directly in meters, no conversion is required
    {
        bit10 = KISS_DATA[MICE_IDX + 9];
        bit11 = KISS_DATA[MICE_IDX + 10];	 // 
        bit12 = KISS_DATA[MICE_IDX + 11];	 // 



        s = (((long)bit10 - 33) * 8281 + ((long)bit11 - 33) * 91 + ((long)bit12 - 33)) - 10000 ; // % 400 ;
        alt_stu = 1;
    }

    if (KISS_DATA[MICE_IDX + 13] == '}')		 // Compress altitude data, the unit is directly in meters, no conversion is required
    {
        bit10 = KISS_DATA[MICE_IDX + 10];
        bit11 = KISS_DATA[MICE_IDX + 11];	 // 
        bit12 = KISS_DATA[MICE_IDX + 12];	 // 

        s = (((long)bit10 - 33) * 8281 + ((long)bit11 - 33) * 91 + ((long)bit12 - 33)) - 10000 ; // % 400 ;
        alt_stu = 1;
    }


    if (alt_stu == 1)
    {
        UI_ALT_MI = s;

        if (s < 0)
        {
            s = -s;
            tostring(s);
            UI_ALT[0] = '-';
            UI_ALT[1] = qian;
            UI_ALT[2] = bai;
            UI_ALT[3] = shi;
            UI_ALT[4] = ge;
            UI_ALT[5] = 0x00;
        }
        else
        {
            tostring(s);
            UI_ALT[0] = wan;
            UI_ALT[1] = qian;
            UI_ALT[2] = bai;
            UI_ALT[3] = shi;
            UI_ALT[4] = ge;
            UI_ALT[5] = 0x00;
        }

// k=0;
// while(UI_ALT[0]==&#39;0&#39;) //The first 0 is blanked, at least one bit is retained
// {
// for(i=0;i<6;i++) 	{ UI_ALT[i]=UI_ALT[i+1]; }
// k++; if (k>3){break;}
// }
    }
    else
    {
        UI_ALT_MI = 0;
    }

// (10th position - &amp;H21)*&amp;H2059+(11th position - &amp;H21)*&amp;H5B+(12th position - &amp;H21)*&amp;H2710
// --------------------------------------------
    UI_ICON = KISS_DATA[MICE_IDX + 7];
// --------------------------------------------

// C0 00
// A4 AC AA A8 A4 62 60
// 84 8E 6E 84 AC 92 E3
// 03 F0 60 28 3E 2E 6C 21 31 5B 2F 60 22 36 35 7D 5F 23 0D C0

// temp=KISS_DATA[MICE_IDX+4]-28; //Analysis speed 0-99
// UI_JD[6]=temp/10+0X30;		UI_JD[7]=temp%10+0X30;
// UI_JD[8]=MICE_WE;
// UI_WD[9]=0x00;
// --------------------------------------------Analysis of call sign and ID


    k = 0;

    for (i = 7; i < 13; i++)    				// Leading space blanking
    {
        temp = ((KISS_DATA[i])  >> 1);

        if (temp != ' ')
        {
            UI_CALL[k] = temp;
            k++;
        }
    }

    ssid =	 ((KISS_DATA[13] & 0x1E) >> 1) ;

    if (ssid > 0)
    {
        UI_CALL[k] = '-';
        k++;

        if (ssid < 10)
        {
            ssid |= 0x30;    // Convert to ASCII
            UI_CALL[k] = ssid;
            k++;
        }
        else
        {
            UI_CALL[k] = ssid / 10 + 0x30;
            k++;
            UI_CALL[k] = ssid % 10 + 0x30;
            k++;
        }
    }

    UI_CALL[k] = 0x00;

// --------------------------------------------Resolving SSID
    KISS_MICE_STU = 1;
    return 1; 	  // 1
}



