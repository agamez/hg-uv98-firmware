#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "GPS2.H"
#include "MICE_CODE.H"


// uchar TEMP_WD[10]; //Latitude before compression
// uchar TEMP_JD[10]; //Longitude before compression
// GPS_ALT_MI=10061; //Altitude data before compression, the unit is directly meter
// GPS_NOW_SPEED; //Speed before compression, in nautical miles
// GPS_NOW_DIR; //Heading before compression 0-359


uchar long_offset;
uchar MICE_WD[10];	 // Compressed latitude
uchar MICE_JD[20];	 // Compressed longitude, speed, heading, altitude


// BH4TDV-10>APET51:!3134.31N/12020.22E>000/000/A=000027	  //14+22  22x833x8	=147ms
// BH4TDV-10&gt;SYUUQ1:`,&lt;|l+Z[/`&quot;4D} //14 bytes



// Compressed Latitude
void Encoding_MICE_LAT(uchar MSG_TYPE)
{
    uchar MSG_BIT_A, MSG_BIT_B, MSG_BIT_C;

    // MSG_TYPE=3;
    // ==============================================
    switch (MSG_TYPE)
    {
        case 0:
            MSG_BIT_A = 1;
            MSG_BIT_B = 1;
            MSG_BIT_C = 1;
            break;

        case 1:
            MSG_BIT_A = 1;
            MSG_BIT_B = 1;
            MSG_BIT_C = 0;
            break;

        case 2:
            MSG_BIT_A = 1;
            MSG_BIT_B = 0;
            MSG_BIT_C = 1;
            break;

        case 3:
            MSG_BIT_A = 1;
            MSG_BIT_B = 0;
            MSG_BIT_C = 0;
            break;

        case 4:
            MSG_BIT_A = 0;
            MSG_BIT_B = 1;
            MSG_BIT_C = 1;
            break;

        case 5:
            MSG_BIT_A = 0;
            MSG_BIT_B = 1;
            MSG_BIT_C = 0;
            break;

        case 6:
            MSG_BIT_A = 0;
            MSG_BIT_B = 0;
            MSG_BIT_C = 1;
            break;

        default:
            MSG_BIT_A = 0;
            MSG_BIT_B = 0;
            MSG_BIT_C = 0;
            break;
    }

    // ====================================================== BIT1
    if (MSG_BIT_A == 1)
    {
        if (GPS_WD[0] == ' ')
        {
            MICE_WD[0] = 'Z';   // 0-9 =P-Y  SPACE=Z
        }
        else
        {
            MICE_WD[0] = GPS_WD[0] + 32 ;
        }
    }
    else
    {
        MICE_WD[0] =  GPS_WD[0];
    }

    // ==============================================BIT2
    if (MSG_BIT_B == 1)
    {
        if (GPS_WD[1] == ' ')
        {
            MICE_WD[1] = 'Z';   // 0-9 =P-Y  SPACE=Z
        }
        else
        {
            MICE_WD[1] = GPS_WD[1] + 32 ;
        }
    }
    else
    {
        MICE_WD[1] =  GPS_WD[1];
    }

    // ==================================================BIT3
    if (MSG_BIT_C == 1)
    {
        if (GPS_WD[2] == ' ')
        {
            MICE_WD[2] = 'Z';   // 0-9 =P-Y  SPACE=Z
        }
        else
        {
            MICE_WD[2] = GPS_WD[2] + 32 ;
        }
    }
    else
    {
        MICE_WD[2] =  GPS_WD[2];
    }

    // ==============================================BIT4
    if (GPS_WD[7] == 'N')
    {
        if (GPS_WD[3] == ' ')
        {
            MICE_WD[3] = 'Z';   // 0-9 =P-Y  SPACE=Z
        }
        else
        {
            MICE_WD[3] = GPS_WD[3] + 32 ;
        }
    }
    else
    {
        if (GPS_WD[3] == ' ')
        {
            MICE_WD[3] = 'L';
        }
        else
        {
            MICE_WD[3] =  GPS_WD[3];
        }
    }

    // ==================================================BIT5
    if (long_offset == 100)	 // If longitude +100 degrees
    {
        if (GPS_WD[5] == ' ')
        {
            MICE_WD[4] = 'Z';   // 0-9 =P-Y  SPACE=Z
        }
        else
        {
            MICE_WD[4] = GPS_WD[5] + 32 ;
        }
    }
    else
    {
        if (GPS_WD[5] == ' ')
        {
            MICE_WD[4] = 'L';
        }
        else
        {
            MICE_WD[4] =  GPS_WD[5];
        }
    }

    // ==============================================BIT6
    if (GPS_JD[8] == 'W')	// 12020.00E
    {
        if (GPS_WD[6] == ' ')
        {
            MICE_WD[5] = 'Z';   // 0-9 =P-Y  SPACE=Z
        }
        else
        {
            MICE_WD[5] = GPS_WD[6] + 32 ;
        }
    }
    else
    {
        if (GPS_WD[6] == ' ')
        {
            MICE_WD[5] = 'L';
        }
        else
        {
            MICE_WD[5] =  GPS_WD[6];
        }
    }

}


// Degrees of compressed longitude
void Encoding_MICE_LONG_DEGREE()
{
    uchar lng;
    MICE_JD[0] = '`'; // Compressed Data Marker
    // ==============================================d+28 0-179
    lng = (GPS_JD[0] - 0X30) * 100 + (GPS_JD[1] - 0X30) * 10 + (GPS_JD[2] - 0X30);

    // 0-9=118-127
    if (lng < 10)
    {
        MICE_JD[1] = 118 + lng;
        long_offset = 100;
        return;
    }

    // 10-99=38-127
    if (lng < 100)
    {
        MICE_JD[1] = 28 + lng;
        long_offset = 0;
        return;
    }

    // 100-109=108-117
    if (lng < 110)
    {
        MICE_JD[1] = 8 + lng;
        long_offset = 100;
        return;
    }

    // 110-179=38-107
    if (lng < 180)
    {
        MICE_JD[1] = lng - 72;
        long_offset = 100;
        return;
    }
}

// Compressed longitude, speed, heading 9 bytes, altitude 5 bytes
void Encoding_MICE()
{
    uchar min;
    uchar sp, dir;
    long alt, a, b, c, d;

    Encoding_MICE_LONG_DEGREE();	 // Degrees of compressed longitude
    // ==============================================
    // The first two decimal places of the compressed fraction are 0-59
    min = (GPS_JD[3] - 0X30) * 10 + (GPS_JD[4] - 0X30);
    // 0-9=88-97	 	//10-59=38-87

    MICE_JD[2] = 28 + min;

    if(MICE_JD[2] < 38)
    {
        MICE_JD[2] += 60;
    }

// if (min<10){MICE_JD[2]=88+min;}	else	{  MICE_JD[2]=28+min; }

    // ==============================================
    // 2 decimal places after compression point 0-99
    min = (GPS_JD[6] - 0X30) * 10 + (GPS_JD[7] - 0X30);
    MICE_JD[3] = 28 + min;

    sp = GPS_NOW_SPEED_KNOT / 10; // The speed is coded in hundreds and tens digits, in 10-nautical-mile sections, and less than 200 nautical miles. There are two types of coding, both of which can be used.

    if(	sp < 20)
    {
        MICE_JD[4] = sp + 28;
    }
    else
    {
        MICE_JD[4] = sp + 28;
    }

// if(	sp <20)	{MICE_JD[4]=sp+108;}else  {MICE_JD[4]=sp+28;}
    // ==============================================
    // Speed unit code + heading hundred digit code
    // Speed unit digit code, 0-9 nautical miles, there are 2 codes, both are available
    // Heading (0-360)/100, 100 degree section
    sp = GPS_NOW_SPEED_KNOT % 10;  	// Assume the speed is 3
    dir = GPS_NOW_DIR / 100;	// Assuming heading 294 degrees
    MICE_JD[5] = dir + 28 + (sp * 10);	 // 60
    // MICE_JD[5]=dir+32+(sp*10);
    // ==============================================
    // Heading ten-digit unit-digit code, (0-360)%100,0-99
    dir = GPS_NOW_DIR % 100; 	// 94
    MICE_JD[6] = dir + 28;		// 122

    // Icons Icon Sets
// MICE_JD[7]=EEPROM_Buffer[0X14] ;	MICE_JD[8]=EEPROM_Buffer[0X13] ;
    if (SMART_MODE == 2)
    {
        MICE_JD[7] = EEPROM_Buffer[0XBD];
        MICE_JD[8] = EEPROM_Buffer[0XBC];
    }
    else
    {
        MICE_JD[7] = EEPROM_Buffer[0X14] ;
        MICE_JD[8] = EEPROM_Buffer[0X13] ;
    }

// MICE_JD[7]=EEPROM_Buffer[0X14] ;	MICE_JD[8]=EEPROM_Buffer[0X13] ;
    // ==============================================
    // Compress altitude data, the unit is directly in meters
// GPS_ALT_MI=-20; //22 33 5E
    alt = GPS_ALT_MI + 10000;
    a = alt / 8281;
    b = alt % 8281;
    c = b / 91;
    d = b % 91;
    MICE_JD[9] = '`' ;		// 
    MICE_JD[10] = ((uchar)a) + 33;
    MICE_JD[11] = ((uchar)c) + 33;
    MICE_JD[12] = ((uchar)d) + 33;
    MICE_JD[13] = '}' ;		// 
    // ==============================================
    // Compressed Latitude
// Encoding_MICE_LAT(3);

    Encoding_MICE_LAT(EEPROM_Buffer[0X11]);

}







