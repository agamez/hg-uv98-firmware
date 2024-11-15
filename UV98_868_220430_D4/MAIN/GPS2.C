
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "IO.H"
// #include "DELAY.H"
// #include "ADC.H"
#include "tostring.H"

#include "GPS2.H"
#include "GPS_CHECKSUM.H"
#include "GPS_JULI.H"

#include "BEACON.H"
#include "UART2.H"
#include "UART3.H"
#include "UART4.H"

#include "AT24C512.h"
#include "PUBLIC_BUF.H"
#include "DISP_GPS.H"

#include<string.h>



uchar GPS_TIME[10] = 0;	// time
uint AUTO_TIME;



uchar GPS_DATE[13] = 0;	// date

uchar GPS_WD[15] = 0;	// latitude
uchar GPS_JD[15] = 0; // longitude

bit GPS_LOCKED;

uint GPS_NOW_DIR;		// GPS current direction 0-359 degrees
uchar GPS_NOW_DIR_TXT[4];



uint GPS_OLD_DIR = 0;		// GPS last direction 0-359 degrees
bit GPS_DIR_NOTHING;	// 0 = stationary, no heading data 1 = moving, heading data available

uint GPS_NOW_SPEED_KNOT = 0;	// The current speed extracted from GPS is 0-255 nautical miles. 1 nautical mile = 1.852 kilometers.
uchar GPS_NOW_SPEED_KNOT_TXT[10];

uint GPS_SPEED_KM = 0;	// GPS speed in kilometers
// uchar GPS_SPEED_KM_TXT[10];


uchar  GPS_ALT_ERR;		// GPS has no valid positioning, 1 = no altitude data, then the data is invalid 0 = altitude data is normal

uchar GPS_HEIGHT[7] = 0;	// Altitude
long GPS_ALT_MI;

uchar GPS_SIG[3] = 0; // Number of satellites located 00-12

uint GPS_WAIT_TIME;   // (0-255)*10 seconds





// uchar LC_TIME; //Mileage cycle AZ
uchar LAST_WD[10] = 0;	// Last Latitude
uchar LAST_JD[10] = 0; // Last longitude


uchar TIME_GPS_SEC;
uchar SMART_MODE;



bit QUICK_SET_EN;  // 0 = Disable fast station setup 1 = Enable fast station setup

// uchar GET_LC();

// mileage
uchar lc_start;  // 0 = First launch, mileage accumulation reset to 0 1 = Not the first time, mileage accumulation continues
unsigned  long  TOTAL_LC, AB_LC;   // 0-6500.0km unsigned long four bytes //0～4294967295




// 0xFE00 Last 512 storage setting data
void LC_MEMORY()  // Record Mileage
{
    if (EEPROM_Buffer[0x00BB] == 0)
    {
        return;
    }

    // Accumulated mileage memory 0=OFF 1=ON

// EEPROM_Buffer[0x00B7]=	 (uchar)(TOTAL_LC/256/256/256);
// EEPROM_Buffer[0x00B8]=	 (uchar)(TOTAL_LC/256/256);
// EEPROM_Buffer[0x00B9]=	 (uchar)(TOTAL_LC/(256));
// EEPROM_Buffer[0x00Ba]=	 (uchar)(TOTAL_LC%(256));
// 
// EEPROM_UPDATA();

    AT24CXX_WRITE(0x0FE00, (uchar)(TOTAL_LC / 256 / 256 / 256)) ;
    AT24CXX_WRITE(0x0FE01, (uchar)(TOTAL_LC / 256 / 256)) ;
    AT24CXX_WRITE(0x0FE02, (uchar)(TOTAL_LC / (256)));
    AT24CXX_WRITE(0x0FE03, (uchar)(TOTAL_LC % (256))) ;

}


void LC_CLEAN()  // Mileage reset to 0
{
    TOTAL_LC = 0;
    AT24CXX_WRITE(0x0FE00, 0 );
    AT24CXX_WRITE(0x0FE01, 0) ;
    AT24CXX_WRITE(0x0FE02, 0);
    AT24CXX_WRITE(0x0FE03, 0) ;
}




uchar GET_LC()
{
    uchar i;

    if (lc_start == 0)
    {
        lc_start = 1; // Start counting mileage

        AB_LC = 0;	 // The first mileage will be reset to 0
    }
    else
    {
        AB_LC = GET_JULI(LAST_WD, LAST_JD, GPS_WD, GPS_JD);	 // Calculate mileage
        TOTAL_LC = TOTAL_LC + AB_LC;	// Mileage Accumulation

        // When the mileage exceeds 5000 km, the mileage is reset to 0 and the mileage unit increases by 1 (letters AZ, az)
        if (TOTAL_LC > 500000000)
        {
            TOTAL_LC = 0;   // More than 500,000 kilometers, clear to 0
        }

        LC_MEMORY();
    }

// UART1_SendString(&quot;============\r\n&quot;);
// UART1_DEBUG(GPS_NOW_SPEED_KNOT); UART1_SendString("\r\n");
// 
// UART1_SendString(LAST_WD); UART1_SendString(LAST_JD); UART1_SendString(&quot;\r\n&quot;);
// UART1_SendString(GPS_WD); UART1_SendString(GPS_JD); UART1_SendString(&quot;\r\n&quot;);
// UART1_DEBUG((uint)(AB_LC)); UART1_SendString(&quot; &quot;);
// UART1_DEBUG((uint)(TOTAL_LC));


    for (i = 0; i < 9; i++)
    {
        LAST_WD[i] = GPS_WD[i];     // Update latitude position
    }

    for (i = 0; i < 10; i++)
    {
        LAST_JD[i] = GPS_JD[i];     // Update longitude position
    }


    return 1;
}


void INIT_LC()	  // Mileage initialization
{
    lc_start = 0;	 // Stop counting mileage

    if (EEPROM_Buffer[0x00BB] == 0)
    {
        TOTAL_LC = 0;   // Accumulated mileage memory 0=OFF 1=ON
    }
    else
    {
        // Reading out the mileage of memory
// TOTAL_LC=(long)(EEPROM_Buffer[0x00B7])*256*256*256+(long)(EEPROM_Buffer[0x00B8])*256*256+(long)(EEPROM_Buffer[0x00B9])*256+(long)(EEPROM_Buffer[0x00Ba]);

// TOTAL_LC=(long)(AT24CXX_READ(0XFE00))*256*256*256+(long)(AT24CXX_READ(0XFE01))*256*256+(long)(AT24CXX_READ(0XFE02))*256+(long)(AT24CXX_READ(0XFE03));

        if (AT24CXX_READ(0XFE00) == 0xff)	 // 24CXX is not initialized, the total mileage is cleared to 0
        {
            LC_CLEAN();  // Mileage reset to 0
        }
        else
        {
            TOTAL_LC = (long)(AT24CXX_READ(0XFE00)) * 256 * 256 * 256 + (long)(AT24CXX_READ(0XFE01)) * 256 * 256 + (long)(AT24CXX_READ(0XFE02)) * 256 + (long)(AT24CXX_READ(0XFE03));
        }


    }

    AB_LC = 0;	 // The first mileage will be reset to 0
}









void GET_GPS_TIME(uchar *nbuffer)	   // Intercept GPS time
{
    uchar i, k, n;
// $GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 1)
        {
            break;
        }
    }

    if (nbuffer[i] == ',')
    {
        return;
    }

    k = 0;
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		// 
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		// 
    GPS_TIME[k] = ':';
    k++; 		// 

    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		// 
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		// 
    GPS_TIME[k] = ':';
    k++; 	// 

    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		// 
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		// 
    GPS_TIME[k] = 0X00;		// 

    // Convert to 24 hours


    if (EEPROM_Buffer[0x002C] < 13) // Time zone adjustment
    {
        n = ((GPS_TIME[0] - 0x30) * 10 + (GPS_TIME[1] - 0x30)) - (13 - EEPROM_Buffer[0x002C]);
    }
    else
    {
        n = ((GPS_TIME[0] - 0x30) * 10 + (GPS_TIME[1] - 0x30)) + (EEPROM_Buffer[0x002C] - 13);
    }

    if (n >= 24)
    {
        n -= 24;
    }

    AUTO_TIME = n;
    GPS_TIME[0] = n / 10 + 0x30;
    GPS_TIME[1] = n % 10 + 0x30;

    TIME_GPS_SEC =	(GPS_TIME[6] - 0x30) * 10 +	(GPS_TIME[7] - 0x30);
}

// uchar GPS_DATE[10]; //date

void GET_GPS_DATE(uchar *nbuffer)	   // Intercept GPS date
{
    uchar i, k;
    uchar temp[10];

    unsigned char Bhour = 0, Bday = 0, Bmonth = 0;
    unsigned int Byear = 0;


// for(i=0;i<12;i++)  {	GPS_DATE[i]=0;}

// $GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 9)
        {
            break;
        }
    }

    if (nbuffer[i] == ',')
    {
        return;
    }

    temp[0] = nbuffer[i];
    temp[1] = nbuffer[i + 1];	// day
    temp[2] = nbuffer[i + 2];
    temp[3] = nbuffer[i + 3];	// moon
    temp[4] = nbuffer[i + 4];
    temp[5] = nbuffer[i + 5];	// Year

    Bday = (temp[0] - 0x30) * 10 + (temp[1] - 0x30);
    Bmonth = (temp[2] - 0x30) * 10 + (temp[3] - 0x30);
    Byear = (temp[4] - 0x30) * 10 + (temp[5] - 0x30) + 2000;

    if (AUTO_TIME < 8) 		 // Adjust date
    {
        Bday++;	    // Add 1 to the date

        switch(Bday)		// Judgment date
        {
            case 29:	// February in an ordinary year
                if((!((Byear % 400 == 0) || ((Byear % 4 == 0) && (Byear % 100 != 0))) && (Bmonth == 2)))
                {
                    Bday = 1;
                    Bmonth++;
                }

                break;

            case 30:      // If it is February in a leap year
                if(((Byear % 400 == 0) || ((Byear % 4 == 0) && (Byear % 100 != 0))) && (Bmonth == 2))
                {
                    Bday = 1;
                    Bmonth++;
                }

                break;

            case 31:
                if((Bmonth == 4) || (Bmonth == 6) || (Bmonth == 9) || (Bmonth == 11))
                {
                    Bday = 1;
                    Bmonth++;
                }

                break;

            case 32:
                Bday = 1;
                Bmonth++;

                if(Bmonth >= 13)
                {
                    Byear++;
                    Bmonth = 1;
                }

                break;
        }
    }


// Byear=Byear-2000;

    k = 0;
    tostring(Byear);
    GPS_DATE[k++] = qian;
    GPS_DATE[k++] = bai;
    GPS_DATE[k++] = shi;
    GPS_DATE[k++] = ge;
    GPS_DATE[k++] = '-';
    tostring(Bmonth);
    GPS_DATE[k++] = shi;
    GPS_DATE[k++] = ge;
    GPS_DATE[k++] = '-';
    tostring(Bday);
    GPS_DATE[k++] = shi;
    GPS_DATE[k++] = ge;
    GPS_DATE[k++] = 0X00;
}




void GPS_INIT()
{
    if (EEPROM_Buffer[0x002B] == 1)
    {
        GPS_EN = 1;   // GPS on or off
    }
    else
    {
        GPS_EN = 0;
    }

    GPS_OLD_DIR = 0;	// Initial GPS previous angle = 0 degrees.
    GPS_NOW_DIR = 0;		 // Initial heading = 0
    GPS_LOCKED = 0;

    INIT_LC();	   // Mileage initialization
    TIME_GPS_SEC = 0;


    QUICK_SET_EN = 0; // Disable fast setup of fixed stations
}


uchar GET_GPS_LOCK_STATUS(uchar *nbuffer) // Check GPS data GPRMC, whether it is positioned, 0 = not positioned, 1 = positioned
{
    // Extract a row from a dataframe
    // $GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    uchar i, k;

    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 2)
        {
            break;   // Skip 2,
        }
    }

    // 1=GPS positioning 0=GPS not positioning GPS_LOCKED=1; return 1; }
    if  (nbuffer[i] == 'A')
    {
        GPS_LOCKED = 1;
        return 1;
    }

    GPS_LOCKED = 0;	 // LED_STU=!LED_STU;
    return 0;
}

uchar GET_GPS_LONG_LAT(uchar *nbuffer)	   // Intercept longitude and latitude and save to GPS_DATA
{
    uchar i, k, n; // float tempA,tempB;
    // Extract a row from a dataframe
    // $GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    // Data analysis and conversion processing


    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 3)
        {
            break;   // Skip 3,
        }
    }

// A,3134.0432,N,12020.2056,E,0.00,353.93,010213,,,A*64
// if (nbuffer[i]!=&#39;A&#39;) { return 0;} //05=GPS not positioned, no further analysis
// i++; i++; // 3rd one&#39;,&#39;
// while (nbuffer[i]!=',') { i++;  }

    // 3134.0377,N,12020.2048,E,0.00,198.43,010213,,,A*6B

// k=0;
// while (nbuffer[i]!=&#39;.&#39;) { GPS_WD[k]=nbuffer[i]; k++; i++; } //Insert latitude to 2 decimal places

// GPS_WD[k]=nbuffer[i]; k++; i++; //Insert decimal point
// GPS_WD[k]=nbuffer[i]; k++; i++; //Insert 2 decimal places
// GPS_WD[k]=nbuffer[i];	 k++;	i++;	   //
// 
// while (nbuffer[i]!=',') { i++;  } i++;//第4个','
// 
// //N,12020.2048,E,0.00,198.43,010213,,,A*6B
// GPS_WD[k]=nbuffer[i]; k++; i++; //Insert N=Southern Hemisphere, S=Northern Hemisphere

// 220430 supports 5 decimal places
// $GNRMC,082737.00,A,2457.09420,N,11833.39780,E,0.467,,300422,,,A,V*19
// $GNGGA,082737.00,2457.09420,N,11833.39780,E,1,04,2.54,18.9,M,9.6,M,,*45

    k = 0;
    GPS_WD[k++] = '0';

    for(n = 0; n < 9; n++)
    {
        GPS_WD[k++] = nbuffer[i];    // Insert the latitude to 4 decimal places
        i++;
    }

    if (nbuffer[i] == ',')
    {
        i++;   // 4-digit
    }
    else
    {
        i++;    // 5th place
        i++;
    }

    GPS_WD[k++] = nbuffer[i];	 		 // Insert N = Southern Hemisphere, S = Northern Hemisphere
    GPS_WD[k++] = 0x00;	// Insert end symbol

    i++;
    i++;

    k = 0;

    for(n = 0; n < 10; n++)
    {
        GPS_JD[k++] = nbuffer[i];    // Insert the longitude to 4 decimal places
        i++;
    }

    if (nbuffer[i] == ',')
    {
        i++;   // 4-digit
    }
    else
    {
        i++;    // 5th place
        i++;
    }

    GPS_JD[k++] = nbuffer[i];	  	 // Insert E = East longitude, W = West longitude
    GPS_JD[k++] = 0x00;	// Insert end symbol


// GPS_WD_NEW=(tempA-0x30)*10+(tempB-0X30); //Decimal part of latitude
// -------------------------------------------------------
// while (nbuffer[i]!=',') { i++;  } i++;//第5个','
// //12020.2048,E,0.00,198.43,010213,,,A*6B
// k=0;
// while (nbuffer[i]!=&#39;.&#39;) { GPS_JD[k]=nbuffer[i]; k++; i++; } //Insert longitude to 2 decimal places
// GPS_JD[k]=nbuffer[i]; k++; i++; //Insert decimal point
// tempA=nbuffer[i];
// GPS_JD[k]=tempA; k++; i++; //Insert 2 decimal places
// tempB=nbuffer[i];
// GPS_JD[k]=tempB;	 k++;	i++;	   //
// GPS_JD[k]=nbuffer[i]; k++; i++; //Insert 2 decimal places
// GPS_JD[k]=nbuffer[i];	 k++;	i++;	   //
// while (nbuffer[i]!=',') { i++;  } i++;//第6个','
// E,0.00,198.43,010213,,,A*6B
// GPS_JD[k]=nbuffer[i]; k++; i++; //Insert E=East longitude, W=West longitude
// GPS_JD[k]=0x00; //Insert end symbol
// GPS_JD_NEW=(tempA-0x30)*10+(tempB-0X30); //decimal part of longitude
    return 1;
}

void GET_GPS_SIG(uchar *nbuffer)	   // Number of satellites intercepted and positioned 00-12
{
    uchar i, k;
    // $GPGGA,154549.00,3134.02922,N,12020.20288,E,1,05,1.69,29.6,M,7.1,M,,*5B
    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 7)
        {
            break;
        }
    }

// k=0;
// GPS_SIG[k]=nbuffer[i];	 k++;	i++;		//
// GPS_SIG[k]=nbuffer[i];	 k++;	i++;		//
// GPS_SIG[k]=0X00; //

    GPS_SIG[0] = nbuffer[i];
    i++;
    GPS_SIG[1] = nbuffer[i];
    GPS_SIG[2] = 0X00;

    // GPS signal unit digit inserted 0
    if (GPS_SIG[1] == ',')
    {
        GPS_SIG[1] = GPS_SIG[0];
        GPS_SIG[0] = '0';
    }
}




uchar GET_GPS_ALT(uchar *nbuffer)	   // Capture the altitude and save it to altitude -9999.9 ~ 99999.9
{
    uchar i, k, n;
    unsigned char TEMP[7];
    unsigned long GPS_ALT;
    uchar  GPS_ALT_ZERO;	// Less than 0 sign


    // Extract a row from a dataframe
    // $GPGGA,154549.00,3134.02922,N,12020.20288,E,1,05,1.69,29.6,M,7.1,M,,*5B
    // Data analysis and conversion processing


    GPS_ALT = 0;	 // Altitude reset to 0
    GPS_HEIGHT[0] = '-';
    GPS_HEIGHT[1] = '-';
    GPS_HEIGHT[2] = '-';
    GPS_HEIGHT[3] = '-';
    GPS_HEIGHT[4] = '-';
    GPS_HEIGHT[5] = '-';
    GPS_HEIGHT[6] = 0x00; // End symbol


    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 9)
        {
            break;
        }
    }

    if (nbuffer[i] == ',')
    {
        GPS_ALT_ERR = 1;    // GPS has no valid positioning, no altitude data, and the data is invalid
        GPS_ALT = 0;
        GPS_ALT_MI = 0;
        return 0;
    }

    GPS_ALT_ERR = 0;

    if (nbuffer[i] == '-') 	// Negative altitude processing //Altitude is less than 0
    {
        GPS_ALT_ZERO = 0;	   	   // Skip the &#39;-&#39; symbol
        i++;
    }
    else
    {
        GPS_ALT_ZERO = 1;	   // Altitude greater than 0
    }


    n = 0;

    while (nbuffer[i] != '.')	 	// Until &#39;.&#39;, ignore the decimal part
    {
        TEMP[n] = nbuffer[i] - 0X30;
        n++;
        i++;

        if (n > 5)
        {
            GPS_ALT_ERR = 1; 	   // Maximum 5 characters. If the character is too long, an error message will be displayed.
            GPS_ALT_MI = 0;
            return 0;
        }
    }

// n=4; //test
// TEMP[0]=9; TEMP[1]=9; TEMP[2]=9; TEMP[3]=9; TEMP[4]=9; //test

    switch (n)
    {
        case 1:		 // Altitude 0-9 meters
            GPS_ALT = TEMP[0];
            break;

        case 2:		 // Altitude 10-99 meters
            GPS_ALT = TEMP[0] * 10 + TEMP[1];
            break;

        case 3:		 // Altitude 100-999 meters
            GPS_ALT = (long)TEMP[0] * 100 + TEMP[1] * 10 + TEMP[2];
            break;

        case 4:		 // Altitude 1000-9999 meters
            GPS_ALT = (long)TEMP[0] * 1000 + (long)TEMP[1] * 100 + TEMP[2] * 10 + TEMP[3];
            break;

        default:	 // Altitude 10,000-99,999 meters
            GPS_ALT = (long)TEMP[0] * 10000 + (long)TEMP[1] * 1000 + (long)TEMP[2] * 100 + TEMP[3] * 10 + TEMP[4];
            break;
    }

    if (GPS_ALT_ZERO == 0)
    {
        GPS_ALT_MI = -GPS_ALT;
    }
    else
    {
        GPS_ALT_MI = GPS_ALT;
    }

    GPS_ALT = GPS_ALT * 3.2808;	 // Convert to Imperial

    GPS_HEIGHT[0] = GPS_ALT / 100000 % 10 + 0x30;
    GPS_HEIGHT[1] = GPS_ALT / 10000 % 10 + 0x30;
    GPS_HEIGHT[2] = GPS_ALT / 1000 % 10 + 0x30;
    GPS_HEIGHT[3] = GPS_ALT / 100 % 10 + 0x30;
    GPS_HEIGHT[4] = GPS_ALT / 10 % 10 + 0x30;
    GPS_HEIGHT[5] = GPS_ALT % 10 + 0x30;
    GPS_HEIGHT[6] = 0x00; // End symbol

    if (GPS_ALT_ZERO == 0)
    {
        GPS_HEIGHT[0] = '-';
    }

    return 1;
}


unsigned char GPS_SMART()	// Whether the smart GPS beacon rules are met, 1=OK
{
    uint DIR, TEMP;
    uchar set_time;

    if (GPS_WAIT_TIME < 10)
    {
        return 0;   // Each beacon transmission interval is 10 seconds
    }

// if (GPS_WAIT_TIME&gt;179) { GPS_WAIT_TIME=0; return 2; } //Standby state, send a data every 3 minutes
    if (GPS_WAIT_TIME > ((uint)EEPROM_Buffer[0XBE] * 256 + EEPROM_Buffer[0XBF]))
    {
        GPS_WAIT_TIME = 0;
        return 2;
    }

    // In standby mode, a data is sent every 3 minutes
    // --------------------------------------------------------------------------

    if (GPS_LOCKED == 0)
    {
        return 0;   // GPS not positioning, not transmitting
    }

    if (GPS_NOW_SPEED_KNOT == 0)
    {
        return 0;   // Speed = 0, no firing
    }

// if (GPS_NOW_SPEED_KNOT==1){ return 0;} //Speed = 1, do not transmit
    if (GPS_DIR_NOTHING == 0)
    {
        return 0;   // No heading data, no transmission
    }


    // --------------------------------------------------------------------------
    // GPS Mode = Smart Beacon Mode
    // ---One of the conditions for smart GPS beacon, the direction exceeds 25 degrees left or right, and the beacon is transmitted //The extracted GPS direction is 0-359 degrees

    // Find the minimum angle between the current angle and the previous angle (absolute value)
    // 1. Compare the current angle with the previous angle to see which value is larger. If the two values are equal, ignore the following calculations.
    // 2. Calculation (large angle - small angle)
    // 3. If (maximum angle - minimum angle) &gt; 180 degrees, then the minimum angle interpolation = 360 - (maximum angle - minimum angle)
    // 4. If (large angle - small angle) &lt;= 180 degrees, then the minimum angle interpolation = (large angle - small angle)
    if (GPS_NOW_DIR > GPS_OLD_DIR) 	 // Current angle value &gt; previous angle value
    {
        TEMP = (GPS_NOW_DIR - GPS_OLD_DIR);
    }
    else							  // Current angle value &lt; previous angle value, that is, previous angle value &gt; current angle value
    {
        TEMP = (GPS_OLD_DIR - GPS_NOW_DIR);
    }

    if (TEMP > 180)
    {
        DIR = (360 - TEMP);
    }
    else
    {
        DIR = TEMP;
    }

    if (DIR > 25)	// The absolute minimum angle deviation is greater than 25 degrees, allowing launch
    {
        GPS_WAIT_TIME = 0;
        GPS_OLD_DIR = GPS_NOW_DIR; // Update the previous angle value
        return 1;	// 08=GPS connected and successfully resolved, the absolute minimum angle deviation is greater than 25 degrees, allowing transmission
    }

    // --------------------------------------------------------------------------
    // --------------------------------------------------------------------------
    switch ( EEPROM_Buffer[0x04])   	// Setting 0=FAST 1=MID1 2=MID2 3=SLOW
    {
        case 1:
            set_time = 20;
            break;

        case 2:
            set_time = 40;
            break;

        case 3:
            set_time = 60;
            break;

        case 4:
            set_time = 90;
            break;

        case 5:
            set_time = 120;
            break;


        default:	// 0
            set_time = 20;
            break;
    }

    if 	(GPS_WAIT_TIME > set_time)  // The average interval is 20 seconds.
// if (GPS_WAIT_TIME&gt;20) //Even if the transmission interval is 20 seconds
    {
        GPS_WAIT_TIME = 0;
        GPS_OLD_DIR = GPS_NOW_DIR; // Update the previous angle value
        return 1;	// Allow to transmit
    }

    return 0;	 // Do nothing
}

// void speed_haili_txt()
// {
// tostring(GPS_NOW_SPEED_KNOT); //Convert nautical mile speed to text
// GPS_NOW_SPEED_KNOT_TXT[0]=bai;
// GPS_NOW_SPEED_KNOT_TXT[1]=shi;
// GPS_NOW_SPEED_KNOT_TXT[2]=ge;
// GPS_NOW_SPEED_KNOT_TXT[3]=0;
// 
// }

uchar GET_GPS_SPEED(uchar *nbuffer)	// Find the current speed GPS_NOW_SPEED_KNOT, return 0 = no speed 1 = speed
{
    unsigned char TEMP[3];
    uchar i, n;
// --------------------------------------------------------------------------
// Find the current speed and heading
// ---------------------------------------------------------
// $GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
// $GPRMC,000000.000,V,0000.0000,S,00000.0000,W,0.00,0.00,220899,,,A*7E
// --------------------------------------------------------------------------
// $GPRMC,145240.00,A,3134.00679,N,12020.21635,E,1.768,159.36,180813,,,A*64
// $GPRMC,145241.00,A,3134.00716,N,12020.21622,E,1.467,,180813,,,A*71
// $GPRMC,145006.00,A,3134.01359,N,12020.21880,E,1.142,,180813,,,A*7A
// Note that this heading data may cause calculation overflow and affect subsequent data.
// --------------------------------------------------------------------------
// $GPRMC,143350.00,A,3134.03522,N,12020.20752,E,1.195,143.22,101013,,,A*66

    i = 0;
    n = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            n++;
        }

        i++;

        if (n == 7)
        {
            break;   // Skip 7,
        }
    }

    if (nbuffer[i] == ',') 	 // No speed data, no processing
    {
        GPS_NOW_SPEED_KNOT = 0;
        GPS_SPEED_KM = 0;

        return 0;
    }

    // --------------------------------------------------------------------------

    n = 0;

    while (nbuffer[i] != '.')
    {
        TEMP[n] = nbuffer[i];		   // Until &#39;.&#39;, ignore the decimal part
        i++;
        n++;
    }

    switch (n)
    {
        case 2:		 // Speed 10-99 knots
            GPS_NOW_SPEED_KNOT = ((TEMP[0] - 0x30) * 10 + (TEMP[1] - 0x30)) ;	 // GPS current speed, 0-255 nautical miles
            break;

        case 3:		 // Speed 100-255 knots
            GPS_NOW_SPEED_KNOT = ((TEMP[0] - 0x30) * 100 + (TEMP[1] - 0x30) * 10 + (TEMP[2] - 0x30));	 // GPS current speed, 0-255 nautical miles
            break;

        default:	 // Speed 0-9 knots
            GPS_NOW_SPEED_KNOT = (TEMP[0] - 0x30);	 // GPS current speed, 0-255 nautical miles
            break;
    }


    GPS_SPEED_KM = GPS_NOW_SPEED_KNOT * 1.852;	// Convert to meters, 0.1KM

// speed_haili_txt();


// if (n==1){TEMP[2]=TEMP[0]; TEMP[1]=&#39;0&#39;; TEMP[0]=&#39;0&#39;; } //Speed 0-9 nautical miles
// if (n==2){TEMP[2]=TEMP[1]; TEMP[1]=TEMP[0]; TEMP[0]=&#39;0&#39;; } //Speed 10-99 nautical miles
// GPS_NOW_SPEED_KNOT=((TEMP[0]-0x30)*100+ (TEMP[1]-0x30)*10+(TEMP[2]-0x30)); //GPS current speed, 0-255 nautical miles
    return 0;
}


uchar GET_GPS_DIR(uchar *nbuffer)	// Request current heading GPS_NOW_DIR, return 0 = no heading 1 = heading
{
    unsigned char TEMP[3];
    uchar i, n;
    // --------------------------------------------------------------------------
    // Find the current speed and heading
    // When stationary, there is no heading data, which may cause calculation overflow and affect subsequent data
    // $GPRMC,145006.00,A,3134.01359,N,12020.21880,E,1.142,,180813,,,A*7A
    // --------------------------------------------------------------------------
    // With heading data
    // $GPRMC,143350.00,A,3134.03522,N,12020.20752,E,1.195,143.22,101013,,,A*66

    i = 0;
    n = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            n++;
        }

        i++;

        if (n == 8)
        {
            break;   // Skip 8,
        }
    }


    if (nbuffer[i] == ',')
    {
        // GPS_NOW_DIR=0; //When there is no heading data, keep the last heading
        GPS_DIR_NOTHING = 0;
        return 0; 	// No heading data, no processing
    }

    // --------------------------------------------------------------------------

    n = 0;

    while (nbuffer[i] != '.')
    {
        TEMP[n] = nbuffer[i];			   // Until &#39;.&#39;, ignore the decimal part
        i++;
        n++;
    }

// n=3;
// TEMP[0]='9'; TEMP[1]='8';TEMP[2]='7';

    switch (n)
    {
        case 2:		 // GPS current direction 10-99 degrees
            GPS_NOW_DIR = ((TEMP[0] - 0x30) * 10 + (TEMP[1] - 0x30)) ;
            break;

        case 3:		 // GPS current direction 100-359 degrees
            GPS_NOW_DIR = ((TEMP[0] - 0x30) * 100 + (TEMP[1] - 0x30) * 10 + (TEMP[2] - 0x30));
            break;

        default:	 // GPS current direction 0-9 degrees
            GPS_NOW_DIR = (TEMP[0] - 0x30);	 // GPS current speed, 0-255 nautical miles
            break;
    }

    GPS_DIR_NOTHING = 1;

    tostring(GPS_NOW_DIR);	  // course
    GPS_NOW_DIR_TXT[0] = bai;
    GPS_NOW_DIR_TXT[1] = shi;
    GPS_NOW_DIR_TXT[2] = ge;
    GPS_NOW_DIR_TXT[3] = 0;

    return 1;
}




void QUICK_SET_FIXED()	  // Whether to quickly modify the longitude and latitude of the fixed station
{
    uint i;

    if (QUICK_SET_EN == 1)
    {
        QUICK_SET_EN = 0;

        // Set the current GPS latitude and longitude to the fixed station latitude and longitude
        for(i = 0; i < 8; i++)
        {
            EEPROM_Buffer[0x20 + i] = GPS_WD[i];
        }

        for(i = 0; i < 9; i++)
        {
            EEPROM_Buffer[0x30 + i] = GPS_JD[i];
        }

        EEPROM_UPDATA();

// UART2_SendString(GPS_WD); UART2_SendString(&quot; &quot;); UART2_SendString(GPS_JD); UART2_SendString(&quot;\r\n&quot;);
// UART2_SendString("QUICK SETUP\r\n");


        UART4_TX_EEROM();  // Refresh menu settings

        UART2_SendString("HELLO");	 // Refresh Bluetooth settings

        for(i = 0; i < 512; i++)
        {
            UART2_SendData(EEPROM_Buffer[i]);	 	   // Copy parameters to buffer
        }

    }
}




void UART3_RX_GPS()	  // Debug GPS data
{
    if (sum_gps() != 1)
    {
        return;   // Check GPS data, if it is correct, continue to process GPS data
    }

    GET_GPS_LOCK_STATUS(UART3_GPRMC_DATA);		// Check positioning status, intercept latitude and longitude, heading, speed, and save to GPS_DATA

    if (GPS_LOCKED == 0)
    {
        QUICK_SET_EN = 0;	   // GPS unlocked interface
        return;
    }


    // If GPS is locked, parse data and send beacon
    GET_GPS_TIME(UART3_GPRMC_DATA);// Request time
    GET_GPS_DATE(UART3_GPRMC_DATA);// Intercept GPS date

    GET_GPS_LONG_LAT(UART3_GPRMC_DATA);	  // GPRMC intercepts the longitude and latitude and saves them to GPS_DATA.
    GET_GPS_SPEED (UART3_GPRMC_DATA);	  // GPRMC for speed
    GPS_DIR_NOTHING = GET_GPS_DIR (UART3_GPRMC_DATA); // GPRMC Heading
// if (GPS_DIR_NOTHING==0){return;} //No heading data, no transmission

    GET_GPS_ALT(UART3_GPGGA_DATA); // Intercepting altitude in GPGGA
    GET_GPS_SIG(UART3_GPGGA_DATA);	// The number of satellites intercepted and positioned in GPGGA 00-12


    GPS_FORMAT_A();   // Degrees and minutes 03134.3795N 12019.8827E rounded to 3134.38N 12019.88E
    // ==============================================
    QUICK_SET_FIXED();	  // Round off first, whether to quickly modify the longitude and latitude of the fixed station
    // ==============================================


// disp_gps_lock(); //GPS lock interface
    // ==============================================
    if (EEPROM_Buffer[0X2A] == 1)	 // Mobile Station Mode
    {
        if (EEPROM_Buffer[0x05] != 0)		// Queue Mode
        {
            if (TIME_GPS_SEC == EEPROM_Buffer[0x06])
            {
                BEACON_GPS_TXD();
            }
        }

        SMART_MODE = 0;

        if (EEPROM_Buffer[0x04] != 0)
        {
            SMART_MODE = GPS_SMART() ;

            if (SMART_MODE != 0)
            {
                BEACON_GPS_TXD();     // Smart Beacons
            }
        }
    }
}















