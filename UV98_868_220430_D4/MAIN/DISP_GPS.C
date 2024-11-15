
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "IO.H"
// #include "DELAY.H"
// #include "ADC.H"
#include "tostring.H"

#include "GPS2.H"
#include "DISP_GPS.H"

#include "UART2.H"
#include "UART3.H"
#include "UART4.H"

#include "bmp280.h"
#include "PUBLIC_BUF.H"
#include "ADC.H"
#include "GPS_JULI.H"
#include "KISS_Analysis.H"

#include<string.h>



uchar GPS_LINE_BUF[25]; // Display Cache
uchar GPS_WD_DISP_BUF[20]; // Display latitude data
uchar GPS_JD_DISP_BUF[20]; // Display longitude data
uchar GPS_WG_BUF[20]; // Grid Data

bit GPS_DISP_MODE;
uchar GPS_DISP_COUNT;
uchar GPS_OLD_STU, GPS_NEW_STU; // Signal prompt

// $GNRMC,150203.000,A,3134.3654,N,12019.8863,E,0.00,194.56,280918,,,A*7F
// $GNGGA,150204.000,3134.3654,N,12019.8863,E,1,05,3.4,137.6,M,0.0,M,,*7E

void gps_uartx_out()
{
    UART4_SendString(SN_RX_BUFFER);	  // RT2_SendString(SN_RX_BUFFER);

}

void calcLocator(char *dst, double lat, double lon) 			// Grid
{
    int o1, o2, o3;
    int a1, a2, a3;
    double remainder;
    // longitude
    remainder = lon + 180.0;
    o1 = (int)(remainder / 20.0);
    remainder = remainder - (double)o1 * 20.0;
    o2 = (int)(remainder / 2.0);
    remainder = remainder - 2.0 * (double)o2;
    o3 = (int)(12.0 * remainder);

    // latitude
    remainder = lat + 90.0;
    a1 = (int)(remainder / 10.0);
    remainder = remainder - (double)a1 * 10.0;
    a2 = (int)(remainder);
    remainder = remainder - (double)a2;
    a3 = (int)(24.0 * remainder);
    dst[0] = (char)o1 + 'A';
    dst[1] = (char)a1 + 'A';
    dst[2] = (char)o2 + '0';
    dst[3] = (char)a2 + '0';
    dst[4] = (char)o3 + 'A';
    dst[5] = (char)a3 + 'A';
    dst[6] = (char)0;
}
// Write degrees, minutes, seconds
void write_degr_min_sec(uchar *p, uint degr, uint min, uint sec, uchar NSWE)
{
    *p = degr / 100 + 0x30;

    if (*p == '0')
    {
        *p = ' ';
    }

    p++;
    *p = degr / 10 % 10 + 0x30;
    p++;
    *p = degr % 10 + 0x30;
    p++;
    *p = '@';
    p++;
    *p = min / 10 + 0x30;
    p++;
    *p = min % 10 + 0x30;
    p++;
    *p = '\'';
    p++;
    *p = sec / 10 + 0x30;
    p++;
    *p = sec % 10 + 0x30;
    p++;
    *p = '"';
    p++;
    *p = ' ';
    p++;
    *p = NSWE;
    p++;
    *p = 0;
}

// Write speed
void write_degr(uchar *p, long degr, uchar NSWE)
{
    *p = (uchar)(degr / 10000000) + 0x30;

    if (*p == '0')
    {
        *p = ' ';
    }

    p++;	  // Latitude leading space
    *p = (uchar)(degr / 1000000 % 10) + 0x30;
    p++;
    *p = (uchar)(degr / 100000 % 10) + 0x30;
    p++;
    *p = '.';
    p++;
    *p = (uchar)(degr / 10000 % 10) + 0x30;
    p++;
    *p = (uchar)(degr / 1000 % 10) + 0x30;
    p++;
    *p = (uchar)(degr / 100 % 10) + 0x30;
    p++;
    *p = (uchar)(degr / 10 % 10) + 0x30;
    p++;
    *p = (uchar)(degr % 10) + 0x30;
    p++;
    *p = '@';
    p++;
    *p = ' ';
    p++;
    *p = NSWE;
    p++;
    *p = 0;
}

// Write degree points.
void write_degr_min_min(uchar *p, uint degr, uint min_a, uint min_b, uchar NSWE)
{
    *p = degr / 100 + 0x30;

    if (*p == '0')
    {
        *p = ' ';
    }

    p++;
    *p = degr / 10 % 10 + 0x30;
    p++;
    *p = degr % 10 + 0x30;
    p++;
    *p = '@';
    p++;
    *p = min_a / 10 + 0x30;
    p++;
    *p = min_a % 10 + 0x30;
    p++;
    *p = '.';
    p++;
    *p = min_b / 10 + 0x30;
    p++;
    *p = min_b % 10 + 0x30;
    p++;
    *p = '\'';
    p++;
    *p = ' ';
    p++;
    *p = NSWE;
    p++;
    *p = 0;
}


void write_lat_lon(uchar *p, uint degr, uint min_a, uint min_b, uint min_c, uchar NSWE, uchar lat_lon)
{
    min_c = min_c + 50;			 // Rounding

    if (min_c > 99)
    {
        min_b++;
    }

    if (min_b > 99)
    {
        min_b = 0;
        min_a++;
    }

    if (min_a > 59)
    {
        min_a = 0;
        degr++;
    }

    if (lat_lon == 0)		// 0 = latitude 1 = longitude
    {
        *p = degr / 10 + 0x30;
        p++;
        *p = degr % 10 + 0x30;
        p++;
    }
    else
    {
        *p = degr / 100 + 0x30;
        p++;
        *p = degr / 10 % 10 + 0x30;
        p++;
        *p = degr % 10 + 0x30;
        p++;
    }

    *p = min_a / 10 + 0x30;
    p++;
    *p = min_a % 10 + 0x30;
    p++;
    *p = '.';
    p++;
    *p = min_b / 10 + 0x30;
    p++;
    *p = min_b % 10 + 0x30;
    p++;
    *p = NSWE;
    p++;
    *p = 0;
}




void GPS_FORMAT_A()	      // Degrees and minutes are rounded
{
    uint Degr, min_a, min_b, min_c, sec;
    uchar NSWE;
    long   wd_Degr, jd_Degr;

    double wd_temp, jd_temp;
    uchar ns, we;

// fly GPS_WD[]={&quot;03134.0377S&quot;}; //NE=PM01EN SE=PF08EK
// fly GPS_JD[]={&quot;12020.2048W&quot;}; //NW=CM91TM SW=CF98TK


    Degr = (GPS_WD[0] - 0x30) * 100 + (GPS_WD[1] - 0x30) * 10 + (GPS_WD[2] - 0x30);
    min_a = (GPS_WD[3] - 0x30) * 10 + (GPS_WD[4] - 0x30);
    min_b = (GPS_WD[6] - 0x30) * 10 + (GPS_WD[7] - 0x30);
    min_c = (GPS_WD[8] - 0x30) * 10 + (GPS_WD[9] - 0x30);

    NSWE = GPS_WD[10];
    ns = GPS_WD[10];

    wd_Degr = ((long)min_b * 100 + (long)min_c + (long)min_a * 10000) * 10;
    wd_Degr = (wd_Degr / 6) +	(long)Degr * 1000000; // Convert to degrees
    wd_Degr = (wd_Degr + 5) / 10;	 // Round to 5 decimal places

    // 0 = degrees 1 = degrees minutes 2 = degrees minutes seconds
    if (EEPROM_Buffer[0X17] == 0)
    {
        write_degr(GPS_WD_DISP_BUF, wd_Degr, NSWE);
    }

    if (EEPROM_Buffer[0X17] == 2)	// 0 = degrees 1 = degrees minutes 2 = degrees minutes seconds
    {
        sec =	((((long)min_b * 100 + (long)min_c) * 0.06) + 5) / 10; // rounding
        write_degr_min_sec(GPS_WD_DISP_BUF, Degr, min_a, sec, NSWE);
    }

    write_lat_lon(GPS_WD, Degr, min_a, min_b, min_c, NSWE, 0);	// Combine the latitude of the beacon transmission and round it up

    // 0 = degrees 1 = degrees minutes 2 = degrees minutes seconds // latitude
    if (EEPROM_Buffer[0X17] == 1)
    {
        write_degr_min_min(GPS_WD_DISP_BUF, Degr, min_a, min_b, NSWE);
    }

    // ==============================================
    // ==============================================
    Degr = (GPS_JD[0] - 0x30) * 100 + (GPS_JD[1] - 0x30) * 10 + (GPS_JD[2] - 0x30);
    min_a = (GPS_JD[3] - 0x30) * 10 + (GPS_JD[4] - 0x30);
    min_b = (GPS_JD[6] - 0x30) * 10 + (GPS_JD[7] - 0x30);
    min_c = (GPS_JD[8] - 0x30) * 10 + (GPS_JD[9] - 0x30);

    NSWE = GPS_JD[10];
    we = GPS_JD[10];

    jd_Degr = ((long)min_b * 100 + (long)min_c + (long)min_a * 10000) * 10;
    jd_Degr = (jd_Degr / 6) +	(long)Degr * 1000000; // Convert to degrees
    jd_Degr = (jd_Degr + 5) / 10;	 // Round to 5 decimal places

    // ==============================================
    if (EEPROM_Buffer[0X17] == 0)	// 0 = degrees 1 = degrees minutes 2 = degrees minutes seconds
    {
        write_degr(GPS_JD_DISP_BUF, jd_Degr, NSWE);
    }

    if (EEPROM_Buffer[0X17] == 2)	// 0 = degrees 1 = degrees minutes 2 = degrees minutes seconds
    {
        sec =	((((long)min_b * 100 + (long)min_c) * 0.06) + 5) / 10; // rounding
        write_degr_min_sec(GPS_JD_DISP_BUF, Degr, min_a, sec, NSWE);
    }

    write_lat_lon(GPS_JD, Degr, min_a, min_b, min_c, NSWE, 1);	// Combine the longitude of the beacon transmission and round it

    // The previous rounding must be done before displaying the degree and minute format
    // 0 = degrees 1 = degrees minutes 2 = degrees minutes seconds
    if (EEPROM_Buffer[0X17] == 1)
    {
        write_degr_min_min(GPS_JD_DISP_BUF, Degr, min_a, min_b, NSWE);
    }

    // ==============================================

    if (ns == 'N')
    {
        wd_temp = (double) wd_Degr / 100000;
    }
    else
    {
        wd_temp = -((double) wd_Degr / 100000);
    }

    if (we == 'E')
    {
        jd_temp = (double) jd_Degr / 100000;
    }
    else
    {
        jd_temp = -((double) jd_Degr / 100000);
    }

    calcLocator(GPS_WG_BUF, wd_temp, jd_temp ) ;	// Grid
// calcLocator(GPS_WG_BUF, (double) wd_Degr/100000, (double) jd_Degr/100000); //Grid

}

void GPS_DISP_SPACE(uchar space)
{
    uchar i;

    for(i = 0; i < space; i++)
    {
        ASC_TEMP[i] = ' ';
        ASC_TEMP[i + 1] = 0;
    }

}
void  disp_TOTAL_LC()
{
    uchar i, k;

    for(i = 0; i < 10; i++)
    {
        ASC_TEMP[i] = 0x00;
    }

    k = 0;
// if (dat&lt;0) {ASC_TEMP[k++]=&#39;-&#39;; dat=-dat; }else{ASC_TEMP[k++]=&#39; &#39;; }

    tostring((uint)(TOTAL_LC / 100));
    ASC_TEMP[k++] = wan;
    ASC_TEMP[k++] = qian;
    ASC_TEMP[k++] = bai;
    ASC_TEMP[k++] = shi;
    ASC_TEMP[k++] = '.';
    ASC_TEMP[k++] = ge;
    ASC_TEMP[k++] = 0x00;
}


void DISP_GPS_MSG()	   // Show additional information
{

// GPS_DISP_COUNT++;
// if (GPS_DISP_COUNT>2){GPS_DISP_COUNT=0; GPS_DISP_MODE=!GPS_DISP_MODE;}

// for (i=0;i<25;i++)	{	GPS_LINE_BUF[i]=' ';	}

    if (GPS_DISP_MODE == 0)
    {
        READ_ADC()	;
// strcat(SN_RX_BUFFER,"B:");
        strcat(SN_RX_BUFFER, DY);
        strcat(SN_RX_BUFFER, "V ");

        if( BMP280_LINK == 1)
        {
            strcat(SN_RX_BUFFER, BMP280_TEMP);
        }
        else
        {
            strcat(SN_RX_BUFFER, "---.-");
        }

        strcat(SN_RX_BUFFER, "@C ");

        if( BMP280_LINK == 1)
        {
            strcat(SN_RX_BUFFER, BMP280_QY);
        }
        else
        {
            strcat(SN_RX_BUFFER, "----.-");
        }

        strcat(SN_RX_BUFFER, "hPa");
        strcat(SN_RX_BUFFER, ",");
    }
    else
    {
        if (GPS_LINK == 0)		 // GPS OFF
        {
            strcat(SN_RX_BUFFER, "GPS OFF ");
            disp_TOTAL_LC();
            strcat(SN_RX_BUFFER, ASC_TEMP);
            strcat(SN_RX_BUFFER, "Km");
            strcat(SN_RX_BUFFER, ",");
            return;
        }

        if (GPS_LOCKED == 0)		 // GPS no positioning
        {
            strcat(SN_RX_BUFFER, "GPS Wait... ");
            disp_TOTAL_LC();
            strcat(SN_RX_BUFFER, ASC_TEMP);
            strcat(SN_RX_BUFFER, "Km");
            strcat(SN_RX_BUFFER, ",");
            return;
        }

        if (GPS_LOCKED == 1)	   // GPS Positioning
        {
            strcat(SN_RX_BUFFER, "S:");
            strcat(SN_RX_BUFFER, GPS_SIG);

            strcat(SN_RX_BUFFER, " ");
            disp_TOTAL_LC();
            strcat(SN_RX_BUFFER, ASC_TEMP);
            strcat(SN_RX_BUFFER, "Km   ");

// GPS_DISP_SPACE(12);		strcat(SN_RX_BUFFER,ASC_TEMP);
            if (GPS_DIR_NOTHING == 1)
            {
                strcat(SN_RX_BUFFER, GPS_NOW_DIR_TXT);
            }
            else
            {
                strcat(SN_RX_BUFFER, "---");
            }

// GPS_DISP_SPACE(2);		strcat(SN_RX_BUFFER,ASC_TEMP);

            strcat(SN_RX_BUFFER, ",");
            return;
        }
    }
}


void  disp_alt_mi(long dat)
{
    uchar i, k;

    for(i = 0; i < 10; i++)
    {
        ASC_TEMP[i] = 0x00;
    }

    k = 0;

    if (dat < 0)
    {
        ASC_TEMP[k++] = '-';
        dat = -dat;
    }
    else
    {
        ASC_TEMP[k++] = ' ';
    }

    tostring((uint)dat);
    ASC_TEMP[k++] = wan;
    ASC_TEMP[k++] = qian;
    ASC_TEMP[k++] = bai;
    ASC_TEMP[k++] = shi;
    ASC_TEMP[k++] = ge;
    ASC_TEMP[k++] = 0x00;
}




void  disp_spd_km(uint dat)
{
    uchar i, k;

    for(i = 0; i < 10; i++)
    {
        ASC_TEMP[i] = 0x00;
    }

    k = 0;
// if (dat&lt;0) {ASC_TEMP[k++]=&#39;-&#39;; dat=-dat; }else{ASC_TEMP[k++]=&#39; &#39;; }

    tostring((uint)dat);
    ASC_TEMP[k++] = wan;
    ASC_TEMP[k++] = qian;
    ASC_TEMP[k++] = bai;
    ASC_TEMP[k++] = shi;
    ASC_TEMP[k++] = ge;
    ASC_TEMP[k++] = 0x00;



}
// 17		18			19			1A			1B
// Coordinate Format Speed Unit Distance Unit Altitude Unit Temperature Unit
// 1 0 0 0=meter 0=celsius
// 0=degrees 0=km 0=km 1=feet 1=Fahrenheit
// 1=degree minute 1=nautical mile 1=nautical mile
// 2 = degrees, minutes, seconds 2 = miles 2 = miles


void disp_gps_nolock()
{
    READ_BMP280();	// Refresh temperature and air pressure
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$A02,V,");

    GPS_DISP_SPACE(14);
    strcat(SN_RX_BUFFER, ASC_TEMP);	 	  // The first line is 19 characters long.
    strcat(SN_RX_BUFFER, "Wait.");
    strcat(SN_RX_BUFFER, ",");


// strcat(SN_RX_BUFFER,"lin 20");	 	strcat(SN_RX_BUFFER,",");
// strcat(SN_RX_BUFFER,"lin 30");	 	strcat(SN_RX_BUFFER,",");
// strcat(SN_RX_BUFFER,"lin 40");	 	strcat(SN_RX_BUFFER,",");
// strcat(SN_RX_BUFFER,"lin 50");	 	strcat(SN_RX_BUFFER,",");
// strcat(SN_RX_BUFFER,"lin 60");	 	strcat(SN_RX_BUFFER,",");

    strcat(SN_RX_BUFFER, ",");	 // Do not refresh
    strcat(SN_RX_BUFFER, ",");
    strcat(SN_RX_BUFFER, ",");
    strcat(SN_RX_BUFFER, ",");
    strcat(SN_RX_BUFFER, ",");


    DISP_GPS_MSG();	   // Show additional information line 7

    strcat(SN_RX_BUFFER, "---");
    strcat(SN_RX_BUFFER, ",");	 // Row 8

    strcat(SN_RX_BUFFER, "\r\n");

    gps_uartx_out();
}



void  disp_Angle(uint dat)
{
    uchar i, k;

    for(i = 0; i < 10; i++)
    {
        ASC_TEMP[i] = 0x00;
    }

    k = 0;
    tostring((uint)dat);
    ASC_TEMP[k++] = bai;
    ASC_TEMP[k++] = shi;
    ASC_TEMP[k++] = ge;
    ASC_TEMP[k++] = 0x00;
}


void disp_gps_lock()
{
    uint temp_angle;
    READ_BMP280();	  	// Refresh temperature and air pressure

    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$A02,A,");
    // ================================================ Line 1 The first line is 19 characters long

    if (GPS_DISP_MODE == 0) // 19 bytes per line
    {
        GPS_DISP_SPACE(11);
        strcat(SN_RX_BUFFER, ASC_TEMP);
        strcat(SN_RX_BUFFER, GPS_TIME);
    }
    else
    {
        strcat(SN_RX_BUFFER, "P0->");
        GET_AB_JULI(2);
        strcat(SN_RX_BUFFER, UI_JULI + 1);
        strcat(SN_RX_BUFFER, "Km ");
        // if (UI_JULI[0]=='*')	{strcat(SN_RX_BUFFER,UI_JULI+1);	strcat(SN_RX_BUFFER,"Km ");}

        temp_angle =	GET_AB_Angle(2);								 // True North
        // List direction display mode 0=English 1=0-12 2-0-36
        disp_Angle(temp_angle);
        strcat(SN_RX_BUFFER, ASC_TEMP);	// strcat(SN_RX_BUFFER,ASC_TEMP);
        strcat(SN_RX_BUFFER, " ");

        if (GPS_DIR_NOTHING == 0)
        {
            strcat(SN_RX_BUFFER, "--");
        }
        else
        {
            temp_angle =   GET_AB_POINT_DIR(temp_angle, GPS_NOW_DIR)	;	// Relative direction of vehicle head
            disp_Angle(temp_angle);
            strcat(SN_RX_BUFFER, ASC_TEMP);	// strcat(SN_RX_BUFFER,ASC_TEMP);
        }
    }

    strcat(SN_RX_BUFFER, ",");

    // ================================================ Line 2

    disp_spd_km(GPS_SPEED_KM);
    strcat(SN_RX_BUFFER, "SPD:");
    strcat(SN_RX_BUFFER, ASC_TEMP);
    strcat(SN_RX_BUFFER, "Km  ");

    strcat(SN_RX_BUFFER, GPS_WG_BUF)	; // Show Grid
    // if (GPS_DIR_NOTHING==1) {strcat(SN_RX_BUFFER,GPS_NOW_DIR_TXT); }else{	strcat(SN_RX_BUFFER,"---");}
    strcat(SN_RX_BUFFER, ",");

    // ================================================ Line 3


// GPS_ALT_ERR; //GPS has no valid positioning, 1 = no altitude data, then the data is invalid 0 = altitude data is normal
    if (GPS_ALT_ERR == 0)
    {
        disp_alt_mi(GPS_ALT_MI);
        strcat(SN_RX_BUFFER, "ALT:");
        strcat(SN_RX_BUFFER, ASC_TEMP);
        strcat(SN_RX_BUFFER, "M,");
    }
    else
    {
        strcat(SN_RX_BUFFER, "ALT:");
        strcat(SN_RX_BUFFER, " -----");
        strcat(SN_RX_BUFFER, "M,");
    }

    // ================================================ Line 4
    strcat(SN_RX_BUFFER, GPS_WD_DISP_BUF);
    strcat(SN_RX_BUFFER, ",");
    // ================================================ Line 5
    strcat(SN_RX_BUFFER, GPS_JD_DISP_BUF);
    strcat(SN_RX_BUFFER, ",");
    // ================================================ Line 6
    strcat(SN_RX_BUFFER, GPS_DATE);
    strcat(SN_RX_BUFFER, ",");
    // ================================================ Line 7
    DISP_GPS_MSG();	   // Show additional information

    // ================================================ Line 8
    if (GPS_DIR_NOTHING == 1)
    {
        strcat(SN_RX_BUFFER, GPS_NOW_DIR_TXT);
    }
    else
    {
        strcat(SN_RX_BUFFER, "---");
    }

    strcat(SN_RX_BUFFER, ",");
    // ==============================================
    strcat(SN_RX_BUFFER, "\r\n");
    gps_uartx_out();
}



void disp_gps_OFF()
{
    uchar i, k;
    READ_BMP280();	// Refresh temperature and air pressure
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$A02,O,");

    GPS_DISP_SPACE(12);
    strcat(SN_RX_BUFFER, ASC_TEMP);
    strcat(SN_RX_BUFFER, "GPS OFF");
    strcat(SN_RX_BUFFER, ",");	 // The first line is 19 characters long.

    strcat(SN_RX_BUFFER, "SPD:------Kmh  Fixed ");
    strcat(SN_RX_BUFFER, ",");	 // 21 bytes per line
    strcat(SN_RX_BUFFER, "alt:------M");
    strcat(SN_RX_BUFFER, ",");


// for(i=0;i<8;i++)  	{KISS_DATA[k]=EEPROM_Buffer[0x20+i];  k++;}
// for(i=0;i<9;i++)  	{KISS_DATA[k]=EEPROM_Buffer[0x30+i];  k++;} 	 //固定站点经度
// unsigned char code WD[10]={&quot;3135.90N&quot;}; //Default
// unsigned char code JD[10]={&quot;12021.80E&quot;}; //Default



    k = 0;
    ASC_TEMP[k++] = ' ';

    for(i = 0; i < 2; i++)
    {
        ASC_TEMP[k++] = EEPROM_Buffer[0x20 + i];
    }

    ASC_TEMP[k++] = '@';

    for(i = 0; i < 5; i++)
    {
        ASC_TEMP[k++] = EEPROM_Buffer[0x22 + i];
    }

    ASC_TEMP[k++] = '\'';
    ASC_TEMP[k++] = ' ';
    ASC_TEMP[k++] = EEPROM_Buffer[0x27];
    ASC_TEMP[k++] = 0;
    strcat(SN_RX_BUFFER, ASC_TEMP);
    strcat(SN_RX_BUFFER, ",");

    k = 0;

    for(i = 0; i < 3; i++)
    {
        ASC_TEMP[k++] = EEPROM_Buffer[0x30 + i];
    }

    ASC_TEMP[k++] = '@';

    for(i = 0; i < 5; i++)
    {
        ASC_TEMP[k++] = EEPROM_Buffer[0x33 + i];
    }

    ASC_TEMP[k++] = '\'';
    ASC_TEMP[k++] = ' ';
    ASC_TEMP[k++] = EEPROM_Buffer[0x38];
    ASC_TEMP[k++] = 0;
    strcat(SN_RX_BUFFER, ASC_TEMP);
    strcat(SN_RX_BUFFER, ",");


// strcat(SN_RX_BUFFER," 31@00.00' N");	 	strcat(SN_RX_BUFFER,",");
// strcat(SN_RX_BUFFER,"120@00.00' E");	 	strcat(SN_RX_BUFFER,",");
    strcat(SN_RX_BUFFER, "---- -- --  ");
    strcat(SN_RX_BUFFER, ",");


    DISP_GPS_MSG();	   // Show additional information line 7
    strcat(SN_RX_BUFFER, "---");
    strcat(SN_RX_BUFFER, ",");	 // Row 8
    strcat(SN_RX_BUFFER, "\r\n");
    // UART1_SendString(SN_RX_BUFFER);
    gps_uartx_out();
}


uchar gps_no_sig_beep()
{
    if (GPS_OLD_STU == GPS_NEW_STU)
    {
        return 0;
    }

    GPS_OLD_STU = GPS_NEW_STU;
    UART4_SendString("$F01\r\n");	// UART2_SendString(&quot;$F01\r\n&quot;);//Signal prompt
    return 1;
}





void DISP_GPS_STU()
{
    GPS_DISP_COUNT++;

    if (GPS_DISP_COUNT > 2)
    {
        GPS_DISP_COUNT = 0;
        GPS_DISP_MODE = !GPS_DISP_MODE;
    }


    if (gps_no_sig_beep() == 1)
    {
        return;   // When the status changes, do not send the prompt tone together with the following data to prevent the main board from not being able to detect it.
    }

    if (GPS_LINK  == 0)
    {
        GPS_NEW_STU = 0;	   // GPS disconnected
        disp_gps_OFF();
        return;
    }

    if (GPS_LOCKED == 0)
    {
        GPS_NEW_STU = 1;	   // GPS no positioning
        disp_gps_nolock();
        return;
    }

    if (GPS_LOCKED == 1)
    {
        GPS_NEW_STU = 2;	   // GPS Positioning
        disp_gps_lock();
        return;
    }
}



// GPS_WD[0]=Degr/10+0x30;	 	GPS_WD[1]=Degr%10+0x30;
// GPS_WD[2]=min_a/10+0x30;	GPS_WD[3]=min_a%10+0x30;
// GPS_WD[4]='.';
// GPS_WD[5]=min_b/10+0x30;	GPS_WD[6]=min_b%10+0x30;
// GPS_WD[7]=NSWE;		GPS_WD[8]=0;

// k=0;
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr/10000000)+0x30;
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr/1000000%10)+0x30;
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr/100000%10)+0x30;
// GPS_JD_DISP_BUF[k++]='.';
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr/10000%10)+0x30;
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr/1000%10)+0x30;
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr/100%10)+0x30;
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr/10%10)+0x30;
// GPS_JD_DISP_BUF[k++]=(fly)(jd_Degr%10)+0x30;
// GPS_JD_DISP_BUF[k++]='@';
// GPS_JD_DISP_BUF[k++]=' '; 		GPS_JD_DISP_BUF[k++]=NSWE; 	GPS_JD_DISP_BUF[k++]=0;

// UART2_SendString(GPS_JD_DISP_BUF); UART2_SendString(&quot;\r\n&quot;);
// k=0;
// GPS_WD_DISP_BUF[k++] =' ';
// GPS_WD_DISP_BUF[k++]=GPS_WD[0]; GPS_WD_DISP_BUF[k++]=GPS_WD[1]; 	GPS_WD_DISP_BUF[k++]='@';
// GPS_WD_DISP_BUF[k++]=GPS_WD[2]; GPS_WD_DISP_BUF[k++]=GPS_WD[3];  	GPS_WD_DISP_BUF[k++]='.';
// GPS_WD_DISP_BUF[k++]=GPS_WD[5]; GPS_WD_DISP_BUF[k++]=GPS_WD[6]; 	GPS_WD_DISP_BUF[k++]='\'';
// GPS_WD_DISP_BUF[k++]=' '; 		GPS_WD_DISP_BUF[k++]=NSWE; 	GPS_WD_DISP_BUF[k++]=0;
// UART2_SendString(GPS_WD_DISP_BUF); UART2_SendString(&quot;\r\n&quot;);
// k=0;
// GPS_JD_DISP_BUF[k++]=GPS_JD[0]; GPS_JD_DISP_BUF[k++]=GPS_JD[1]; 	GPS_JD_DISP_BUF[k++]=GPS_JD[2];	GPS_JD_DISP_BUF[k++]='@';
// GPS_JD_DISP_BUF[k++]=GPS_JD[3]; GPS_JD_DISP_BUF[k++]=GPS_JD[4];  	GPS_JD_DISP_BUF[k++]='.';
// GPS_JD_DISP_BUF[k++]=GPS_JD[6]; GPS_JD_DISP_BUF[k++]=GPS_JD[7]; 	GPS_JD_DISP_BUF[k++]='\'';
// GPS_JD_DISP_BUF[k++]=' '; 		GPS_JD_DISP_BUF[k++]=NSWE; 	GPS_JD_DISP_BUF[k++]=0;
// UART2_SendString(GPS_JD_DISP_BUF); UART2_SendString(&quot;\r\n&quot;);
