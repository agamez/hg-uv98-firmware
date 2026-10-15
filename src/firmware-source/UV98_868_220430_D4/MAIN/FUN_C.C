#include<string.h>
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "UART1.H"
#include "UART2.H"
#include "UART4.H"

#include "IO.H"
#include "tostring.H"
#include "BEACON.H"
#include "AT24C512.h"
#include "KISS2ASC.H"
#include "FUN_C.h"
#include "GPS2.h"
#include "GPS_JULI.H"
#include "PUBLIC_BUF.H"
#include "KISS_Analysis.H"
#include  <string.H> // Keil library


uchar WX_BUF[50] ;		  // Buffer size
uchar WX_LINE1[30];
uchar WX_LINE2[30];


uchar read_hx(uchar *p)	   // Display 8 directions
{
    uint dat;

    if (*p == 0)
    {
        strcat(SN_RX_BUFFER, "--");
        return 0;
    }

    dat = (uint)(*p - 0x30) * 100 + (uint)(*(p + 1) - 0x30) * 10 + (uint)(*(p + 2) - 0x30);

    // List direction display mode 0=English 1=0-12 2-0-36
    angle_to_txt(dat, p);
    strcat(SN_RX_BUFFER, p);
    return 1;
}



void disp_wide_space(uchar len)
{
    uchar i;

    if(len == 0)
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        strcat(SN_RX_BUFFER, "------ --");
        strcat(SN_RX_BUFFER, ",");	 // Path
    }

}


void disp_wide()
{

    uint i, n;
    uchar temp[20];
    uchar dat;
    uchar idx;
    uchar path_count;

// BH4TDV-10>APET51,WIDE1-1:!3134.31N/12020.22Er090/050/A=003080 12.1V
// BH4TDV-6&gt;AP6688,BH4TDV-10*,WIDE1*:!3135.90N/12021.80E[668 9.7V 34.7C 1020.2hPa

    idx = 0;

    path_count = 0;

    for (i = 0; i < 20; i++)
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == ':')
        {
            disp_wide_space(5);
            return;
        }

        if (dat == ',')
        {
            break;   // Find the first comma
        }
    }


    for (n = 0; n < 5; n++) 		// Contains up to 5 paths
    {
        for (i = 0; i < 12; i++)
        {
            dat = ASC_TEMP[idx];
            idx++;

            if (dat == ':')
            {
                strcat(SN_RX_BUFFER, temp);
                strcat(SN_RX_BUFFER, ",");
                path_count++;
                disp_wide_space(5 - path_count);
                return;
            }

            if (dat == ',')
            {
                strcat(SN_RX_BUFFER, temp);	      // Find the fifth comma
                strcat(SN_RX_BUFFER, ",");
                path_count++;
                break;
            }

            temp[i] = dat;
            temp[i + 1] = 0;
        }
    }

// If there are more than 5 paths, they will be ignored.
// 
// for (i = 0; i<10; i++)
// {
// dat= ASC_TEMP[idx];   idx++;
// if (dat==':'){strcat(SN_RX_BUFFER,temp);	   strcat(SN_RX_BUFFER,",");	         return;}
// if (dat==&#39;,&#39;){strcat(SN_RX_BUFFER,temp); strcat(SN_RX_BUFFER,&quot;,&quot;); break;} //Find the 5th comma
// temp[i] =dat;	temp[i+1]=0;
// }
}

void FUN_C_KISS_TO_ASC(uint add)  // Read the stored KISS data, convert it into text, and store it in ASC_TEMP
{
    uint i;
    uchar dat;
    KISS_LEN = 0;	// Show KISS

    for (i = 0; i < 128; i++)
    {
        dat =	AT24CXX_READ(add + i + 128); 	 // KISS in the last 128 bytes

        if (dat == 0x00)
        {
            break;
        }

        KISS_DATA[i] = dat;
        KISS_LEN++;
    }

    KISS_TO_ASCII(ASC_TEMP, 0);	// KISS data conversion ASCII UI format, and get UI data length UI_DIGI_LEN
    // ==============================================
}


// //==============================================


void FUN_C_DISP_WX()  // Display weather board data
{
    uchar  i  ;
    uchar temp[10];
    float dat;	// With decimal point, positive and negative

    // --------------------------------------------
    // UART1_SendString(WX_BUF); //debugging
    // wxdata= "000/000g000t078r000p000h99b09966";
    // Extracted external interface board data, format: //000/000g000t078r000p000h99b09966
    // Extracted meteorological data, 32-byte length format: //c000s000g000t093r000p000h48b10016

    // --------------------------------------------
    WX_LINE1[0] = 0;	 	// 
    strcat(WX_LINE1, " ");	// 2 spaces

    // -------------------------------------------- //Wind direction
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[0 + i];
        temp[i + 1] = 0;
    }

    if (temp[0] == '0')		// The first 1-2 bits are blanked
    {
        temp[0] = ' '	;

        if (temp[1] == '0')  temp[1] = ' '	;
    }

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "  "); // 2 spaces

    // -------------------------------------------- //The average wind speed address in the previous minute is 4
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[4 + i];
        temp[i + 1] = 0;
    }

    // Wind speed, Imperial to Metric, mile/hour*1.61*1000/3600=M/S
    dat = (temp[0] - 0x30) * 100 + (temp[1] - 0x30) * 10 + (temp[2] - 0x30);
    // WD=30;
// WD=WD*1.61/3.6;
// temp[0]=(uint)WD/10%10+0X30 ;  temp[1]=(uint)WD%10+0X30 ;    temp[2] =0x00;
    dat = dat * 1.61 / 3.6 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    // The first 1 bit is blanked
    }

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "m/s  "); // 

    // -------------------------------------------- //Humidity
    for (i = 0; i < 2; i++)
    {
        temp[i] = WX_BUF[24 + i];
        temp[i + 1] = 0;
    }

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "% "); // 1 space

    // -------------------------------------------- //temperature
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[12 + i];
        temp[i + 1] = 0;
    }

    if (temp[0] == '-')
    {
        dat = -((float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30)) ;
    }
    else
    {
        dat = (float)(temp[0] - 0x30) * 100 + (float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30);
    }

    dat = (dat - 32) / 1.8 * 10;	// Fahrenheit to Celsius // Keep one decimal place

    if (dat < 0)	 // Below zero // Convert to positive number
    {
        dat = -dat;
        temp[0] = '-';
    }
    else
    {
        temp[0] = ' ';
    }

    temp[1] = (uint)dat / 100 % 10 + 0X30 ;
    temp[2] = (uint)dat / 10 % 10 + 0X30 ;
    temp[3] = '.' ;
    temp[4] = (uint)dat % 10 + 0X30 ;
    temp[5] = 0x00;

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "C"); // 

    // --------------------------------------------	 //
    strcat(WX_LINE1, ","); // 

    // -------------------------------------------- // Line 2
    // --------------------------------------------	 //


    // --------------------------------------------
    WX_LINE2[0] = 0;	 	// 

// strcat(WX_LINE2,&quot; &quot;); //1 space
    // -------------------------------------------- //Previous hour rainfall
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[16 + i];
        temp[i + 1] = 0;
    }

    // Rainfall, Imperial to Metric, 0.01 inches*0.254 mm
    dat	= (float)(temp[0] - 0x30) * 100 + (float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30);
    dat = dat * 0.254 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    // The first 1 bit is blanked
    }

    strcat(WX_LINE2, temp);
    strcat(WX_LINE2, "mm "); // 


    // --------------------------------------------
    // -------------------------------------------- //Previous 24 hours rainfall
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[20 + i];
        temp[i + 1] = 0;
    }

    // Rainfall, Imperial to Metric, 0.01 inches*0.254 mm
    dat	= (float)(temp[0] - 0x30) * 100 + (float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30);
    dat = dat * 0.254 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    // The first 1 bit is blanked
    }

    strcat(WX_LINE2, temp);
    strcat(WX_LINE2, "mm "); // 

    // --------------------------------------------

    // -------------------------------------------- //The highest instantaneous wind speed gust in the first 5 minutes
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[8 + i];
        temp[i + 1] = 0;
    }

    // Wind speed, Imperial to Metric, mile/hour*1.61*1000/3600=M/S
    dat = (temp[0] - 0x30) * 100 + (temp[1] - 0x30) * 10 + (temp[2] - 0x30);
    // WD=30;
// WD=WD*1.61/3.6;
// temp[0]=(uint)WD/10%10+0X30 ;  temp[1]=(uint)WD%10+0X30 ;    temp[2] =0x00;
    dat = dat * 1.61 / 3.6 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    // The first 1 bit is blanked
    }

    strcat(WX_LINE2, temp);
    strcat(WX_LINE2, " "); // //strcat(WX_LINE2,"m/s ");//

    // -------------------------------------------- //Air pressure has 5 digits
    for (i = 0; i < 5; i++)
    {
        temp[i] = WX_BUF[27 + i];
        temp[i + 1] = 0;
    }

    temp[5] = temp[4];
    temp[4] = '.';
    temp[6] = 0;	 // Add decimal point

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    // The first 1 bit is blanked
    }

    strcat(WX_LINE2, temp);
    // --------------------------------------------	 //
    strcat(WX_LINE2, ","); // 

    // --------------------------------------------
    // --------------------------------------------
    // --------------------------------------------
    // --------------------------------------------
}




void FUN_C_WX(uint add)
{
    uint i ;
    uchar dat;
    uchar idx;

    if (	AT24CXX_READ(add + 0X3F) != '_'	)
    {
        return;   // Icon // Weather beacon is handled separately
    }

// if ( AT24CXX_READ(add+0X17)!=&#39;1&#39; ){return;} // Icon//Weather beacon is processed separately
// if ( AT24CXX_READ(add+0X18)!=&#39;3&#39; ){return;} // Icon//Weather beacon is processed separately


    // Display custom information
    idx = 0;

    for (i = 0; i < 200; i++) 	// Retrieve weather icons
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == '_')
        {
            break;   // Contains weather icons
        }

        if (i > 198)
        {
            return;
        }
    }

    // Extracted meteorological data, 32-byte length format: //c000s000g000t093r000p000h48b10016
    for (i = 0; i < 32; i++)
    {
        WX_BUF[i] = ASC_TEMP[idx + i];
        WX_BUF[i + 1] = 0;
    }

    FUN_C_DISP_WX(); // Display weather board data


    strcat(SN_RX_BUFFER, "W,"); // Weather signs
    strcat(SN_RX_BUFFER, WX_LINE1); // Weather Line 1
    strcat(SN_RX_BUFFER, WX_LINE2); // Weather Line 2

// strcat(SN_RX_BUFFER,&quot; 120 1.3M/S 65% 17.3C,&quot;); // Weather display line 1
// 
// strcat(SN_RX_BUFFER,&quot; 0.0mm 0.0mm 0000 1000,&quot;); // Weather display line 3


}

// float WX_DATA[200]={&quot;c000s000g000t...r000p000h..b.....\r\n&quot;};

// c000(0-3)  s000(4-7)  g000(8-11)  t...(12-15)  r000(16-19)  p000(20-23)   h00(24-26)  b.....(27-32)

// Output data, format: c000s000g000t093r000p000h48b10016, including line break symbol, a total of 35 bytes
// float WX_DATA[40]={&quot;c000s000g000t032r000p000h00b.....\r\n&quot;};
// c = Report wind direction
// s = wind speed sustained in the last minute before the report (in miles per hour) 10 seconds cumulative sampling
// g = peak wind speed in the previous 5 minutes, miles per hour
// t = temperature in Fahrenheit
// r = rainfall in the past hour (in hundredths of an inch). 10-minute cumulative sampling
// p = the amount of rainfall in the past 24 hours (in hundredths of an inch).

// P = rainfall (in hundredths of an inch) since midnight. Not required
// h = humidity (00% = 100%). //Send DTH11 humidity data
// b = air pressure 0.1pa

// Reverse the direction of the vehicle from the north to the front of the vehicle.
// my_dir My heading (arrow direction), UI_Angle_N Due north (dot position) UI_Angle_CAR Relative direction to the front of the vehicle
void get_my_dir()
{
    uint my_dir;

    if (UI_Angle_N < UI_Angle_CAR)
    {
        my_dir = 360 - (UI_Angle_CAR - UI_Angle_N);
    }
    else
    {
        my_dir = UI_Angle_N - UI_Angle_CAR;
    }

}


// Callsign,Speed
void FUN_C_READ(uint add)	   // 
{
    uint i ;
    uchar temp[20];
    uchar dat;
    uchar idx;
    uchar no_wd_jd;

// if ( EEPROM_Buffer[0XB6]==1) {FUN_C_GPS( add); return;}
// 
    FUN_C_KISS_TO_ASC( add) ; // Read the stored KISS data, convert it into text, and store it in ASC_TEMP
    FUN_C_WX(add);	// Weather processing


// Call sign, speed, distance, altitude, latitude, NS, longitude, WE, other party&#39;s heading, relative north direction, relative vehicle head direction, date, time, path 1, path 2, path 3, path 4, path 5, custom information
    for (i = 0; i < 9; i++)
    {
        temp[i] = AT24CXX_READ(add + 16 + i);    // Call Sign
        temp[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

// for (i = 0; i<6; i++) { temp[i]=AT24CXX_READ(add+71+i); temp[i+1]=0; } 	  //速度
// if (temp[0]==0){strcat(SN_RX_BUFFER,"---.-");}else{strcat(SN_RX_BUFFER,temp);} strcat(SN_RX_BUFFER,"Km/h");  strcat(SN_RX_BUFFER,",");

    for (i = 0; i < 5; i++)
    {
        temp[i] = AT24CXX_READ(add + 0x48 + i);    // Speed, read only 5 bits
        temp[i + 1] = 0;
    }

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---.-");
    }
    else
    {
        for (i = 0; i < 2; i++)
        {
            if (temp[i] == '0')
            {
                temp[i] = ' ';   // Replace with spaces
            }
            else
            {
                break;
            }
        }

        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, "Kmh ");



    // ==============================================
    for (i = 0; i < 8; i++)
    {
        UI_WD[i] = AT24CXX_READ(add + 0x20 + i);      // latitude
        UI_WD[i + 1] = 0;
    }

    for (i = 0; i < 9; i++)
    {
        UI_JD[i] = AT24CXX_READ(add + 0x30 + i);      // latitude
        UI_JD[i + 1] = 0;
    }

    no_wd_jd = 1;

    if ((UI_WD[0] == 0) | (UI_JD[0] == 0) | (GPS_LOCKED == 0) )
    {
        no_wd_jd = 0;    // Check if the beacon contains latitude and longitude data
    }

// if(no_wd_jd==0) //Check if the beacon contains latitude and longitude data
// {
// 
// }

    for (i = 0; i < 3; i++)
    {
        temp[i] = AT24CXX_READ(add + 0x3a + i);    // The other party&#39;s heading
        temp[i + 1] = 0;
    }

    read_hx(temp) ;
    strcat(SN_RX_BUFFER, " ");


    if ( EEPROM_Buffer[0XB6] == 1)	 // If dynamic is turned on and longitude and latitude are included and GPS positioning is effective, the real-time distance will be calculated.
    {
        if(no_wd_jd == 1)
        {
            GET_AB_JULI(1);	  // MODE=1 Calculate the distance between the local GPS position and the other party, with an accuracy of 1 meter
            UI_Angle_N =  GET_AB_Angle(1); // Find the angle between the mobile station and the other party
            UI_Angle_CAR =   GET_AB_POINT_DIR(UI_Angle_N, GPS_NOW_DIR)	;	// Relative direction of vehicle head
        }
    }

    if ( EEPROM_Buffer[0XB6] == 1)		// Enable dynamic real-time calculation of the relative vehicle direction
    {

        if(no_wd_jd == 1)	 // Check if the beacon contains latitude and longitude data
        {
            tostring(UI_Angle_CAR);
            temp[0] = bai;
            temp[1] = shi;
            temp[2] = ge;
            temp[3] = 0;		 // Relative position of vehicle head
            read_hx(temp);
        }
        else
        {
            strcat(SN_RX_BUFFER, "--");
        }

        strcat(SN_RX_BUFFER, ",");
        // ==============================================

        if(no_wd_jd == 1)					   	// Distance display 6 digits
        {
            strcat(SN_RX_BUFFER, UI_JULI + 1);	 // Display real-time distance
        }
        else
        {
            strcat(SN_RX_BUFFER, "---.-");	   // If there is no distance, it will display ----.-
        }

        strcat(SN_RX_BUFFER, "Km,");

    }
    else
    {

        for (i = 0; i < 3; i++)
        {
            temp[i] = AT24CXX_READ(add + 0x1D + i);    // Relative position of vehicle head
            temp[i + 1] = 0;
        }

        read_hx(temp) ;
        strcat(SN_RX_BUFFER, ",");

        // ==============================================
        for (i = 0; i < 6; i++)
        {
            temp[i] = AT24CXX_READ(add + 41 + i);      // Distance display 6 digits
            temp[i + 1] = 0;
        }

        switch (temp[0])
        {
            case 0:
                strcat(SN_RX_BUFFER, "----.-");
                strcat(SN_RX_BUFFER, "Km,");	   	// If there is no distance, it will display ----.-
                break;

            case '*':	  // Less than 10,000 meters
                temp[0] = ' ';
                strcat(SN_RX_BUFFER, temp + 1);
                strcat(SN_RX_BUFFER, "Km,");
                break;

            default:
                strcat(SN_RX_BUFFER, temp + 1);
                strcat(SN_RX_BUFFER, "Km,");
                break;
        }

    }



    // ==============================================


// for (i = 0; i<6; i++) { temp[i]=AT24CXX_READ(add+64+i); temp[i+1]=0; } 	  //海拔
// if (temp[0]==0){strcat(SN_RX_BUFFER,"------");}else{strcat(SN_RX_BUFFER,temp);}  strcat(SN_RX_BUFFER,"m");  strcat(SN_RX_BUFFER,",");

    for (i = 0; i < 6; i++)
    {
        temp[i] = AT24CXX_READ(add + 64 + i);
        temp[i + 1] = 0;
    }

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "------");
    }
    else	    // altitude
    {
        for (i = 0; i < 5; i++)
        {
            if (temp[i] == '0')
            {
                temp[i] = ' ';   // Replace with spaces
            }
            else
            {
                break;
            }
        }

        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, "M");
    strcat(SN_RX_BUFFER, ",");


    // ==============================================
    strcat(SN_RX_BUFFER, " ");
    temp[0] = AT24CXX_READ(add + 0x20);
    temp[1] = AT24CXX_READ(add + 0x21);
    temp[2] = '@';	// strcat(SN_RX_BUFFER,temp);		strcat(SN_RX_BUFFER,"@");			//纬度
    temp[3] = AT24CXX_READ(add + 0x22);
    temp[4] = AT24CXX_READ(add + 0x23);
    temp[5] = '.';	// strcat(SN_RX_BUFFER,temp);		strcat(SN_RX_BUFFER,".");
    temp[6] = AT24CXX_READ(add + 0x25);
    temp[7] = AT24CXX_READ(add + 0x26);
    temp[8] = '\'';	// strcat(SN_RX_BUFFER,temp);		strcat(SN_RX_BUFFER,"'");
// strcat(SN_RX_BUFFER," ");
    temp[9] = AT24CXX_READ(add + 0x27);
    temp[10] = 0;	 // NS

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "--@--.--'-");
    }
    else
    {
        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, ",");
    // ==============================================
    temp[0] = AT24CXX_READ(add + 0x30);
    temp[1] = AT24CXX_READ(add + 0x31);
    temp[2] = AT24CXX_READ(add + 0x32);
    temp[3] = '@';
    temp[4] = AT24CXX_READ(add + 0x33);
    temp[5] = AT24CXX_READ(add + 0x34);
    temp[6] = '.';
    temp[7] = AT24CXX_READ(add + 0x36);
    temp[8] = AT24CXX_READ(add + 0x37);
    temp[9] = '\'';
    temp[10] = AT24CXX_READ(add + 0x38);
    temp[11] = 0;	 // WE

// for (i = 0; i<3; i++) { temp[i]=AT24CXX_READ(add+0x30+i); temp[i+1]=0; } 	strcat(SN_RX_BUFFER,temp);	   	strcat(SN_RX_BUFFER,"@");	  	//经度
// for (i = 0; i<2; i++) { temp[i]=AT24CXX_READ(add+0x33+i); temp[i+1]=0; } 	strcat(SN_RX_BUFFER,temp);	   	strcat(SN_RX_BUFFER,".");
// for (i = 0; i<2; i++) { temp[i]=AT24CXX_READ(add+0x36+i); temp[i+1]=0; } 	strcat(SN_RX_BUFFER,temp);	   	strcat(SN_RX_BUFFER,"'");
// strcat(SN_RX_BUFFER," ");
// temp[0]=AT24CXX_READ(add+0x38); temp[1]=0;

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---@--.--'-");
    }
    else
    {
        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, ",");
    // ==============================================

    for (i = 0; i < 3; i++)
    {
        temp[i] = AT24CXX_READ(add + 0x3a + i);    // The other party&#39;s heading
        temp[i + 1] = 0;
    }

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---");
    }
    else
    {
        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, ",");



    if ( EEPROM_Buffer[0XB6] == 1)		// Enable dynamic real-time calculation of the relative vehicle direction
    {

        if(no_wd_jd == 1)	 // Check if the beacon contains latitude and longitude data
        {
            tostring(UI_Angle_N);				 // Relative North
            temp[0] = bai;
            temp[1] = shi;
            temp[2] = ge;
            temp[3] = 0;
            strcat(SN_RX_BUFFER, temp);
        }
        else
        {
            strcat(SN_RX_BUFFER, "---");
        }

        strcat(SN_RX_BUFFER, ",");

        if(no_wd_jd == 1)	 // Check if the beacon contains latitude and longitude data
        {
            tostring(UI_Angle_CAR);	  // Relative position of vehicle head
            temp[0] = bai;
            temp[1] = shi;
            temp[2] = ge;
            temp[3] = 0;
            strcat(SN_RX_BUFFER, temp);
        }
        else
        {
            strcat(SN_RX_BUFFER, "---");
        }

        strcat(SN_RX_BUFFER, ",");
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            temp[i] = AT24CXX_READ(add + 0x1a + i);    // Relative North
            temp[i + 1] = 0;
        }

        if (temp[0] == 0)
        {
            strcat(SN_RX_BUFFER, "---");
        }
        else
        {
            strcat(SN_RX_BUFFER, temp);
        }

        strcat(SN_RX_BUFFER, ",");

        for (i = 0; i < 3; i++)
        {
            temp[i] = AT24CXX_READ(add + 0x1D + i);    // Relative position of vehicle head
            temp[i + 1] = 0;
        }

        if (temp[0] == 0)
        {
            strcat(SN_RX_BUFFER, "---");
        }
        else
        {
            strcat(SN_RX_BUFFER, temp);
        }

        strcat(SN_RX_BUFFER, ",");
    }


    for (i = 0; i < 8; i++)
    {
        temp[i] = AT24CXX_READ(add + 8 + i);    // date
        temp[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    for (i = 0; i < 8; i++)
    {
        temp[i] = AT24CXX_READ(add + 0 + i);    // time
        temp[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    // ==============================================

// KISS_LEN =0; //Display KISS
// for (i = 0; i<128; i++)
// { dat= AT24CXX_READ(add+i+128); //KISS in the last 128 bytes
// if (dat==0x00){break;}
// KISS_DATA[i]=data; KISS_LEN++;
// }
// KISS_TO_ASCII(ASC_TEMP,0); //Convert KISS data to ASCII UI format and get UI data length UI_DIGI_LEN
// //==============================================

// FUN_C_KISS_TO_ASC( add) ; //Read the stored KISS data, convert it into text, and store it in ASC_TEMP

    disp_wide();	   	// Show Path
    // ==============================================
    // Display custom information
    idx = 0;

    for (i = 0; i < 200; i++) 	// Searching for colon
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == ':')
        {
            break;
        }

        if (i > 198)
        {
            strcat(SN_RX_BUFFER, "----,");
            return;
        }
    }

    for (i = 0; i < 200 - idx; i++) 	// Analyze the remaining characters
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == 0x00)
        {
            break;
        }

        if (dat == 0x0D)
        {
            break;
        }

        if ((dat > '~') | (dat < ' '))
        {
            dat = 'x';
        }

        ASC_TEMP[i] = dat;
        ASC_TEMP[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, ASC_TEMP);
    strcat(SN_RX_BUFFER, ",");
}

















void DISP_A08()
{
    uint  add;
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$A08,");
    add =  AT24CXX_READ(0x6500 ) * 256; // The list starts at address 0X6500, and the index of the first row of KISS data storage is obtained.
    FUN_C_READ(add);
    strcat(SN_RX_BUFFER, "\r\n");

    UART4_SendString(SN_RX_BUFFER);	   // UART2_SendString(SN_RX_BUFFER);
}



uchar FUN_C(uint idx)	   // Detailed beacon
{
    uint  add;
    uchar temp[8];	  // uint i;

    if (idx > 99)
    {
        return 0;
    }

    // ================================================= Insert index numbers 00-99
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$C");
    temp[0] = idx / 10 + 0x30;
    temp[1] = idx % 10 + 0x30;
    temp[2] = 0;
    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");
    // ==============================================

    add =  AT24CXX_READ(0x6500 + idx ) * 256; // The list starts at address 0X6500, and the KISS data storage index is obtained.

    if (AT24CXX_READ(add) == 0xff)
    {
        strcat(SN_RX_BUFFER, "NO DATA\r\n");		   // 24C512 did not write data
    }
    else
    {
        FUN_C_READ(add);
        strcat(SN_RX_BUFFER, "\r\n");
    }

    UART4_SendString(SN_RX_BUFFER);	   // UART2_SendString(SN_RX_BUFFER);

// for (i = 0; i<256; i++) { UART2_SendData(AT24CXX_READ(add+0+i));   } 	  //调试
    return 0;
}

// 0x0000-0x6400 (0-25600), each beacon occupies 256 bytes, a total of 100 records, index 0-99 is stored at 0x8000

// 0x6500-0x6600 , list storage area, one set of indexes per 1 byte, a total of 100 indexes
// 0-5 bytes call sign BH4TDV
// 6-byte SSID-XX 99
// 7-byte beacon KISS storage address index 0-99

