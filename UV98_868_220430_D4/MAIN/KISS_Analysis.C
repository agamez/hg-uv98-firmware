#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
// #include "HMI.H"

// #include "KISS2ASC.H"
#include "MICE_DECODE.H"
#include "UART1.H"
#include "GPS2.H"
#include "GPS_JULI.H"
#include "tostring.H"

#include "KISS_Analysis.H"

// #include "BEACON.H"
#include "KISS2ASC.H"
#include "BT.H"

#include "BEACON_SAVE.H"
#include "PUBLIC_BUF.H"



uchar UI_CALL[10];
// float UI_SSID[3];
uchar UI_DEST[10];
uchar UI_TIME[10];
uchar UI_WD[10];
uchar UI_WD_DIR;
uchar UI_JD[10];
uchar UI_JD_DIR;
uchar UI_ICON;

uchar UI_DIR[10];
uchar UI_SPEED[10];
uchar UI_ALT[10];
uchar UI_JULI[10];

uint UI_Angle_N;	// Relative North
uint UI_Angle_CAR;  // Relative position of vehicle head

float UI_ALT_MI;	 // Beacon altitude

// fly W25Q64_BUF_A[128];


uint Elevation_angle; // Relative elevation angle


// void CLEAN_UI_DATA();

// uchar CONV_JD( uint JD); //angle pointing






void CLEAN_UI_DATA()
{
    uchar i;

    for(i = 0; i < 10; i++)
    {
        UI_CALL[i] = 0x00;
    }

    for(i = 0; i < 10; i++)
    {
        UI_WD[i] = 0x00;   // 8 bytes
    }

    for(i = 0; i < 10; i++)
    {
        UI_JD[i] = 0x00;   // 9 bytes
    }

    for(i = 0; i < 10; i++)
    {
        UI_DIR[i] = 0x00;   // Heading 3 bytes
    }

    for(i = 0; i < 10; i++)
    {
        UI_SPEED[i] = 0x00;   // Speed 6 bytes
    }

    for(i = 0; i < 10; i++)
    {
        UI_ALT[i] = 0x00;   // 6 bytes
    }

    for(i = 0; i < 10; i++)
    {
        UI_JULI[i] = 0x00;   // 6 bytes
    }

    UI_Angle_N = 0;	 // Relative North
    UI_Angle_CAR = 0; // Relative position of vehicle head
    UI_ALT_MI = 0;	 // Beacon altitude
    Elevation_angle = 0; // Relative elevation angle

// for(i=0;i<10;i++){W25Q64_BUF_A[i]=0x00;}

    for(i = 0; i < 10; i++)
    {
        UI_DEST[i] = 0x00;
    }

    for(i = 0; i < 10; i++)
    {
        UI_TIME[i] = 0x00;   // 
    }
}





void SPEED_MI()	 // Speed nautical miles to metric meters
{
    unsigned long SPEED;
    uchar i;	  // flying i,n;
// uint	txt;

    for(i = 0; i < 3; i++)
    {
        UI_SPEED[i] = UI_SPEED[i] - 0X30;   // Convert text to numbers
    }

    SPEED = (long)UI_SPEED[0] * 100 + UI_SPEED[1] * 10 + UI_SPEED[2];
    SPEED = SPEED * 1.852 * 10;	// Convert to meters, 0.1KM

// txt=(uint)SPEED;
// UI_SPEED[0]=txt/10000+0x30;
// UI_SPEED[1]=txt/1000%10+0x30;
// UI_SPEED[2]=txt/100%10+0x30;
// UI_SPEED[3]=txt/10%10+0x30;
// UI_SPEED[4]='.';
// UI_SPEED[5]=txt%10+0x30;
// UI_SPEED[6]=0x00;

    tostring((uint)SPEED);
    UI_SPEED[0] = wan;
    UI_SPEED[1] = qian;
    UI_SPEED[2] = bai;
    UI_SPEED[3] = shi;
    UI_SPEED[4] = '.';
    UI_SPEED[5] = ge;
    UI_SPEED[6] = 0x00;


// n=0;
// while(UI_SPEED[0]==&#39;0&#39;) //The first 0 is blanked, at least one bit is retained
// {
// for(i=0;i<7;i++) 	{ UI_SPEED[i]=UI_SPEED[i+1]; }
// n++; if (n>2){break;}
// }
}



void ALT_MI()	 // Convert English to Metric meters
{
    unsigned long temp_ALT;
    // uchar GPS_ALT_ZERO; //less than 0 sign
    // uchar GPS_ALT_ERR; //GPS has no valid positioning, no altitude data, so the data is invalid
    uchar i;		 // flying i,n;
    uint txt;

    if (UI_ALT[0] == '-')
    {
        for(i = 1; i < 6; i++)
        {
            UI_ALT[i] = UI_ALT[i] - 0X30;   // Convert text to numbers
        }

        temp_ALT = (long)UI_ALT[1] * 10000 + (long)UI_ALT[2] * 1000 + (long)UI_ALT[3] * 100 + UI_ALT[4] * 10 + UI_ALT[5];
        temp_ALT = temp_ALT * 0.3048;	// Convert to meters

        UI_ALT_MI = 0; // Below sea level, the beacon altitude is set to 0

// txt=(uint)GPS_ALT;
// UI_ALT[1]=txt/10000+0x30;
// UI_ALT[2]=txt/1000%10+0x30;
// UI_ALT[3]=txt/100%10+0x30;
// UI_ALT[4]=txt/10%10+0x30;
// UI_ALT[5]=txt%10+0x30;
// UI_ALT[6]=0x00;

        tostring((uint)temp_ALT);
        UI_ALT[1] = wan;
        UI_ALT[2] = qian;
        UI_ALT[3] = bai;
        UI_ALT[4] = shi;
        UI_ALT[5] = ge;
        UI_ALT[6] = 0x00;

// n=0;
// while(UI_ALT[1]==&#39;0&#39;) //The first 0 is blanked, at least one bit is retained
// {
// for(i=1;i<7;i++) 	{ UI_ALT[i]=UI_ALT[i+1]; }
// n++; if (n>3){break;}
// }

    }
    else
    {
        for(i = 0; i < 6; i++)
        {
            UI_ALT[i] = UI_ALT[i] - 0X30;	   // Convert text to numbers
        }

        temp_ALT = (long)UI_ALT[0] * 100000 + (long)UI_ALT[1] * 10000 + (long)UI_ALT[2] * 1000 + (long)UI_ALT[3] * 100 + UI_ALT[4] * 10 + UI_ALT[5];
        temp_ALT = temp_ALT * 0.3048;	// Convert to meters

        UI_ALT_MI = (float)temp_ALT;	 // Beacon altitude

        txt = (uint)temp_ALT;
        UI_ALT[0] = txt / 100000 + 0x30;
        UI_ALT[1] = txt / 10000 % 10 + 0x30;
        UI_ALT[2] = txt / 1000 % 10 + 0x30;
        UI_ALT[3] = txt / 100 % 10 + 0x30;
        UI_ALT[4] = txt / 10 % 10 + 0x30;
        UI_ALT[5] = txt % 10 + 0x30;
        UI_ALT[6] = 0x00;


// n=0;
// while(UI_ALT[0]==&#39;0&#39;) //The first 0 is blanked, at least one bit is retained
// {
// for(i=0;i<7;i++) 	{ UI_ALT[i]=UI_ALT[i+1]; }
// n++; if (n>4){break;}
// }
    }
}





unsigned char READ_UI_indx(uchar *p)	 // Search for colon-starting
{
    unsigned char  k ;

    for (k = 0; k < 125; k++)		 // Search up to 125
    {
        if (*p  == ':')
        {
            return k + 1;	   // Return data starting address
        }

        p++;
    }

    return 0; // Retrieval Error
}

void READ_UI_CALL(uchar *p)	 // Callsign, ID, SOURCE
{
    uchar i, txt;

    for(i = 0; i < 10; i++)
    {
        txt = *p;
        p++;

        if (txt == '>')
        {
            break;
        }

        UI_CALL[i] = txt ;
    }

    for(i = 0; i < 10; i++)
    {
        txt = *p;
        p++;

        if ((txt == ':') | (txt == ','))
        {
            break;
        }

        UI_DEST[i] = txt ;
    }
}

// BG8TFC-12>AP51G2:!2502.07N/10134.94E>167/029/A=005917APRS 51Track 51G2 Tel 13577841761 14.6V S06


void Resolution_UI_A(uchar *p, uchar idx)	  // Parse ! = Type Data
{


    uchar i, txt;

    p += idx;

    txt = *p;
    p++;

    if ((txt == '/') | (txt == '@'))
    {
        for(i = 0; i < 7; i++)
        {
            UI_TIME[i] = *p;	   // Parsing/@ with time and date type data
            p++;
        }
    }

// BH4TDV-7&gt;APUV98,WIDE1-1:!3133.90S/12022.80E[HG-UV98 9.7V 32.0C 1018.1hPa
    for(i = 0; i < 8; i++)
    {
        UI_WD[i] = *p;	   // latitude
        p++;
    }

    UI_WD_DIR = UI_WD[7];  	 // NS


    p++;		// Skip Icon Set /

    for(i = 0; i < 9; i++)
    {
        UI_JD[i] = *p;
        p++;
    }

    UI_JD_DIR = UI_JD[8];   	 // WE





    UI_ICON = *p;
    p++;	  // icon

    // Determine whether it is weather station data
    if (UI_ICON == '_')
    {
        return;
    }	// 

    // Determine whether there is speed, heading and altitude data


    UI_ALT_MI = 0;	 // Beacon altitude

    if (*(p + 3) == '/')
    {
        for(i = 0; i < 3; i++)
        {
            UI_DIR[i] = *p;	   // course
            p++;
        }

        p++;

        for(i = 0; i < 3; i++)
        {
            UI_SPEED[i] = *p;	   // speed
            p++;
        }

        SPEED_MI();	 // Speed nautical miles to metric meters

        if ((*p == '/') && (*(p + 1) == 'A') && (*(p + 2) == '=')) // Determine whether there is altitude data
        {
            p++;
            p++;
            p++;

            for(i = 0; i < 6; i++)
            {
                UI_ALT[i] = *p;	   // Altitude Imperial
                p++;
            }

            ALT_MI();	 // Convert English to Metric meters
        }
    }
    else  // No direction
    {


    }

    // Customized information
// i=0;
// while(*p!=0x00)
// {UI_MSG[i]=*p; p++; i++;
// if (i&gt;70){UI_MSG[i]=0x00; break;} //Limit length
// }

}


// Common data formats
// BG8TFC-12>AP51G2:!2502.07N/10134.94E>167/029/A=005917APRS 51Track 51G2 Tel 13577841761 14.6V S06
// BH1PEZ-7>SYUUQ1,WIDE1-1,WIDE2-1:`,<|l+Z[/`"4D}Welcome to QSO with me!._(
// BG6KOY-5&gt;APDR13,TCPIP*::BH6MCZ-11: Can you receive it? {1
// BH7PCT-15>APLM3D,TCPIP*:@111031h2304.63N/11321.44E>246/009/A=000042http://www.aprs.cn G07 M17 11.4V
// BH4TDV-6>AP51TT:>HELLO 14.30V 15.6C
uchar Resolution_UI_DATA()	 // Parsing UI data types
{
    uchar idx;

    READ_UI_CALL(SN_RX_BUFFER);	 // Callsign, ID, SOURCE


    idx = READ_UI_indx(SN_RX_BUFFER); // The first colon of the retrieved data

    if (idx == 0)
    {
        return 0;   // Retrieval Error
    }

    switch (SN_RX_BUFFER[idx]) // Determine the data type symbol
    {
        case '!':	// A without time
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;

        case '=':	// A without time
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;


        case '/':	 // B with time
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;

        case '@':	 // B with time
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;

        case 0x27:	// C compression
            return 2;
            break;

        case 0x60:	// C compression
            return 2;
            break;

        case ':':	// D Information
            return 3;
            break;

        case '>':	 // state
            return 4;
            break;


        default:   // GUnknown
            return 5;
            break;
    }

    return 1;
}




void GET_JULI_DIR_EL()	  // Find the distance, relative north direction, relative vehicle head direction
{
    if (EEPROM_Buffer[0x002a] == 0)	 // 0 = Fixed station, set longitude and latitude 1 = Mobile station, GPS longitude and latitude
    {
        GET_AB_JULI(0);			   // Find the distance between the fixed station and the other two locations
        UI_Angle_N =	GET_AB_Angle(0); // Find the angle between the fixed station and the other two places
        UI_Angle_CAR = UI_Angle_N; // Relative direction of vehicle head

        Elevation_angle = GET_AB_EL(0); // Relative elevation angle
    }
    else
    {
        if (GPS_LOCKED == 1)
        {
            GET_AB_JULI(1);			   // Find the distance between the mobile station and the other party
            UI_Angle_N =	GET_AB_Angle(1); // Find the angle between the mobile station and the other party
// UI_Angle_CAR=UI_Angle_N+(360-GPS_NOW_DIR); //Relative direction of the vehicle head

// if (UI_Angle_N&gt;GPS_NOW_DIR) {UI_Angle_CAR=UI_Angle_N-GPS_NOW_DIR;} //Relative direction of the front of the vehicle
// else	   { UI_Angle_CAR=360-(GPS_NOW_DIR-UI_Angle_N);	 }

            UI_Angle_CAR =   GET_AB_POINT_DIR(UI_Angle_N, GPS_NOW_DIR)	;	// Relative direction of vehicle head

            Elevation_angle = GET_AB_EL(1); // Relative elevation angle
        }
    }
}

void DISP_KISS_DATA()   // Analyze and display the other party&#39;s positioning data, and display
{
    uchar STU;
    CLEAN_UI_DATA();

    if (KISS_TO_MICE() == 1)
    {
        GET_JULI_DIR_EL();	   // Analyze the compressed data, decompose the call sign, ID, SOURCE to analyze the latitude and longitude, heading, speed, altitude, information
        BEACON_SAVE();
        BT_OUT(1);
        return;
    }

    KISS_TO_ASCII(SN_RX_BUFFER, 0);	// KISS to ASC text
    STU = Resolution_UI_DATA();  	// status 1= uncompressed regular data format 2=`compressed 3=: short message 4=&gt; status 5= unknown


    if (STU == 0x01)
    {
        GET_JULI_DIR_EL();    // According to the general data analysis, decompose the call sign, ID, SOURCE to analyze the latitude and longitude, heading, speed, altitude, information
        BEACON_SAVE();
        BT_OUT(1);
    }
    else
    {
        BEACON_SAVE();    // Unknown format, prohibit output of waypoints, parse as normal data, decompose call sign, ID, SOURCE to parse latitude and longitude, heading, speed, altitude, information
        BT_OUT(0);
    }

}




// 
// Write beacon list and store beacons

// flying code KISS_DEMO[]=
// {
// 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, // Target address + SSID
// 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF2, // Source address + SSID
// 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, // Path + SSID
// 0x03, 0xF0, 											//	03f0
// 0x21, // Type
// 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, // longitude and latitude
// 0x2F, // separator &quot;/&quot;
// 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, // longitude and latitude
// 0x3E, // Icon type
// 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74, // information
// };

