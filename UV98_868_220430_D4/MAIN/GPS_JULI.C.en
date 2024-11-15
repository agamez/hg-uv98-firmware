// Find the distance between 2 points in longitude and latitude
#include <math.h>


#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "PUBLIC_BUF.H"

#include "GPS_JULI.H"
#include "GPS2.H"
#include "KISS_Analysis.H"
#include "tostring.H"

#include "UART2.H"


#define   ulong unsigned long


#define pi 3.1415926
#define EARTH_RADIUS 6378.137


double GetDistance(double lat1, double lng1, double lat2, double lng2) ;
float GetAngle(float x1, float y1, float x2, float y2) ;
// void GPS_DMM_D(uchar mode); //Convert text to degrees

double my_lat, my_lng, you_lat, you_lng;


double Distance_AB;


double rad(double d)
{
    return (d * pi / 180.0);
}



double GET_WD(uchar *lat)	  	// Convert latitude and longitude, text, degrees to minutes
{
    uchar i;
    double lat_degree, lat_min; // Degree, minute
    unsigned char temp[10];

    for (i = 0; i < 8; i++)
    {
        temp[i] = lat[i];
    }

    lat_min = (double)(temp[2] - 0x30) * 10 + (double)(temp[3] - 0x30) + (double)(temp[5] - 0x30) * 0.1 + (double)(temp[6] - 0x30) * 0.01;
    lat_degree =	(double)(temp[0] - 0x30) * 10 + (double)(temp[1] - 0x30) +	(lat_min / 60) ;

    // North latitude N is positive, south latitude S is negative, east longitude E is positive, west longitude W is negative,
    if (temp[7] == 'S')
    {
        lat_degree = -lat_degree;
    }

    return  lat_degree;
}

double GET_JD(uchar *lng)	 	// Convert latitude and longitude, text, degrees to minutes
{
    uchar i;
    double lng_degree, lng_min; // Degree, minute
    unsigned char temp[10];

    for (i = 0; i < 9; i++)
    {
        temp[i] = lng[i];
    }

    lng_min = (double)(temp[3] - 0x30) * 10 + (double)(temp[4] - 0x30) + (double)(temp[6] - 0x30) * 0.1 + (double)(temp[7] - 0x30) * 0.01;
    lng_degree = (double)(temp[0] - 0x30) * 100 + (double)(temp[1] - 0x30) * 10 + (double)(temp[2] - 0x30) +	(lng_min / 60) ;

    // North latitude N is positive, south latitude S is negative, east longitude E is positive, west longitude W is negative,
    if (temp[8] == 'W')
    {
        lng_degree = -lng_degree;
    }

    return 	lng_degree;
}

// Convert degrees to minutes
// A=31.3401N 120.2021E i.e. 31.566833 120.336833
// B=31.3400N 120.2020E i.e. 31.566666 120.336666

// AB=24.3m

// Latitude measured northward from the Earth&#39;s equatorial plane. Symbol N, north latitude 0° to 90°. S, south latitude, 0-90 degrees
// East longitude is indicated by &quot;E&quot; and West longitude is indicated by &quot;W&quot;. Range 0-180 degrees
// Convert latitude and longitude to degrees
// 1 degree is equal to 60 minutes, 1 minute is equal to 60 seconds
// Latitude temp // Longitude longitude
// North latitude N is positive, south latitude S is negative, east longitude E is positive, west longitude W is negative,
// GetDistance(Latitude of point A, Longitude of point A, Latitude of point B, Longitude of point B)

double GetDistance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = rad(lng1) - rad(lng2);

    double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));

    s = s * EARTH_RADIUS;
    // The round method means &quot;rounding off&quot;. The algorithm is Math.floor(x+0.5), which means adding 0.5 to the original number and then rounding it down.
    // s = Math.Round(s * 10000) / 10000;	 //
// s = floor((s * 10000)+0.5) / 10000; // Unit KM
// s = floor((s * 10000)+0.5) / 10000; // Unit 1KM 1000 meters
// s = floor((s * 10000)+0.5) / 1000 ; // Unit 0.1KM 100 meters
// s = floor((s * 10000)+0.5) / 100 ; // Unit 0.01KM 10 meters
    s = floor((s * 10000) + 0.5) / 10 ;		 // Unit 0.001KM 1 meter

    return s;	// 0.02426 KM is approximately equal to 24.3 meters when rounded off.
}


double GET_JULI(uchar *a_lat, uchar *a_lng, uchar *b_lat, uchar *b_lng )
{
    // double JL,DF;
    // --------------------------------------------

// my_lat = GET_WD(&quot;3134.00N&quot;); //Convert point A&#39;s latitude, text, degrees to minutes
// 
// //--------------------------------------------
// my_lng = GET_JD(&quot;12020.20W&quot;); //Convert point A&#39;s longitude, text, degrees to minutes
// //--------------------------------------------
// you_lat = GET_WD(&quot;3134.00N&quot;); //Convert the latitude of point B, text, degrees to minutes
// //--------------------------------------------
// you_lng = GET_JD(&quot;12020.20W&quot;); //Convert the longitude of point B, text, degrees to minutes
// 
// 
// JL=  GetDistance(my_lat, my_lng, you_lat, you_lng);
// DF=JL;

// UART2_SendString(a_lat); UART2_SendString(a_lng);
// UART2_SendString(b_lat); UART2_SendString(b_lng);


    my_lat = GET_WD(a_lat);	// Convert point A&#39;s latitude, text, degrees to degrees
    // --------------------------------------------
    my_lng = GET_JD(a_lng); // Convert point A&#39;s longitude, text, degrees to minutes
    // --------------------------------------------
    you_lat = GET_WD(b_lat); // Convert the latitude of point B, text, degrees to degrees
    // --------------------------------------------
    you_lng = GET_JD(b_lng);	// Convert the longitude of point B, text, degrees to minutes
    // --------------------------------------------
// JL= GetDistance(my_lat, my_lng,you_lat, you_lng)*10; //Accuracy 100 meters, 0.0243 KM is approximately equal to 0.0 KM
// JL= GetDistance(my_lat, my_lng,you_lat, you_lng)*100; //Accuracy 10 meters, 0.0243 KM is approximately equal to 0.02 KM
// JL= GetDistance(31.566833, 120.336833,31.566666, 120.336666)*1000; //Accuracy 1 meter, 0.0243 KM is approximately equal to 0.024 KM
// JL= GetDistance(31.566833, 120.336833,31.566666, 120.336666)*10000; //Precision 0.1 meter, 0.0243 KM is approximately equal to 24.3 meters

// JL= GetDistance(-31.114921, -120.336833,-31.114921, -120.336666);
// 
// DF=JL;


    return GetDistance(my_lat, my_lng, you_lat, you_lng);	  // The accuracy is 1 meter, 0.0243 KM is approximately equal to
}







// void GPS_DMM_D(uchar mode) //Convert text to degrees
// {
// //MODE=0 Find the distance between the fixed station and the other two locations
// //MODE=1 Find the distance between the mobile station and the other party
// //MODE=2 Find the distance between GPS and fixed station
// 
// //Convert the latitude and longitude of point A, text, degrees to minutes
// if (mode==1) //0=fixed station, set longitude and latitude 1=mobile station, GPS longitude and latitude
// { my_lat = GET_WD(GPS_WD); //Own GPS latitude
// my_lng = GET_JD(GPS_JD); //Own GPS longitude
// }
// else
// { my_lat = GET_WD(EEPROM_Buffer+0x20); //Fixed station latitude
// my_lng = GET_JD(EEPROM_Buffer+0x30); //Fixed station longitude
// }
// //--------------------------------------------
// //Convert the latitude and longitude of point B, text, degrees to minutes
// if (mode==2)
// { you_lat = GET_WD(GPS_WD); // your own GPS latitude
// you_lng = GET_JD(GPS_JD); //your own GPS longitude
// }
// else
// { you_lat= GET_WD(UI_WD); //Get the other party&#39;s moving latitude
// you_lng = GET_JD(UI_JD); //Get the other party&#39;s longitude
// }
// //--------------------------------------------
// }
// 
// 





void GET_AB_JULI(uchar mode)
{
    unsigned int JL;
    unsigned char i;	// unsigned char  n;

    // --------------------------------------------
    // MODE=0 Find the distance between the fixed station and the other party, with an accuracy of 1 meter
    if (mode == 0)
    {
        Distance_AB = GET_JULI(EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30, UI_WD, UI_JD);
    }

    // MODE=1 Find the distance between the mobile station and the other party, with an accuracy of 1 meter
    if (mode == 1)
    {
        Distance_AB = GET_JULI(GPS_WD, GPS_JD, UI_WD, UI_JD);
    }

    // MODE=2 Find the distance between GPS and fixed station, with an accuracy of 1 meter
    if (mode == 2)
    {
        Distance_AB = GET_JULI(EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30, GPS_WD, GPS_JD);
    }


// Distance_AB = GET_JULI(&quot;3134.32N&quot;, &quot;12020.01E&quot;,&quot;3134.32N&quot;, &quot;12020.01E&quot;); //wd 0.01 = 18.2 meters JD 15.5 meters

    for(i = 0; i < 8; i++)
    {
        UI_JULI[i] = 0x00;
    }

    if (Distance_AB < 10000)	 // Less than 10,000 meters, display meters
    {
        JL = (uint)(Distance_AB);
        tostring(JL) ;
        UI_JULI[0] = '*';
        UI_JULI[1] = qian;
        UI_JULI[2] = '.';
        UI_JULI[3] = bai;
        UI_JULI[4] = shi;
        UI_JULI[5] = ge;
        UI_JULI[6] = 0x00;


// for(i=0;i<4;i++) 	  //0替换成空格
// {
// if (UI_JULI[i+1]=='0'){UI_JULI[i+1]=' ';}else{break;}
// }

    }
    else
    {
        JL = (uint)(Distance_AB / 100); // The accuracy is adjusted to 10 meters, 0.0243 KM is approximately equal to 24 meters

        tostring(JL) ;

        UI_JULI[0] = wan;
        UI_JULI[1] = qian;
        UI_JULI[2] = bai;
        UI_JULI[3] = shi;
        UI_JULI[4] = '.';
        UI_JULI[5] = ge;
        UI_JULI[6] = 0x00;

        // n=0;
        // while(UI_JULI[0]==&#39;0&#39;) //The first 0 is blanked
        // {
        // for(i=0;i<7;i++) 	{ UI_JULI[i]=	UI_JULI[i+1]; }
        // n++; if (n&gt;2){break;} //Blank 3 bits at most,
        // }

        for(i = 0; i < 3; i++) 	 // 0 is replaced by a space
        {
            if (UI_JULI[i] == '0')
            {
                UI_JULI[i] = ' ';
            }
            else
            {
                break;
            }
        }

    }

// n=0;
// while(UI_JULI[0]==&#39; &#39;) //The first 0 is blanked and filled with spaces
// {
// for(i=0;i<7;i++) 	{ UI_JULI[i]=	UI_JULI[i+1]; }
// n++; if (n&gt;2){break;} //Blank 3 bits at most,
// }
}







float GetAngle(float x1, float y1, float x2, float y2)
{
    /*float angle ;
    float deltaX = x2 - x1 ;
    float deltaY = y2 - y1 ;
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY) ;
    angle = acosf(deltaX/distance) ;
    if(deltaY>0)
    {
    	angle = 3.14 + (3.14 - angle) ;
    }*/


    float angle ;
    angle = atan2(x2 - x1, y2 - y1) ;
    angle = -(angle * 180.0f) / 3.141592f ;
    angle += 90.0f ;

    if(angle < 0.0f)	angle += 360.0f ;

    return angle ;
}


float GET_Angle(uchar *a_lat, uchar *a_lng, uchar *b_lat, uchar *b_lng )
{
    // --------------------------------------------
    my_lat = GET_WD(a_lat);	// Convert point A&#39;s latitude, text, degrees to degrees
    // --------------------------------------------
    my_lng = GET_JD(a_lng); // Convert point A&#39;s longitude, text, degrees to minutes
    // --------------------------------------------
    you_lat = GET_WD(b_lat); // Convert the latitude of point B, text, degrees to degrees
    // --------------------------------------------
    you_lng = GET_JD(b_lng);	// Convert the longitude of point B, text, degrees to minutes
    // --------------------------------------------
    return GetAngle(my_lat, my_lng, you_lat, you_lng);	  // Returns an angle between 0 and 359 degrees
}



uint GET_AB_Angle(uchar mode)	  // Find the horizontal angle of the longitude and latitude of two points relative to due north
{
    uint Angle;

    // --------------------------------------------
    // MODE=0 Find the angle between the fixed station and the other two locations
    if (mode == 0)
    {
        Angle = (uint) GET_Angle(EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30, UI_WD, UI_JD);
    }

    // MODE=1 Find the angle between the mobile station and the other party
    if (mode == 1)
    {
        Angle = (uint)GET_Angle(GPS_WD, GPS_JD, UI_WD, UI_JD);
    }

    // MODE=2 Find the angle between GPS and fixed station
    if (mode == 2)
    {
        Angle = (uint) GET_Angle(GPS_WD, GPS_JD, EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30);
    }



    if (Angle > 359)
    {
        Angle = 0;
    }

    return Angle;
}













uint GET_EL(float Distance_AB_MI, float MY_ALT_MI,  float Target_ALT_MI   )	 // Find the elevation angle of 2 points 0-89 degrees
{
// UART0_DEBUG( (uint) (atan(3)*180/PI ) );
// UART0_DEBUG( (uint) (atan(0)*180/PI ) );
    if (Target_ALT_MI < MY_ALT_MI)
    {
        return 0;   // If the beacon altitude is lower than the fixed/mobile station altitude, the elevation angle is returned to 0
    }

    return     (uint)(atan((Target_ALT_MI - MY_ALT_MI) / Distance_AB_MI) * 180 / 3.1415926)	;
}

uint GET_AB_EL(uchar mode)	   // Find the elevation angle of 2 points 0-89 degrees
{
    uint Angle;

    // --------------------------------------------
    // MODE=0 Convert the angle and distance between the fixed station and the other party into integers
    if (mode == 0)
    {
        Angle = GET_EL((ulong)Distance_AB, EEPROM_Buffer[0x0129] * 256 + EEPROM_Buffer[0x012A], UI_ALT_MI);
    }

    // MODE=1 Find the angle between the mobile station and the other party
    if (mode == 1)
    {
        Angle = GET_EL((ulong)Distance_AB, GPS_ALT_MI, UI_ALT_MI);
    }

    // MODE=2 Find the angle between GPS and fixed station
    if (mode == 2)
    {
        Angle = GET_EL((ulong)Distance_AB, GPS_ALT_MI, EEPROM_Buffer[0x0129] * 256 + EEPROM_Buffer[0x012A]);
    }

    if (Angle > 90)
    {
        Angle = 90;
    }



    return Angle;
}




uint GET_AB_POINT_DIR(uint A_POINT, uint GPS_DIR)		// Relative direction of vehicle head
{
    if  (A_POINT > GPS_NOW_DIR)
    {
        return A_POINT - GPS_DIR;
    }
    else
    {
        return 360 - (GPS_DIR - A_POINT);
    }
}

void angle_to_txt(uint angle, uchar *p)
{
    uint  dat;
    dat = angle;

    if (EEPROM_Buffer[0x0012B] == 0)
    {
        if (dat < 23)
        {
            *(p + 0) = 'N';  	   // NW point orientation
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 68)
        {
            *(p + 0) = 'N';
            *(p + 1) = 'E';
            *(p + 2) = 0;
            return;
        }

        if (dat < 113)
        {
            *(p + 0) = 'E';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 158)
        {
            *(p + 0) = 'S';
            *(p + 1) = 'E';
            *(p + 2) = 0;
            return;
        }

        if (dat < 203)
        {
            *(p + 0) = 'S';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 248)
        {
            *(p + 0) = 'S';
            *(p + 1) = 'W';
            *(p + 2) = 0;
            return;
        }

        if (dat < 293)
        {
            *(p + 0) = 'W';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 338)
        {
            *(p + 0) = 'N';
            *(p + 1) = 'W';
            *(p + 2) = 0;
            return;
        }

        if (dat < 360)
        {
            *(p + 0) = 'N';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        *(p + 0) = '-';
        *(p + 1) = '-';
        *(p + 2) = 0;
        return;
    }

    if (EEPROM_Buffer[0x0012B] == 1)
    {
        dat = dat / 30.0 + 0.5;			 // 00-12 o&#39;clock position
        *(p + 0) = dat / 10 + 0x30;
        *(p + 1) = dat % 10 + 0x30;
        *(p + 2) = 0;
        return;
    }

    if (EEPROM_Buffer[0x0012B] == 2)
    {
        dat = dat / 10.0 + 0.5;			 // 00-36 o&#39;clock position
        *(p + 0) = dat / 10 + 0x30;
        *(p + 1) = dat % 10 + 0x30;
        *(p + 2) = 0;
        return;
    }

    *(p + 0) = '-';
    *(p + 1) = '-';
    *(p + 2) = 0; 	 // Setting error, display --, reinitialize
    EEPROM_Buffer[0x0012B] = 1;
    EEPROM_UPDATA();  	// If not initialized, update the settings

}