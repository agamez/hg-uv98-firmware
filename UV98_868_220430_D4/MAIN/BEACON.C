#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "tostring.H"

#include "UART2.H"
#include "UART4.H"

#include "IO.H"
#include "ADC.H"

#include "GPS2.H"

#include "MICE_CODE.H"

#include "CMX865A_CODE.H"
#include "BEACON.H"
#include "DIGI.H"

#include "DELAY.H"
#include "CHx.H"

#include "bmp280.h"

#include "PUBLIC_BUF.H"


uint GPS_BEACON_TIME; 		   // GPS Timing Beacon Timing



// Gateway radio RF beacon, KISS format, excluding C0 00 ..C0, excluding checksum, with end character 0x00 added at the end, end character 0X00 is not sent
// unsigned char xdata Gate_beacen[128]=
// { 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, // target address + SSID
// 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF4, // Source address + SSID
// 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, // Path + SSID
// 0x03, 0xF0, 											//	03f0
// 0x21, // Type
// 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, // longitude and latitude
// 0x2F, // separator &quot;/&quot;
// 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, // longitude and latitude
// 0x3E, // Icon type
// 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74, // information
// 0x00, }; //The compiler will not add 00H to the end of the non-text string, you need to add it manually, and the end character 0X00 is not sent

// Gateway Network IS Beacon
// BH4TDV-10>APET51:!3134.31N/12020.22E>000/000/A=000027	  //14+22  22x833x8	=147ms
// BH4TDV-10&gt;SYUUQ1:`,&lt;|l+Z[/`&quot;4D} //14 bytes

uchar GPS_TO_KISS()		// 0 = Gateway beacon KISS, 1 = Weather interface board KISS, 2 = WS1 weather beacon KISS, 3 = GPS beacon KISS // GPS to KISS data
{
    uchar i, k, temp;
    uchar MICE_EN;
    long ALT_TEMP;

    uchar B_SSID, PATH1, PATH2;

// uchar code WX_DST_NAME[] ={&quot;WX51 &quot;}; //6 bits of target address
    uchar code   DST_NAME[]    = {"APUV98"} ; // Destination address 6 bits
// uchar code KISS_WIDE1[] ={&quot;WIDE1 &quot;}; //6-digit path address, including spaces
// uchar code KISS_WIDE2[] ={&quot;WIDE2 &quot;}; //6-digit path address, including spaces
// flying code LOGO[] ={&quot;51G3&quot;} ;

// BH4TDV-10>APET51:!3134.31N/12020.22E>000/000/A=000027	  //14+22  22x833x8	=147ms
// BH4TDV-10&gt;SYUUQ1:`,&lt;|l+Z[/`&quot;4D} //14 bytes

    MICE_EN = EEPROM_Buffer[0X10];

    if (MICE_EN == 1)
    {
        Encoding_MICE();
    }


    k = 0;

    if (EEPROM_Buffer[0X2A] == 1) // 0 = Fixed station, set longitude and latitude 1 = Mobile station, GPS longitude and latitude
    {
        if (MICE_EN == 1)
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k] = (MICE_WD[i] << 1 );     // Convert DST_NAME
                k++;
            }
        }
        else
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k] = (DST_NAME[i] << 1 );     // Convert DST_NAME
                k++;
            }
        }
    }
    else
    {
        for (i = 0; i < 6; i++)
        {
            KISS_DATA[k] = (DST_NAME[i] << 1 );     // Convert DST_NAME
            k++;
        }
    }


    KISS_DATA[k] = 0x60;
    k++;	// Insert DST_NAME SSID or WX_DST_NAME SSID
    // --------------------------------

    for (i = 0; i < 6; i++)
    {
        KISS_DATA[k + i]	= 0x40;    // Source Call Sign Filler
    }

    for (i = 0; i < 6; i++)  					  	// Source Call Sign Filler
    {
        if (EEPROM_Buffer[0x08 + i] == 0x00)
        {
            break;
        }

        KISS_DATA[k] = EEPROM_Buffer[0x08 + i] << 1;
        k++;
    }

    // --------------------------------

    B_SSID = EEPROM_Buffer[0X0F];	 // Beacon SSID

    PATH1 = EEPROM_Buffer[PATH1_COUNT]; // Insert forwarding path 1
    PATH2 = EEPROM_Buffer[PATH2_COUNT]; // Insert forwarding path 2

    k = 13;

    if ((PATH1 == 0) && (PATH2 == 0))
    {
        KISS_DATA[k] = 0x60 | (B_SSID << 1) | 0x01;    // Insert path terminator = 1
        k++;
    }
    else
    {
        KISS_DATA[k] = 0xE0 | (B_SSID << 1) ;
        k++;	  // Insert Source SSID + path terminator = 0, high bit = 1

        if (PATH1 != 0)
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k + i]	= 0x40;    // Conversion path WIDE1
            }

            for (i = 0; i < 6; i++)  		// Source Call Sign Filler
            {
                if (EEPROM_Buffer[PATH1_NAME + i] == 0x00)
                {
                    break;
                }

                KISS_DATA[k + i] = EEPROM_Buffer[PATH1_NAME + i] << 1;
            }

            k = k + 6;
            KISS_DATA[k] = 0x60 | (EEPROM_Buffer[PATH1_COUNT] << 1);  	// Insert WIDE1 SSID=1 + path terminator=0 or 1

            if(PATH2 == 0)
            {
                KISS_DATA[k] |= 0x01;    // Insert path terminator = 1
                k++;
            }
            else
            {
                k++;
            }
        }


        if (PATH2 != 0)
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k + i]	= 0x40;    // Conversion path WIDE1
            }

            for (i = 0; i < 6; i++)  		// Source Call Sign Filler
            {
                if (EEPROM_Buffer[PATH2_NAME + i] == 0x00)
                {
                    break;
                }

                KISS_DATA[k + i] = EEPROM_Buffer[PATH2_NAME + i] << 1;
            }

            k = k + 6;
            KISS_DATA[k] = 0x60 | (EEPROM_Buffer[PATH2_COUNT] << 1) | 0x01;
            k++;	// Insert WIDE2 SSID=1 + Path terminator=1
        }

    }

    KISS_DATA[k] = 0x03 ;
    k++;	// Insert control code 03
    KISS_DATA[k] = 0xF0 ;
    k++;	// Insert PRO code F0


// for(i=0;i<18;i++)  { KISS_DATA[k]= GPS_DATA[i]; k++;} //插入GPS经纬度
// The latitude and longitude data extracted from GPS, length = 18 bytes, format: 3134.03N/12020.19E


    if (EEPROM_Buffer[0X2A] == 1) // 0 = Fixed station, set longitude and latitude 1 = Mobile station, GPS longitude and latitude
    {

        if (MICE_EN == 1)
        {
            for (i = 0; i < 14; i++)
            {
                KISS_DATA[k]	= (MICE_JD[i] );     // Compressing Data
                k++;
            }

// for (i=0;i<4;i++)  	{  	KISS_DATA[k]	=LOGO[i];  k++; 	}  	//LOGO
        }
        else
        {
            KISS_DATA[k] = EEPROM_Buffer[0X12];
            k++;	// Insert type symbol, 31 bytes

            // GPS original latitude or GPS corrected latitude
            for(i = 0; i < 8; i++)
            {
                KISS_DATA[k] = GPS_WD[i];
                k++;
            }

            // Insert Icon Set
            if (SMART_MODE == 2)
            {
                KISS_DATA[k] = EEPROM_Buffer[0XBC];
                k++;
            }
            else
            {
                KISS_DATA[k] = EEPROM_Buffer[0X13];
                k++;
            }

// KISS_DATA[k]=EEPROM_Buffer[0X13]; k++;
            // GPS original longitude or GPS corrected longitude
            for(i = 0; i < 9; i++)
            {
                KISS_DATA[k] = GPS_JD[i];
                k++;
            }

            // Insert Icon Symbol
            if (SMART_MODE == 2)
            {
                KISS_DATA[k] = EEPROM_Buffer[0XBD];
                k++;
            }
            else
            {
                KISS_DATA[k] = EEPROM_Buffer[0X14];
                k++;
            }

// KISS_DATA[k]=EEPROM_Buffer[0X14]; k++;

            tostring(GPS_NOW_DIR);	  // course
            KISS_DATA[k] = bai;
            k++;
            KISS_DATA[k] = shi;
            k++;
            KISS_DATA[k] = ge;
            k++;
            KISS_DATA[k] = '/';
            k++;

            tostring(GPS_NOW_SPEED_KNOT);	  // speed
            KISS_DATA[k] = bai;
            k++;
            KISS_DATA[k] = shi;
            k++;
            KISS_DATA[k] = ge;
            k++;
            // Insert altitude, 9 bytes in total // /A=000027
            KISS_DATA[k] = '/';
            k++;
            KISS_DATA[k] = 'A';
            k++;
            KISS_DATA[k] = '=';
            k++;

            for(i = 0; i < 6; i++)
            {
                KISS_DATA[k] = GPS_HEIGHT[i];
                k++;
            }
        }
    }
    else	// 0 = Fixed station, set longitude and latitude, beacon uses the set fixed longitude and latitude to send
    {
        KISS_DATA[k] = EEPROM_Buffer[0X12];
        k++;	// Insert type symbol, 31 bytes

        // Fixed site latitude
        for(i = 0; i < 8; i++)
        {
            KISS_DATA[k] = EEPROM_Buffer[0x20 + i];
            k++;
        }

        KISS_DATA[k] = EEPROM_Buffer[0X13];
        k++;	// Insert Icon Set

        for(i = 0; i < 9; i++)
        {
            KISS_DATA[k] = EEPROM_Buffer[0x30 + i];     // Fixed site longitude
            k++;
        }

        KISS_DATA[k] = EEPROM_Buffer[0X14];
        k++;	// Insert Icon Symbol


        tostring(0);	  // course
        KISS_DATA[k] = bai;
        k++;
        KISS_DATA[k] = shi;
        k++;
        KISS_DATA[k] = ge;
        k++;
        KISS_DATA[k] = '/';
        k++;

        tostring(0);	  // speed
        KISS_DATA[k] = bai;
        k++;
        KISS_DATA[k] = shi;
        k++;
        KISS_DATA[k] = ge;
        k++;
        // Insert altitude, 9 bytes in total // /A=000027
        KISS_DATA[k] = '/';
        k++;
        KISS_DATA[k] = 'A';
        k++;
        KISS_DATA[k] = '=';
        k++;

        ALT_TEMP =	(float) ( EEPROM_Buffer[0x0129] * 256 + EEPROM_Buffer[0x012A]) * 3.2808;	 // Convert to Imperial

        GPS_HEIGHT[0] = ALT_TEMP / 100000 % 10 + 0x30;
        GPS_HEIGHT[1] = ALT_TEMP / 10000 % 10 + 0x30;
        GPS_HEIGHT[2] = ALT_TEMP / 1000 % 10 + 0x30;
        GPS_HEIGHT[3] = ALT_TEMP / 100 % 10 + 0x30;
        GPS_HEIGHT[4] = ALT_TEMP / 10 % 10 + 0x30;
        GPS_HEIGHT[5] = ALT_TEMP % 10 + 0x30;
        GPS_HEIGHT[6] = 0x00; // End symbol

        for(i = 0; i < 6; i++)
        {
            KISS_DATA[k] = GPS_HEIGHT[i];
            k++;
        }

    }

    for(i = 0; i < 60; i++) // Insert custom information and limit the length of custom information
    {
        temp = EEPROM_Buffer[0x40 + i];

        if (temp == 0x00)
        {
            break;
        }

        KISS_DATA[k] = temp;
        k++;

        if (k > 100)
        {
            KISS_DATA[k] = '~';      // Limit character length
            k++;
            break;
        }
    }


    GET_LC();	  // Mileage calculation

    if ((EEPROM_Buffer[0X3E] == 1) && (EEPROM_Buffer[0X2A] == 1) )  	 // Insert mileage 5 bytes 0-650.00km //moto 1-4
    {

        KISS_DATA[k] = ' ';
        k++;	  // KISS_DATA[k]=LC_TIME;   k++;
        tostring((uint)(TOTAL_LC / 100));
        KISS_DATA[k] = wan ;
        k++;
        KISS_DATA[k] = qian ;
        k++;
        KISS_DATA[k] = bai ;
        k++;
        KISS_DATA[k] = shi ;
        k++;
        KISS_DATA[k] = '.';
        k++;
        KISS_DATA[k] = ge ;
        k++;

        KISS_DATA[k] = 'K';
        k++;
        KISS_DATA[k] = 'm';
        k++;
    }

    // ---------------------------------
    if (EEPROM_Buffer[0X3C] == 1)   	// Insertion voltage
    {
        KISS_DATA[k] = ' ';
        k++; // Insert a space 6 bytes
        READ_ADC();	// Read voltage value
        i = 0;

        while (DY[i] != 0x00)
        {
            KISS_DATA[k] = DY[i];    // Insertion voltage
            i++;
            k++;
        }

        KISS_DATA[k] = 'V';
        k++;
    }

// if (EEPROM_Buffer[0X3D]==1) //Insert temperature
// {
// if (DS18B20_READTEMP()==1)
// {
// i=0;
// while (DS18B20_TEMP[i]!=0x00) {KISS_DATA[k]=DS18B20_TEMP[i]; i++; k++;} //Insert temperature
// KISS_DATA[k]='C';   k++;
// }
// }


// 
// if (EEPROM_Buffer[0X3D]==1)
// {
// //Insert DS18B20
// if (DS18B20_READTEMP()==1) //If DS18B20 is detected, insert the second temperature 6 bytes
// {
// KISS_DATA[k]=&#39; &#39;; k++; //Insert a space
// i=0;
// while (DS18B20_TEMP[i]!=0x00) {KISS_DATA[k]=DS18B20_TEMP[i]; i++; k++;} //Insert temperature
// 
// KISS_DATA[k]='C';   k++;
// }
// }

// 
// if (EEPROM_Buffer[0X3D]==1)
// {
// //Insert DS18B20
// if (read_sht2x()==1) //If sht2x is detected, insert the second temperature 6 bytes
// {
// KISS_DATA[k]=&#39; &#39;; k++; //Insert a space
// i=0;
// while (SHT2X_DATA[i]!=0x00) {KISS_DATA[k]=SHT2X_DATA[i]; i++; k++;} //Insert temperature
// 
// }
// }
// 



    READ_BMP280();

    // ---------------------------------
    if (EEPROM_Buffer[0x003D] == 1) 		// Reporting Temperature
    {
        if( BMP280_LINK == 1)
        {
            KISS_DATA[k] = ' ';
            k++;
            i = 0;

            while (BMP280_TEMP[i] != 0x00)
            {
                KISS_DATA[k] = BMP280_TEMP[i];    // Insertion temperature
                i++;
                k++;
            }

            KISS_DATA[k] = 'C';
            k++;
        }

    }

    // ---------------------------------

    if (EEPROM_Buffer[0x003b] == 1)
    {
        if( BMP280_LINK == 1)	// 0 = Not installed 1 = Installed // Air pressure (0.1 hpa) 9 bytes
        {
            KISS_DATA[k] = ' ';
            k++;

            i = 0;

            while (BMP280_QY[i] != 0x00)
            {
                KISS_DATA[k] = BMP280_QY[i];    // Insert air pressure
                i++;
                k++;
            }

            KISS_DATA[k] = 'h';
            k++;
            KISS_DATA[k] = 'P';
            k++;
            KISS_DATA[k] = 'a';
            k++;
        }
    }




    if ((EEPROM_Buffer[0X2A] == 1) && (EEPROM_Buffer[0X3F] == 1)) // 0 = Fixed station, set longitude and latitude 1 = Mobile station, GPS longitude and latitude 4 bytes
    {
        // ---------------------------------
        KISS_DATA[k] = ' ';
        k++;   // Insert the number of satellites to locate
        KISS_DATA[k] = 'S';
        k++;
        KISS_DATA[k] = GPS_SIG[0];
        k++;
        KISS_DATA[k] = GPS_SIG[1];
        k++;
    }


    KISS_LEN = k;
    return 1;
}






void BEACON_GPS_TXD()	 // Send mobile/fixed station RF beacon
{
    // If the device is in power saving mode and GPS is off, turn on GPS.
    if ((EEPROM_Buffer[0X2D] == 1) && (GPS_EN == 0))
    {
        GPS_EN = 1;
        UART2_SendString("GPS ON\r\n");
        return;
    }

    if (EEPROM_Buffer[0X2A] == 1)
    {
        if (GPS_LOCKED == 0)
        {
            UART2_SendString("Wait GPS Lock \r\n");    // When moving the station, the GPS must be locked before transmission is allowed
            return;
        }
    }

    GPS_TO_KISS();	   // Beacon data combination

    BEACON_TX_CHX(0);
}




void BECON_MODE()
{
    if (EEPROM_Buffer[0X02] == 1)			// Timing Beacon
    {
        if (GPS_BEACON_TIME > (EEPROM_Buffer[0X00] * 256 +	EEPROM_Buffer[0X01]) - 1)
        {
            GPS_BEACON_TIME = 0;
            BEACON_GPS_TXD();	 	 // Send mobile/fixed station RF beacon
        }
    }

    // GPS power saving, automatically turns off after transmitting beacon
    if ((EEPROM_Buffer[0X03] == 1) | (EEPROM_Buffer[0X02] == 1))	// When the timed transmission or manual transmission mode is turned on
    {
        // When the mobile station is in operation, when the GPS power saving function is turned on, when the GPS is effectively positioned, a beacon is sent and the GPS is turned off.
        if ((EEPROM_Buffer[0X2D] == 1) && (EEPROM_Buffer[0X2A] == 1) && (GPS_EN == 1) && (GPS_LOCKED == 1))
        {
            BEACON_GPS_TXD();
            GPS_EN = 0;
            GPS_LOCKED = 0; // LED_STU=1;
            UART2_SendString("GPS OFF\r\n");
        }
    }
}

