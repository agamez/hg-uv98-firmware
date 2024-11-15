#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

// #include "UART1.H"
#include "UART2.H"
#include "UART4.H"

#include "IO.H"
#include "tostring.H"

#include "BEACON.H"

#include "AT24C512.h"
#include "FUN_B.h"

#include "PUBLIC_BUF.H"
#include "GPS_JULI.H"
#include "GPS2.H"
#include "KISS_Analysis.H"

#include  <string.H> // Keil library



unsigned char  FUN_B_BUF[50] ;		  // Buffer size

void disp_fw()	   // Display true north
{
    uint dat;

    if (ASC_TEMP[0] == 0)
    {
        strcat(SN_RX_BUFFER, "--");
        return;
    }

    dat = (uint)(ASC_TEMP[0] - 0x30) * 100 + (uint)(ASC_TEMP[1] - 0x30) * 10 + (uint)(ASC_TEMP[2] - 0x30);

    // List direction display mode 0=English 1=0-12 2-0-36
    angle_to_txt(dat, ASC_TEMP);
    strcat(SN_RX_BUFFER, ASC_TEMP);

}
uchar FUN_B_GPS(uint idx)	 	// Real-time display of distance and bearing, list $B00\r\n-$B19\r\n
{
    uint i, k;
    uint  add;
    uchar temp[20];
    uchar dat;	// uchar len;
    uchar no_sig;
// flying *p;

    if (idx > 19)
    {
        return 0;
    }

    // ================================================= Insert page numbers 00-19
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$B");
    temp[0] = idx / 10 + 0x30;
    temp[1] = idx % 10 + 0x30;
    temp[2] = 0;
    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    // ==============================================
    for(i = 0; i < 5; i++)
    {
        FUN_B_BUF[i] = AT24CXX_READ(0x6500 + (idx * 5) + i);
    }

// UART1_SendString(&quot;read a: &quot;); DEBUG_KISS(FUN_B_BUF,5); //debugging
    // The starting address is 0X6500, a total of 100 bytes, each byte is a segment, and each page has 5 indexes
    // ==============================================
    // for (i=0;i<20;i++)  		{  call[i]=0; }  	//Source呼号填充空格
    for(k = 0; k < 5; k++)
    {
        if (FUN_B_BUF[k] > 99)
        {
            strcat(SN_RX_BUFFER, "------ -- ---.- --,");	   // 24C512 did not write data
        }
        else
        {
            // ==============================================
            add =   FUN_B_BUF[k] * 256; // The list starts at address 0X6500, and the KISS data storage index is obtained.
            // Read out call sign, distance

// for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(add+16+i); temp[i+1]=' '; temp[i+2]=' ';temp[i+3]=0; } 	  //呼号

            for (i = 0; i < 9; i++)  	 // Read out the call sign, always 9 digits, less than 9 digits will be filled with spaces
            {
                dat = AT24CXX_READ(add + 16 + i);

                if(dat == 0)
                {
                    dat = ' ';   // Replace with spaces
                }

                temp[i] = dat;
                temp[i + 1] = 0;
            }

            strcat(SN_RX_BUFFER, temp);

            // ==============================================
            strcat(SN_RX_BUFFER, " ");

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

            no_sig = 1;

            if ((UI_WD[0] == 0) | (UI_JD[0] == 0) | (GPS_LOCKED == 0))
            {
                no_sig = 0;    // Check if the beacon contains latitude and longitude data
            }

            if(no_sig == 1)
            {
// 
// UART2_SendString(&quot;===========\r\n&quot;);
// UART2_SendString(GPS_WD); UART2_SendString(&quot; &quot;);
// UART2_SendString(GPS_JD); UART2_SendString(&quot;\r\n&quot;);
// UART2_SendString(UI_WD); UART2_SendString(&quot; &quot;);
// UART2_SendString(UI_JD); UART2_SendString(&quot;\r\n&quot;);


                GET_AB_JULI(1);	  // MODE=1 Calculate the distance between the local GPS position and the other party, with an accuracy of 1 meter

// UART2_SendString(GPS_WD); UART2_SendString(&quot; &quot;);
// UART2_SendString(GPS_JD); UART2_SendString(&quot;\r\n&quot;);
// UART2_SendString(UI_WD); UART2_SendString(&quot; &quot;);
// UART2_SendString(UI_JD); UART2_SendString(&quot;\r\n&quot;);
// 
// UART2_SendString(UI_JULY); UART2_SendString(&quot;\r\n&quot;);
// UART2_SendString(&quot;===========\r\n&quot;);

                UI_Angle_N =  GET_AB_Angle(1); // Find the angle between the mobile station and the other party
                UI_Angle_CAR =   GET_AB_POINT_DIR(UI_Angle_N, GPS_NOW_DIR)	;	// Relative direction of vehicle head
// Elevation_angle=GET_AB_EL(1); //Relative elevation angle
            }



// for (i = 0; i<6; i++) { temp[i]=AT24CXX_READ(add+64+i); temp[i+1]=0; }
// 
// if (temp[0]==0){strcat(SN_RX_BUFFER,&quot;------&quot;);}else //Altitude

            // ==============================================
            if(no_sig == 1)
            {
                strcat(SN_RX_BUFFER, UI_JULI + 1);	 // Display real-time distance
            }
            else
            {
                strcat(SN_RX_BUFFER, "---.-");  	// Distance display----.-
            }

            // ==============================================
            strcat(SN_RX_BUFFER, " ");

            // ==============================================
            // ================================================= //Relative north direction
            if(no_sig == 1)			  // Check if the beacon contains latitude and longitude data
            {
                tostring(UI_Angle_N);
                ASC_TEMP[0] = bai;
                ASC_TEMP[1] = shi;
                ASC_TEMP[2] = ge;
                ASC_TEMP[3] = 0;		 // Relative North
            }
            else
            {
                ASC_TEMP[0] = 0;
            }

            disp_fw();	   // Display true north
            strcat(SN_RX_BUFFER, ",");

// for (i = 0; i<16; i++) { UART2_SendData(AT24CXX_READ(add+0x20+i));   } 	  //调试距离
// 33 31 31 31 2E 30 31 4E 00 31 32 30 2E 33 00 00
// 33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
// 33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
// 33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
// 33 31 30 30 2E 30 30 4E 00 38 39 2E 34 00 00 00

// for(i=0;i<6;i++)	 {call[i]=' ';	 } //填充6个空格呼号
// for(i=0;i<6;i++)
// {
// if (FUN_B_BUF[k*8+i]==0){break;} //Exit when encountering 0
// call[i]=FUN_B_BUF[k*8+i];
// } //Fill in call sign
// call[6]='-';
// //================================================ //Fill in SSID
// SSID=FUN_B_BUF[k*8+6];	call[7]=SSID/10+0x30;  call[8]=SSID%10+0x30; if(call[7]=='0'){call[7]=' ';}
// call[9]=0;
        }
    }

    strcat(SN_RX_BUFFER, "\r\n");


    UART4_SendString(SN_RX_BUFFER);	    // UART2_SendString(SN_RX_BUFFER);


    return 1;


}


uchar FUN_B(uint idx)	 	// Displays a list of memorized distances and bearings $B00\r\n-$B19\r\n
{
    uint i, k;
    uint  add;
    uchar temp[20];
    uchar dat;	// uchar len;
// flying *p;


// EEPROM_write_one(0x00B6, 0); //0=passive navigation 1=dynamic navigation
    if ( EEPROM_Buffer[0XB6] == 1)
    {
        FUN_B_GPS( idx);
        return 0;
    }

    // ==============================================
// p=strstr(UART4_BUF_DATA,"$B");
// if  (p>0)
// {
// if (UART4_BUF_LENTH==6) //
// {
// idx=(UART4_BUF_DATA[2]-0X30)*10+(UART4_BUF_DATA[3]-0X30);

    if (idx > 19)
    {
        return 0;
    }

    // ================================================= Insert page numbers 00-19
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$B");
    temp[0] = idx / 10 + 0x30;
    temp[1] = idx % 10 + 0x30;
    temp[2] = 0;
    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    // ==============================================
    for(i = 0; i < 5; i++)
    {
        FUN_B_BUF[i] = AT24CXX_READ(0x6500 + (idx * 5) + i);
    }

// UART1_SendString(&quot;read a: &quot;); DEBUG_KISS(FUN_B_BUF,5); //debugging
    // The starting address is 0X6500, a total of 100 bytes, each byte is a segment, and each page has 5 indexes
    // ==============================================
    // for (i=0;i<20;i++)  		{  call[i]=0; }  	//Source呼号填充空格
    for(k = 0; k < 5; k++)
    {
        if (FUN_B_BUF[k] > 99)
        {
            strcat(SN_RX_BUFFER, "------ -- ---.- --,");	   // 24C512 did not write data
        }
        else
        {
            // ==============================================
            add =   FUN_B_BUF[k] * 256; // The list starts at address 0X6500, and the KISS data storage index is obtained.
            // Read out call sign, distance




// for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(add+16+i); temp[i+1]=' '; temp[i+2]=' ';temp[i+3]=0; } 	  //呼号

            for (i = 0; i < 9; i++)  	 // Read out the call sign, always 9 digits, less than 9 digits will be filled with spaces
            {
                dat = AT24CXX_READ(add + 16 + i);

                if(dat == 0)
                {
                    dat = ' ';   // Replace with spaces
                }

                temp[i] = dat;
                temp[i + 1] = 0;
            }

            strcat(SN_RX_BUFFER, temp);

            // ==============================================
            strcat(SN_RX_BUFFER, " ");

            // ==============================================
// for (i = 0; i<6; i++) 	{ temp[i]=AT24CXX_READ(add+41+i);   temp[i+1]=0;} 	    //距离
// 
// switch (temp[0])
// {
// case 0:
// strcat(SN_RX_BUFFER,&quot;----.-&quot;); //strcat(SN_RX_BUFFER,&quot;Km,&quot;); //If there is no distance, then display----.-
// break;
// case &#39;*&#39;: //less than 10,000 meters
// temp[0]=' ';	strcat(SN_RX_BUFFER,temp);	//strcat(SN_RX_BUFFER,"Km,");
// break;
// default:
// strcat(SN_RX_BUFFER,temp);	//strcat(SN_RX_BUFFER,"Km");
// break;
// }

// strcat(SN_RX_BUFFER,"Km,");

            // ==============================================
            for (i = 0; i < 6; i++)
            {
                ASC_TEMP[i] = AT24CXX_READ(add + 41 + i);      // distance
                ASC_TEMP[i + 1] = 0;
            }

            switch (ASC_TEMP[0])
            {
                case 0:
                    strcat(SN_RX_BUFFER, "---.-");	// strcat(SN_RX_BUFFER,&quot;Km,&quot;); //If there is no distance, then display----.-
                    break;

                case '*':	  // Less than 10,000 meters
                    strcat(SN_RX_BUFFER, ASC_TEMP + 1);	// strcat(SN_RX_BUFFER,"Km,");
                    break;

                default:
                    strcat(SN_RX_BUFFER, ASC_TEMP + 1);	// strcat(SN_RX_BUFFER,"Km,");
                    break;
            }

            strcat(SN_RX_BUFFER, " ");

            // ================================================= Direction
            for (i = 0; i < 3; i++)
            {
                ASC_TEMP[i] = AT24CXX_READ(add + 0x1a + i);    // Relative North
                ASC_TEMP[i + 1] = 0;
            }

            disp_fw();	   // Display true north
            strcat(SN_RX_BUFFER, ",");

// for (i = 0; i<16; i++) { UART2_SendData(AT24CXX_READ(add+0x20+i));   } 	  //调试距离
// 33 31 31 31 2E 30 31 4E 00 31 32 30 2E 33 00 00
// 33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
// 33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
// 33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
// 33 31 30 30 2E 30 30 4E 00 38 39 2E 34 00 00 00

// for(i=0;i<6;i++)	 {call[i]=' ';	 } //填充6个空格呼号
// for(i=0;i<6;i++)
// {
// if (FUN_B_BUF[k*8+i]==0){break;} //Exit when encountering 0
// call[i]=FUN_B_BUF[k*8+i];
// } //Fill in call sign
// call[6]='-';
// //================================================ //Fill in SSID
// SSID=FUN_B_BUF[k*8+6];	call[7]=SSID/10+0x30;  call[8]=SSID%10+0x30; if(call[7]=='0'){call[7]=' ';}
// call[9]=0;
        }
    }

    strcat(SN_RX_BUFFER, "\r\n");


    UART4_SendString(SN_RX_BUFFER);	    // UART2_SendString(SN_RX_BUFFER);


    return 1;
// }

    return 0;
}
// 0x0000-0x6400 (0-25600), each beacon occupies 256 bytes, a total of 100 records, index 0-99 is stored at 0x8000

// 0x6500-0x6600 , list storage area, one set of indexes per 1 byte, a total of 100 indexes
// 0-5 bytes call sign BH4TDV
// 6-byte SSID-XX 99
// 7-byte beacon KISS storage address index 0-99